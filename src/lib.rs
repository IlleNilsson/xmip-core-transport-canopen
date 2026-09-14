#![forbid(unsafe_code)]

//! Streams that are objects in a `CANopen` node's dictionary. One object —
//! an index and a subindex — is one Stream: reading it is an SDO upload,
//! writing it an SDO download, and a Stream longer than four bytes travels
//! segmented, seven bytes to a frame, the way `CiA 301` says a domain does.
//!
//! `CANopen` is the machine builder's CAN: drives, encoders, valve islands,
//! every one a node with an object dictionary. The identifier does most of
//! the work — a function code in the top four bits and the node in the low
//! seven — so NMT is COB-ID 0, a node's SDO server answers under `0x580`
//! plus its id to requests under `0x600` plus its id, and its first PDOs
//! sit at `0x180` and `0x200`. What is here is the SDO protocol ([`sdo`]),
//! the NMT commands, the first receive PDO at its minimum, and a master
//! that speaks them; [`Node`] is a node on an in-process bus for tests and
//! the loopback. The carrier is `xmip-core-transport-can-bus`: this crate
//! rides its [`Bus`] and [`Frame`] rather than knowing a wire of its own.
//!
//! The origin URI names the bus, the node and the object:
//! `canopen://<bus>/<node>/0x2000/0`. A target is the same, or a bare
//! `<node>/0x<index>/<sub>`, or nothing for the configured object.

pub mod node;
pub mod sdo;

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_bus::{Bus, Frame};
pub use node::{Command, Node, State};
pub use sdo::Sdo;
use transport::error::{Result, protocol_error};
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};
use transport::{Arrived, Directions, Transport};

use crate::sdo::{CLIENT_BASE, EXPEDITED_DATA, SEGMENT_DATA, SERVER_BASE};

/// The object a Stream travels as unless a target says otherwise: the
/// first manufacturer-specific index, a domain.
pub const STREAM_OBJECT: (u16, u8) = (0x2000, 0);

/// The master's side of one bus, addressing one node's one object.
#[derive(Clone)]
pub struct CanOpenTransport {
    bus: Arc<dyn Bus>,
    node: u8,
    index: u16,
    subindex: u8,
    timeout: Duration,
}

impl CanOpenTransport {
    /// A master on `bus` speaking to `node` about [`STREAM_OBJECT`].
    #[must_use]
    pub fn new(bus: Arc<dyn Bus>, node: u8) -> Self {
        Self {
            bus,
            node,
            index: STREAM_OBJECT.0,
            subindex: STREAM_OBJECT.1,
            timeout: Duration::from_secs(1),
        }
    }

    /// Speak about `index:subindex` instead.
    #[must_use]
    pub const fn about(mut self, index: u16, subindex: u8) -> Self {
        self.index = index;
        self.subindex = subindex;
        self
    }

    /// Give up on a node that stops answering.
    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// `canopen://<bus>/<node>/0x<index>/<sub>`.
    #[must_use]
    pub fn origin(&self, node: u8, index: u16, subindex: u8) -> String {
        format!(
            "canopen://{}/{node}/{index:#06x}/{subindex}",
            self.bus.name()
        )
    }

    /// Send `command` to `node`, or to every node with 0.
    ///
    /// # Errors
    /// Where the bus refused the frame.
    pub fn nmt(&self, command: Command, node: u8) -> Result<()> {
        self.bus
            .transmit(&Frame::new(node::NMT, false, &[command as u8, node])?)
    }

    /// Put `data` on the bus as `node`'s first receive PDO: eight bytes at
    /// most, taken only by a node that is operational.
    ///
    /// # Errors
    /// More than eight bytes, or a bus that refused the frame.
    pub fn pdo(&self, node: u8, data: &[u8]) -> Result<()> {
        self.bus.transmit(&Frame::new(
            node::RPDO1_BASE + u32::from(node),
            false,
            data,
        )?)
    }

    /// Write `bytes` to `index:subindex` on `node`: expedited when they fit
    /// four bytes, segmented otherwise.
    ///
    /// # Errors
    /// A node that aborts, answers out of turn, or stops answering.
    pub fn download(&self, node: u8, index: u16, subindex: u8, bytes: &[u8]) -> Result<()> {
        let expedited =
            (!bytes.is_empty() && bytes.len() <= EXPEDITED_DATA).then(|| bytes.to_vec());
        let segmented = expedited.is_none();
        let open = Sdo::InitiateDownload {
            index,
            subindex,
            expedited,
            size: u32::try_from(bytes.len())
                .map_err(|_| protocol_error("over what an SDO sizes"))?,
        };
        match self.request(node, &open)? {
            Sdo::DownloadAccepted { .. } => {}
            other => return Err(unexpected(&other)),
        }
        if !segmented {
            return Ok(());
        }
        let mut toggle = false;
        let chunks: Vec<&[u8]> = if bytes.is_empty() {
            vec![&[]]
        } else {
            bytes.chunks(SEGMENT_DATA).collect()
        };
        let total = chunks.len();
        for (n, chunk) in chunks.into_iter().enumerate() {
            let segment = Sdo::DownloadSegment {
                toggle,
                data: chunk.to_vec(),
                last: n + 1 == total,
            };
            match self.request(node, &segment)? {
                Sdo::SegmentAccepted { toggle: took } if took == toggle => {}
                other => return Err(unexpected(&other)),
            }
            toggle = !toggle;
        }
        Ok(())
    }

    /// Read `index:subindex` from `node`.
    ///
    /// # Errors
    /// A node that aborts, answers out of turn, or stops answering.
    pub fn upload(&self, node: u8, index: u16, subindex: u8) -> Result<Vec<u8>> {
        let ask = Sdo::InitiateUpload { index, subindex };
        let size = match self.request(node, &ask)? {
            Sdo::UploadOpened {
                expedited: Some(data),
                ..
            } => return Ok(data),
            Sdo::UploadOpened { size, .. } => size,
            other => return Err(unexpected(&other)),
        };
        let mut bytes = Vec::with_capacity(usize::try_from(size).unwrap_or(0));
        let mut toggle = false;
        loop {
            match self.request(node, &Sdo::UploadSegment { toggle })? {
                Sdo::UploadData {
                    toggle: got,
                    data,
                    last,
                } if got == toggle => {
                    bytes.extend_from_slice(&data);
                    if last {
                        return Ok(bytes);
                    }
                }
                other => return Err(unexpected(&other)),
            }
            toggle = !toggle;
        }
    }

    /// One SDO request to `node` and its answer.
    fn request(&self, node: u8, request: &Sdo) -> Result<Sdo> {
        let node = u32::from(node);
        self.bus
            .transmit(&Frame::new(CLIENT_BASE + node, false, &request.encode())?)?;
        let deadline = Instant::now() + self.timeout;
        loop {
            if let Some(frame) = self.bus.receive(self.timeout)? {
                if frame.id == SERVER_BASE + node {
                    return Sdo::decode(&frame.data, false);
                }
                continue;
            }
            if Instant::now() >= deadline {
                return Err(protocol_error(
                    "the node did not answer before the deadline",
                ));
            }
            std::thread::yield_now();
        }
    }

    /// The node and object a target names, or the configured ones.
    ///
    /// # Errors
    /// A target that is not `<node>/0x<index>/<sub>`.
    fn resolve(&self, target: &str) -> Result<(u8, u16, u8)> {
        let path = match transport::socket::target("canopen", target) {
            Some((_, path)) => path,
            None => target,
        };
        if path.is_empty() {
            return Ok((self.node, self.index, self.subindex));
        }
        let bad = || protocol_error(format!("{target:?} is not <node>/0x<index>/<sub>"));
        let mut parts = path.split('/');
        let node = parts.next().and_then(|n| n.parse().ok()).ok_or_else(bad)?;
        let index = parts
            .next()
            .and_then(|hex| hex.strip_prefix("0x"))
            .and_then(|hex| u16::from_str_radix(hex, 16).ok())
            .ok_or_else(bad)?;
        let subindex = parts.next().and_then(|n| n.parse().ok()).ok_or_else(bad)?;
        Ok((node, index, subindex))
    }
}

fn unexpected(answer: &Sdo) -> transport::TransportError {
    match answer {
        Sdo::Abort { code, .. } => protocol_error(format!("the node aborted with {code:#010x}")),
        other => protocol_error(format!("the node answered out of turn: {other:?}")),
    }
}

impl Transport for CanOpenTransport {
    fn name(&self) -> &'static str {
        "canopen"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    /// One upload of the object: its bytes as one Stream.
    fn receive(&self) -> Result<Vec<Arrived>> {
        let bytes = self.upload(self.node, self.index, self.subindex)?;
        Ok(vec![Arrived::new(
            self.origin(self.node, self.index, self.subindex),
            bytes,
        )])
    }

    /// One download of `bytes` to the object `target` names.
    fn send(&self, target: &str, bytes: &[u8]) -> Result<()> {
        let (node, index, subindex) = self.resolve(target)?;
        self.download(node, index, subindex, bytes)
    }
}

impl CanOpenTransport {
    /// Both ends on one in-process bus: a master and node 1, started, the
    /// Stream object empty, the loopback timeout on the master.
    #[must_use]
    pub fn loopback() -> Self {
        let node = Node::new(1).with_object(STREAM_OBJECT.0, STREAM_OBJECT.1, Vec::new());
        let master = Self::new(Arc::new(node), 1).timing_out_after(LOOPBACK_TIMEOUT);
        // A start is answered with a heartbeat; nobody is reading, and the
        // SDO client skips what is not its answer.
        drop(master.nmt(Command::Start, 1));
        master
    }
}

/// The node holding what the master wrote, until it is uploaded back.
struct Holding {
    master: CanOpenTransport,
    address: String,
}

impl FarEnd for Holding {
    fn address(&self) -> &str {
        &self.address
    }

    fn take_one(self: Box<Self>) -> Result<Arrived> {
        self.master
            .receive()?
            .into_iter()
            .next()
            .ok_or_else(|| protocol_error("nothing came back from the node"))
    }
}

/// A Stream of any length travels as a segmented domain: the SDO size is
/// thirty-two bits, and no ceiling below that is a fact of the protocol.
impl Loopback for CanOpenTransport {
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        Ok(Box::new(Holding {
            master: self.clone(),
            address: self.origin(self.node, self.index, self.subindex),
        }))
    }

    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        self.send(address, payload)
    }

    fn unblock(&self, _address: &str) {}

    /// In order on one thread: the node answers as the master transmits, so
    /// the download goes first and the upload reads it back.
    fn round(&self, payload: &[u8]) -> Result<Arrived> {
        self.round_in_order(payload)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use transport::payload::edge_payloads;

    #[test]
    fn a_loopback_round_downloads_a_domain_and_uploads_it_back() {
        let loopback = CanOpenTransport::loopback();
        let arrived = loopback.round(b"a segmented domain").expect("round");
        assert_eq!(arrived.bytes, b"a segmented domain");
        assert_eq!(arrived.origin_uri, "canopen://loopback/1/0x2000/0");
        let arrived = loopback.round(b"four").expect("expedited");
        assert_eq!(arrived.bytes, b"four");
        assert!(loopback.ceiling().is_none());
        assert!(loopback.refuses(b"four").is_none());
        assert_eq!(loopback.name(), "canopen");
        assert!(loopback.claims().is_none());
    }

    #[test]
    fn the_loopback_returns_the_edges_whole() {
        let loopback = CanOpenTransport::loopback();
        for (name, bytes) in edge_payloads() {
            let arrived = loopback
                .round(&bytes)
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(arrived.bytes, bytes, "{name}");
        }
    }

    #[test]
    fn a_target_names_the_node_and_object_and_a_missing_one_is_aborted() {
        let node = Arc::new(Node::new(5).with_object(0x2001, 3, vec![0]));
        let master = CanOpenTransport::new(Arc::clone(&node) as Arc<dyn Bus>, 5)
            .timing_out_after(Duration::from_millis(100));
        master
            .send(
                "canopen://loopback/5/0x2001/3",
                &[1, 2, 3, 4, 5, 6, 7, 8, 9],
            )
            .expect("segmented to another object");
        assert_eq!(
            node.object(0x2001, 3),
            Some(vec![1, 2, 3, 4, 5, 6, 7, 8, 9])
        );
        master
            .send("5/0x1000/0", &[7, 0, 0, 0])
            .expect("bare target");
        assert_eq!(node.object(0x1000, 0), Some(vec![7, 0, 0, 0]));
        let error = master.send("", b"x").expect_err("no such object");
        assert!(error.message.contains("0x06020000"), "{error}");
        assert!(master.send("5/2001/3", b"x").is_err(), "not hex");
        assert!(master.send("five/0x2001/3", b"x").is_err(), "not a node");
        assert_eq!(
            master.about(0x2001, 3).receive().expect("upload")[0].bytes,
            [1, 2, 3, 4, 5, 6, 7, 8, 9]
        );
    }

    #[test]
    fn a_pdo_lands_once_the_node_is_started() {
        let node = Arc::new(Node::new(2));
        let master = CanOpenTransport::new(Arc::clone(&node) as Arc<dyn Bus>, 2);
        master.pdo(2, &[0xaa]).expect("pdo");
        assert!(node.object(0x6200, 1).is_none());
        master.nmt(Command::Start, 0).expect("start all");
        assert_eq!(node.state(), State::Operational);
        master.pdo(2, &[0xaa, 0x55]).expect("pdo");
        assert_eq!(node.object(0x6200, 1), Some(vec![0xaa, 0x55]));
        assert!(master.pdo(2, &[0; 9]).is_err(), "nine bytes is no PDO");
        let quiet = CanOpenTransport::new(Arc::new(can_bus::Loopback::new()), 9)
            .timing_out_after(Duration::from_millis(20));
        assert!(quiet.receive().is_err(), "nobody answers on an empty bus");
    }
}
