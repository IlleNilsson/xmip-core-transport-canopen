//! A `CANopen` node on an in-process bus: what a test or the loopback puts at
//! the far end so a master can be driven without a drive in the room.
//!
//! Not a device profile. One node holds an object dictionary of byte
//! vectors keyed by index and subindex, obeys NMT, serves SDO uploads and
//! downloads to it — expedited and segmented, with the toggle checked and
//! the aborts `CiA 301` names — takes its first receive PDO into the
//! dictionary when operational, and answers a start with a heartbeat. It
//! is a [`Bus`]: what the master transmits, the node answers, and the answer
//! is what the master receives next.

use std::collections::{HashMap, VecDeque};
use std::sync::{Mutex, PoisonError};
use std::time::Duration;

use can_bus::{Bus, Frame};
use transport::error::Result;

use crate::sdo::{
    ABORT_COMMAND, ABORT_LENGTH, ABORT_NO_OBJECT, ABORT_TOGGLE, CLIENT_BASE, EXPEDITED_DATA,
    SEGMENT_DATA, SERVER_BASE, Sdo,
};

/// The COB-ID of network management: every node listens.
pub const NMT: u32 = 0x000;
/// The COB-ID a node's heartbeat goes under, plus the node.
pub const HEARTBEAT_BASE: u32 = 0x700;
/// The COB-ID of a node's first receive PDO, plus the node.
pub const RPDO1_BASE: u32 = 0x200;
/// The COB-ID of a node's first transmit PDO, plus the node.
pub const TPDO1_BASE: u32 = 0x180;

/// The object the first receive PDO is mapped to: write output 8-bit,
/// subindex 1, as `CiA 401` maps it.
pub const RPDO1_OBJECT: (u16, u8) = (0x6200, 1);

/// The NMT commands a master sends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Start = 0x01,
    Stop = 0x02,
    PreOperational = 0x80,
    Reset = 0x81,
}

/// Where a node is in its NMT state machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    PreOperational,
    Operational,
    Stopped,
}

impl State {
    /// The byte a heartbeat carries.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            Self::PreOperational => 0x7f,
            Self::Operational => 0x05,
            Self::Stopped => 0x04,
        }
    }
}

/// A segmented transfer in progress.
struct Transfer {
    index: u16,
    subindex: u8,
    toggle: bool,
    bytes: Vec<u8>,
    /// How far an upload has been read.
    at: usize,
}

struct Inner {
    state: State,
    dictionary: HashMap<(u16, u8), Vec<u8>>,
    download: Option<Transfer>,
    upload: Option<Transfer>,
    to_master: VecDeque<Frame>,
}

/// One node on the bus.
pub struct Node {
    id: u8,
    inner: Mutex<Inner>,
}

impl Node {
    /// Node `id`, pre-operational, holding device type `0x0000_0000` at
    /// `0x1000:00` and nothing else.
    #[must_use]
    pub fn new(id: u8) -> Self {
        let mut dictionary = HashMap::new();
        dictionary.insert((0x1000, 0), vec![0, 0, 0, 0]);
        Self {
            id,
            inner: Mutex::new(Inner {
                state: State::PreOperational,
                dictionary,
                download: None,
                upload: None,
                to_master: VecDeque::new(),
            }),
        }
    }

    /// Hold `bytes` at `index:subindex`.
    #[must_use]
    pub fn with_object(self, index: u16, subindex: u8, bytes: impl Into<Vec<u8>>) -> Self {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .dictionary
            .insert((index, subindex), bytes.into());
        self
    }

    /// The node's identifier.
    #[must_use]
    pub const fn id(&self) -> u8 {
        self.id
    }

    /// The bytes held at `index:subindex`, as they are now.
    #[must_use]
    pub fn object(&self, index: u16, subindex: u8) -> Option<Vec<u8>> {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .dictionary
            .get(&(index, subindex))
            .cloned()
    }

    /// Where the node is.
    #[must_use]
    pub fn state(&self) -> State {
        self.inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .state
    }

    fn take(&self, frame: &Frame) {
        let mut inner = self.inner.lock().unwrap_or_else(PoisonError::into_inner);
        let id = u32::from(self.id);
        if frame.id == NMT {
            inner.nmt(self.id, &frame.data);
        } else if frame.id == CLIENT_BASE + id {
            let answer = match Sdo::decode(&frame.data, true) {
                Ok(request) => inner.serve(request),
                Err(_) => abort(0, 0, ABORT_COMMAND),
            };
            inner.answer(SERVER_BASE + id, &answer.encode());
        } else if frame.id == RPDO1_BASE + id && inner.state == State::Operational {
            inner.dictionary.insert(RPDO1_OBJECT, frame.data.clone());
        }
    }
}

impl Inner {
    fn nmt(&mut self, id: u8, data: &[u8]) {
        let (Some(&command), Some(&node)) = (data.first(), data.get(1)) else {
            return;
        };
        if node != 0 && node != id {
            return;
        }
        self.state = match command {
            0x01 => State::Operational,
            0x02 => State::Stopped,
            0x80 | 0x81 => State::PreOperational,
            _ => return,
        };
        self.download = None;
        self.upload = None;
        let state = self.state.code();
        self.answer(HEARTBEAT_BASE + u32::from(id), &[state]);
    }

    fn answer(&mut self, id: u32, data: &[u8]) {
        if let Ok(frame) = Frame::new(id, false, data) {
            self.to_master.push_back(frame);
        }
    }

    fn serve(&mut self, request: Sdo) -> Sdo {
        match request {
            Sdo::InitiateDownload {
                index,
                subindex,
                expedited,
                size,
            } => self.open_download(index, subindex, expedited, size),
            Sdo::DownloadSegment { toggle, data, last } => self.segment(toggle, &data, last),
            Sdo::InitiateUpload { index, subindex } => self.open_upload(index, subindex),
            Sdo::UploadSegment { toggle } => self.next_segment(toggle),
            Sdo::Abort { .. } => {
                self.download = None;
                self.upload = None;
                abort(0, 0, ABORT_COMMAND)
            }
            _ => abort(0, 0, ABORT_COMMAND),
        }
    }

    fn open_download(&mut self, index: u16, sub: u8, expedited: Option<Vec<u8>>, size: u32) -> Sdo {
        if !self.dictionary.contains_key(&(index, sub)) {
            return abort(index, sub, ABORT_NO_OBJECT);
        }
        match expedited {
            Some(data) => {
                self.dictionary.insert((index, sub), data);
            }
            None => {
                self.download = Some(Transfer {
                    index,
                    subindex: sub,
                    toggle: false,
                    bytes: Vec::with_capacity(usize::try_from(size).unwrap_or(0)),
                    at: 0,
                });
            }
        }
        Sdo::DownloadAccepted {
            index,
            subindex: sub,
        }
    }

    fn segment(&mut self, toggle: bool, data: &[u8], last: bool) -> Sdo {
        let Some(transfer) = self.download.as_mut() else {
            return abort(0, 0, ABORT_COMMAND);
        };
        if toggle != transfer.toggle {
            let (index, sub) = (transfer.index, transfer.subindex);
            self.download = None;
            return abort(index, sub, ABORT_TOGGLE);
        }
        transfer.toggle = !toggle;
        transfer.bytes.extend_from_slice(data);
        if last {
            let Some(done) = self.download.take() else {
                return abort(0, 0, ABORT_COMMAND);
            };
            self.dictionary
                .insert((done.index, done.subindex), done.bytes);
        }
        Sdo::SegmentAccepted { toggle }
    }

    fn open_upload(&mut self, index: u16, sub: u8) -> Sdo {
        let Some(bytes) = self.dictionary.get(&(index, sub)).cloned() else {
            return abort(index, sub, ABORT_NO_OBJECT);
        };
        let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
        if bytes.len() <= EXPEDITED_DATA && !bytes.is_empty() {
            return Sdo::UploadOpened {
                index,
                subindex: sub,
                expedited: Some(bytes),
                size,
            };
        }
        self.upload = Some(Transfer {
            index,
            subindex: sub,
            toggle: false,
            bytes,
            at: 0,
        });
        Sdo::UploadOpened {
            index,
            subindex: sub,
            expedited: None,
            size,
        }
    }

    fn next_segment(&mut self, toggle: bool) -> Sdo {
        let Some(transfer) = self.upload.as_mut() else {
            return abort(0, 0, ABORT_LENGTH);
        };
        if toggle != transfer.toggle {
            let (index, sub) = (transfer.index, transfer.subindex);
            self.upload = None;
            return abort(index, sub, ABORT_TOGGLE);
        }
        transfer.toggle = !toggle;
        let end = (transfer.at + SEGMENT_DATA).min(transfer.bytes.len());
        let data = transfer.bytes[transfer.at..end].to_vec();
        transfer.at = end;
        let last = end == transfer.bytes.len();
        if last {
            self.upload = None;
        }
        Sdo::UploadData { toggle, data, last }
    }
}

const fn abort(index: u16, subindex: u8, code: u32) -> Sdo {
    Sdo::Abort {
        index,
        subindex,
        code,
    }
}

/// What the master transmits, the node answers; the answers are what the
/// master receives next.
impl Bus for Node {
    fn name(&self) -> &'static str {
        "loopback"
    }

    fn receive(&self, _timeout: Duration) -> Result<Option<Frame>> {
        Ok(self
            .inner
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .to_master
            .pop_front())
    }

    fn transmit(&self, frame: &Frame) -> Result<()> {
        self.take(frame);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn answer(node: &Node, id: u32, data: &[u8]) -> Frame {
        node.transmit(&Frame::new(id, false, data).expect("frame"))
            .expect("transmit");
        node.receive(Duration::ZERO)
            .expect("receive")
            .expect("an answer")
    }

    #[test]
    fn a_start_is_answered_with_a_heartbeat_and_a_pdo_lands_when_operational() {
        let node = Node::new(3);
        assert_eq!(node.state(), State::PreOperational);
        node.transmit(&Frame::new(RPDO1_BASE + 3, false, &[1]).expect("pdo"))
            .expect("transmit");
        assert!(
            node.object(0x6200, 1).is_none(),
            "ignored while pre-operational"
        );
        let heartbeat = answer(&node, NMT, &[Command::Start as u8, 0]);
        assert_eq!(heartbeat.id, HEARTBEAT_BASE + 3);
        assert_eq!(heartbeat.data, [State::Operational.code()]);
        node.transmit(&Frame::new(RPDO1_BASE + 3, false, &[1, 2]).expect("pdo"))
            .expect("transmit");
        assert_eq!(node.object(0x6200, 1), Some(vec![1, 2]));
        node.transmit(&Frame::new(NMT, false, &[Command::Stop as u8, 9]).expect("other"))
            .expect("transmit");
        assert_eq!(
            node.state(),
            State::Operational,
            "addressed to another node"
        );
        assert_eq!(node.id(), 3);
    }

    #[test]
    fn a_download_to_a_missing_object_is_aborted_and_a_bad_toggle_too() {
        let node = Node::new(1).with_object(0x2000, 0, Vec::new());
        let open = Sdo::InitiateDownload {
            index: 0x9999,
            subindex: 0,
            expedited: None,
            size: 1,
        };
        let refused = answer(&node, CLIENT_BASE + 1, &open.encode());
        assert_eq!(refused.id, SERVER_BASE + 1);
        assert_eq!(
            Sdo::decode(&refused.data, false).expect("abort"),
            Sdo::Abort {
                index: 0x9999,
                subindex: 0,
                code: ABORT_NO_OBJECT
            }
        );
        let open = Sdo::InitiateDownload {
            index: 0x2000,
            subindex: 0,
            expedited: None,
            size: 3,
        };
        answer(&node, CLIENT_BASE + 1, &open.encode());
        let wrong = Sdo::DownloadSegment {
            toggle: true,
            data: vec![1],
            last: true,
        };
        let aborted = answer(&node, CLIENT_BASE + 1, &wrong.encode());
        assert!(matches!(
            Sdo::decode(&aborted.data, false).expect("abort"),
            Sdo::Abort {
                code: ABORT_TOGGLE,
                ..
            }
        ));
        let garbage = answer(&node, CLIENT_BASE + 1, &[0xff; 8]);
        assert!(matches!(
            Sdo::decode(&garbage.data, false).expect("abort"),
            Sdo::Abort {
                code: ABORT_COMMAND,
                ..
            }
        ));
    }
}
