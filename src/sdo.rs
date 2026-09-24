//! The service data object protocol of `CiA 301` section 7.2.4: what a
//! client and a server exchange to move one object dictionary entry,
//! expedited when it fits in four bytes and segmented when it does not.
//! The one SDO in the estate: `EtherCAT`'s `CoE` is this protocol in a
//! mailbox (ETG.1000.6 section 5.6), and the ethercat technology carries
//! it rather than writing it again.
//!
//! The first byte is the command specifier: the top three bits say which
//! of the eight exchanges this is, and the rest say how many of the data
//! bytes are meaningful, whether the transfer is expedited, whether a size
//! follows, which toggle this segment is under, and whether it is the last.
//! Every request is answered; an abort in either direction ends the transfer
//! with a reason.
//!
//! On CAN an SDO is the eight bytes of one frame. A mailbox stretches it:
//! a sized initiate carries the object's first bytes after its size, and a
//! segment carries as many as the mailbox holds. The codec reads and
//! writes both; a [`Window`] says what a carrier holds, [`client`] moves an
//! object through one and [`server`] answers from a dictionary.

pub mod client;
pub mod server;

use transport::error::{Result, protocol_error};

/// The COB-ID a client sends requests under, plus the node.
pub const CLIENT_BASE: u32 = 0x600;
/// The COB-ID a server answers from, plus the node.
pub const SERVER_BASE: u32 = 0x580;

/// The shortest SDO: a command specifier, index, subindex and four bytes.
pub const SDO_LENGTH: usize = 8;
/// The payload a segment of one CAN frame carries beside its specifier.
pub const SEGMENT_DATA: usize = 7;
/// The payload an expedited transfer carries beside index and subindex.
pub const EXPEDITED_DATA: usize = 4;

/// The object does not exist in the object dictionary.
pub const ABORT_NO_OBJECT: u32 = 0x0602_0000;
/// The toggle bit was not alternated.
pub const ABORT_TOGGLE: u32 = 0x0503_0000;
/// The command specifier is not valid or unknown.
pub const ABORT_COMMAND: u32 = 0x0504_0001;
/// The data type does not match: the length is wrong.
pub const ABORT_LENGTH: u32 = 0x0607_0010;

/// What one SDO carries of an object on one carrier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    /// The most an expedited initiate carries: four, or none where the
    /// carrier's initiate carries more.
    pub expedited: usize,
    /// The most a sized initiate carries after its size: none on CAN.
    pub initiate: usize,
    /// The most one segment carries.
    pub segment: usize,
}

/// The window of one classical CAN frame.
pub const CAN: Window = Window {
    expedited: EXPEDITED_DATA,
    initiate: 0,
    segment: SEGMENT_DATA,
};

/// How an initiate opens a transfer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Opening {
    /// The whole object, one to four bytes, in the initiate itself.
    Expedited(Vec<u8>),
    /// The object's size — 0 where the initiate did not indicate one — and
    /// the first bytes, where the carrier's initiate carries any.
    Sized { size: u32, data: Vec<u8> },
}

/// One SDO exchange, told apart by its command specifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sdo {
    /// A client opens a download.
    InitiateDownload {
        index: u16,
        subindex: u8,
        opening: Opening,
    },
    /// The server accepts the download.
    DownloadAccepted { index: u16, subindex: u8 },
    /// One segment of a download, under a toggle, `last` on the final one.
    DownloadSegment {
        toggle: bool,
        data: Vec<u8>,
        last: bool,
    },
    /// The server took the segment under that toggle.
    SegmentAccepted { toggle: bool },
    /// A client asks for an object.
    InitiateUpload { index: u16, subindex: u8 },
    /// The server answers with how the upload opens.
    UploadOpened {
        index: u16,
        subindex: u8,
        opening: Opening,
    },
    /// A client asks for the next segment under a toggle.
    UploadSegment { toggle: bool },
    /// One segment of an upload.
    UploadData {
        toggle: bool,
        data: Vec<u8>,
        last: bool,
    },
    /// Either side ends the transfer with a reason.
    Abort { index: u16, subindex: u8, code: u32 },
}

impl Sdo {
    /// Whether a client sends this; an abort goes both ways and reads as
    /// the server's.
    #[must_use]
    pub const fn is_request(&self) -> bool {
        matches!(
            self,
            Self::InitiateDownload { .. }
                | Self::DownloadSegment { .. }
                | Self::InitiateUpload { .. }
                | Self::UploadSegment { .. }
        )
    }

    /// The bytes: eight, or more where a stretched initiate or segment
    /// carries more.
    ///
    /// # Errors
    /// An expedited transfer of no bytes or of more than four.
    pub fn encode(&self) -> Result<Vec<u8>> {
        let mut out = vec![0u8; SDO_LENGTH];
        match self {
            Self::InitiateDownload {
                index,
                subindex,
                opening,
            } => initiate(&mut out, 0x20, *index, *subindex, opening)?,
            Self::UploadOpened {
                index,
                subindex,
                opening,
            } => initiate(&mut out, 0x40, *index, *subindex, opening)?,
            Self::DownloadAccepted { index, subindex } => {
                out[0] = 0x60;
                multiplexer(&mut out, *index, *subindex);
            }
            Self::DownloadSegment { toggle, data, last }
            | Self::UploadData { toggle, data, last } => segment(&mut out, *toggle, data, *last),
            Self::SegmentAccepted { toggle } => out[0] = 0x20 | toggle_bit(*toggle),
            Self::InitiateUpload { index, subindex } => {
                out[0] = 0x40;
                multiplexer(&mut out, *index, *subindex);
            }
            Self::UploadSegment { toggle } => out[0] = 0x60 | toggle_bit(*toggle),
            Self::Abort {
                index,
                subindex,
                code,
            } => {
                out[0] = 0x80;
                multiplexer(&mut out, *index, *subindex);
                out[4..8].copy_from_slice(&code.to_le_bytes());
            }
        }
        Ok(out)
    }

    /// The exchange `bytes` carry, read as a request when `request` and as
    /// an answer otherwise — the same specifier bits mean different things
    /// in the two directions.
    ///
    /// # Errors
    /// Fewer than eight bytes, or a specifier neither direction uses.
    pub fn decode(bytes: &[u8], request: bool) -> Result<Self> {
        if bytes.len() < SDO_LENGTH {
            return Err(protocol_error("an SDO shorter than eight bytes"));
        }
        let specifier = bytes[0] >> 5;
        let index = u16::from_le_bytes([bytes[1], bytes[2]]);
        let subindex = bytes[3];
        let toggle = bytes[0] & 0x10 != 0;
        let last = bytes[0] & 0x01 != 0;
        Ok(match (specifier, request) {
            (0, true) => Self::DownloadSegment {
                toggle,
                data: segment_data(bytes),
                last,
            },
            (0, false) => Self::UploadData {
                toggle,
                data: segment_data(bytes),
                last,
            },
            (1, true) => Self::InitiateDownload {
                index,
                subindex,
                opening: opening(bytes),
            },
            (1, false) => Self::SegmentAccepted { toggle },
            (2, true) => Self::InitiateUpload { index, subindex },
            (2, false) => Self::UploadOpened {
                index,
                subindex,
                opening: opening(bytes),
            },
            (3, true) => Self::UploadSegment { toggle },
            (3, false) => Self::DownloadAccepted { index, subindex },
            (4, _) => Self::Abort {
                index,
                subindex,
                code: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            },
            (other, _) => {
                return Err(protocol_error(format!(
                    "an SDO command specifier of {other}"
                )));
            }
        })
    }
}

fn multiplexer(out: &mut [u8], index: u16, subindex: u8) {
    out[1..3].copy_from_slice(&index.to_le_bytes());
    out[3] = subindex;
}

const fn toggle_bit(toggle: bool) -> u8 {
    if toggle { 0x10 } else { 0x00 }
}

/// An initiate under `base`: the e, s and n bits, the multiplexer, and
/// the expedited data or the size and what follows it.
fn initiate(
    out: &mut Vec<u8>,
    base: u8,
    index: u16,
    subindex: u8,
    opening: &Opening,
) -> Result<()> {
    multiplexer(out, index, subindex);
    match opening {
        Opening::Expedited(data) => {
            if data.is_empty() || data.len() > EXPEDITED_DATA {
                return Err(protocol_error("an expedited SDO carries one to four bytes"));
            }
            out[4..4 + data.len()].copy_from_slice(data);
            let unused = u8::try_from(EXPEDITED_DATA - data.len()).unwrap_or(0);
            out[0] = base | 0x03 | (unused << 2);
        }
        Opening::Sized { size, data } => {
            out[0] = base | 0x01;
            out[4..8].copy_from_slice(&size.to_le_bytes());
            out.extend_from_slice(data);
        }
    }
    Ok(())
}

/// A segment: the t, n and c bits, and its data, padded to eight bytes
/// when shorter than seven — n counting the padding — and stretched past
/// them when longer.
fn segment(out: &mut Vec<u8>, toggle: bool, data: &[u8], last: bool) {
    let unused = SEGMENT_DATA.saturating_sub(data.len());
    out.truncate(1);
    out.extend_from_slice(data);
    out.resize(out.len() + unused, 0);
    out[0] = toggle_bit(toggle) | (u8::try_from(unused).unwrap_or(0) << 1) | u8::from(last);
}

fn opening(bytes: &[u8]) -> Opening {
    let expedited = bytes[0] & 0x02 != 0;
    let sized = bytes[0] & 0x01 != 0;
    if expedited {
        let unused = usize::from((bytes[0] >> 2) & 0x03);
        let length = if sized {
            EXPEDITED_DATA - unused
        } else {
            EXPEDITED_DATA
        };
        return Opening::Expedited(bytes[4..4 + length].to_vec());
    }
    let size = if sized {
        u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]])
    } else {
        0
    };
    Opening::Sized {
        size,
        data: bytes[SDO_LENGTH..].to_vec(),
    }
}

/// A segment's data: what follows the specifier, less the unused bytes
/// it counts — at most seven, so never past the eight an SDO has.
fn segment_data(bytes: &[u8]) -> Vec<u8> {
    let unused = usize::from((bytes[0] >> 1) & 0x07);
    bytes[1..bytes.len() - unused].to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round(sdo: &Sdo, request: bool) -> Vec<u8> {
        let bytes = sdo.encode().expect("encode");
        assert_eq!(&Sdo::decode(&bytes, request).expect("decode"), sdo);
        assert_eq!(sdo.is_request(), request);
        bytes
    }

    #[test]
    fn an_expedited_download_carries_its_bytes_in_the_initiate() {
        let sdo = Sdo::InitiateDownload {
            index: 0x1017,
            subindex: 0,
            opening: Opening::Expedited(vec![0xe8, 0x03]),
        };
        assert_eq!(
            round(&sdo, true),
            [0x2b, 0x17, 0x10, 0x00, 0xe8, 0x03, 0, 0]
        );
        let accepted = Sdo::DownloadAccepted {
            index: 0x1017,
            subindex: 0,
        };
        assert_eq!(round(&accepted, false), [0x60, 0x17, 0x10, 0, 0, 0, 0, 0]);
        let none = Sdo::InitiateDownload {
            index: 1,
            subindex: 0,
            opening: Opening::Expedited(Vec::new()),
        };
        assert!(none.encode().is_err(), "an expedited SDO of nothing");
        let five = Sdo::InitiateDownload {
            index: 1,
            subindex: 0,
            opening: Opening::Expedited(vec![0; 5]),
        };
        assert!(five.encode().is_err(), "cut at four until 2026-09-24");
    }

    #[test]
    fn a_segmented_download_says_its_size_then_toggles_its_segments() {
        let open = Sdo::InitiateDownload {
            index: 0x2000,
            subindex: 0,
            opening: Opening::Sized {
                size: 300,
                data: Vec::new(),
            },
        };
        assert_eq!(
            round(&open, true),
            [0x21, 0x00, 0x20, 0x00, 0x2c, 0x01, 0, 0]
        );
        let segment = Sdo::DownloadSegment {
            toggle: true,
            data: vec![1, 2, 3],
            last: true,
        };
        assert_eq!(round(&segment, true), [0x19, 1, 2, 3, 0, 0, 0, 0]);
        let took = Sdo::SegmentAccepted { toggle: true };
        assert_eq!(round(&took, false)[0], 0x30);
    }

    #[test]
    fn a_mailbox_stretches_the_initiate_and_the_segment() {
        let open = Sdo::InitiateDownload {
            index: 0x2000,
            subindex: 0,
            opening: Opening::Sized {
                size: 1000,
                data: vec![7; 240],
            },
        };
        let bytes = round(&open, true);
        assert_eq!(bytes.len(), 248);
        assert_eq!(&bytes[..8], &[0x21, 0x00, 0x20, 0x00, 0xe8, 0x03, 0, 0]);
        let long = Sdo::UploadData {
            toggle: false,
            data: vec![9; 200],
            last: true,
        };
        let bytes = round(&long, false);
        assert_eq!((bytes.len(), bytes[0]), (201, 0x01), "no unused bytes");
        let empty = Sdo::DownloadSegment {
            toggle: false,
            data: Vec::new(),
            last: true,
        };
        assert_eq!(round(&empty, true), [0x0f, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn an_upload_is_the_mirror_and_an_abort_carries_its_reason() {
        let ask = Sdo::InitiateUpload {
            index: 0x1000,
            subindex: 0,
        };
        assert_eq!(round(&ask, true), [0x40, 0x00, 0x10, 0, 0, 0, 0, 0]);
        round(
            &Sdo::UploadOpened {
                index: 0x2000,
                subindex: 0,
                opening: Opening::Sized {
                    size: 9,
                    data: Vec::new(),
                },
            },
            false,
        );
        round(&Sdo::UploadSegment { toggle: false }, true);
        round(
            &Sdo::UploadData {
                toggle: false,
                data: vec![9; 7],
                last: false,
            },
            false,
        );
        let abort = Sdo::Abort {
            index: 0x9999,
            subindex: 1,
            code: ABORT_NO_OBJECT,
        };
        assert_eq!(
            abort.encode().expect("encode"),
            [0x80, 0x99, 0x99, 1, 0x00, 0x00, 0x02, 0x06]
        );
        assert_eq!(
            Sdo::decode(&abort.encode().expect("encode"), true).expect("abort"),
            abort
        );
        assert!(!abort.is_request());
    }

    #[test]
    fn what_is_not_an_sdo_is_refused() {
        assert!(Sdo::decode(&[0x40, 0, 0], true).is_err(), "short");
        assert!(
            Sdo::decode(&[0xa0, 0, 0, 0, 0, 0, 0, 0], true).is_err(),
            "specifier 5"
        );
        assert!(
            Sdo::decode(&[0xe0, 0, 0, 0, 0, 0, 0, 0], false).is_err(),
            "specifier 7"
        );
        assert!(
            Sdo::decode(&[0x21, 0, 0x20], true).is_err(),
            "short initiate"
        );
    }
}
