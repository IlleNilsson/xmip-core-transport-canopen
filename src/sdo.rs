//! The service data object protocol of `CiA 301` section 7.2.4: the eight
//! bytes a client and a server exchange to move one object dictionary
//! entry, expedited when it fits in four bytes and segmented when it does
//! not.
//!
//! The first byte is the command specifier: the top three bits say which
//! of the eight exchanges this is, and the rest say how many of the data
//! bytes are meaningful, whether the transfer is expedited, whether a size
//! follows, which toggle this segment is under, and whether it is the last.
//! Every request is answered; an abort in either direction ends the transfer
//! with a reason.

use transport::error::{Result, protocol_error};

/// The COB-ID a client sends requests under, plus the node.
pub const CLIENT_BASE: u32 = 0x600;
/// The COB-ID a server answers from, plus the node.
pub const SERVER_BASE: u32 = 0x580;

/// The payload one segment carries beside its command specifier.
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

/// One SDO exchange, told apart by its command specifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Sdo {
    /// A client opens a download: expedited with the data, or segmented
    /// with the size to come.
    InitiateDownload {
        index: u16,
        subindex: u8,
        expedited: Option<Vec<u8>>,
        size: u32,
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
    /// The server answers: the data, expedited, or the size to come.
    UploadOpened {
        index: u16,
        subindex: u8,
        expedited: Option<Vec<u8>>,
        size: u32,
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
    /// The eight bytes.
    #[must_use]
    pub fn encode(&self) -> [u8; 8] {
        let mut out = [0u8; 8];
        match self {
            Self::InitiateDownload {
                index,
                subindex,
                expedited,
                size,
            } => {
                out[0] = 0x20 | initiate_bits(expedited.as_deref(), &mut out);
                multiplexer(&mut out, *index, *subindex);
                if expedited.is_none() {
                    out[4..].copy_from_slice(&size.to_le_bytes());
                }
            }
            Self::DownloadAccepted { index, subindex } => {
                out[0] = 0x60;
                multiplexer(&mut out, *index, *subindex);
            }
            Self::DownloadSegment { toggle, data, last }
            | Self::UploadData { toggle, data, last } => {
                out[0] = segment_bits(*toggle, data, *last, &mut out);
            }
            Self::SegmentAccepted { toggle } => out[0] = 0x20 | toggle_bit(*toggle),
            Self::InitiateUpload { index, subindex } => {
                out[0] = 0x40;
                multiplexer(&mut out, *index, *subindex);
            }
            Self::UploadOpened {
                index,
                subindex,
                expedited,
                size,
            } => {
                out[0] = 0x40 | initiate_bits(expedited.as_deref(), &mut out);
                multiplexer(&mut out, *index, *subindex);
                if expedited.is_none() {
                    out[4..].copy_from_slice(&size.to_le_bytes());
                }
            }
            Self::UploadSegment { toggle } => out[0] = 0x60 | toggle_bit(*toggle),
            Self::Abort {
                index,
                subindex,
                code,
            } => {
                out[0] = 0x80;
                multiplexer(&mut out, *index, *subindex);
                out[4..].copy_from_slice(&code.to_le_bytes());
            }
        }
        out
    }

    /// The exchange `bytes` carry, read as a request when `request` and as
    /// an answer otherwise — the same specifier bits mean different things
    /// in the two directions.
    ///
    /// # Errors
    /// Fewer than eight bytes, a specifier neither direction uses, or an
    /// expedited transfer claiming more than four bytes.
    pub fn decode(bytes: &[u8], request: bool) -> Result<Self> {
        if bytes.len() < 8 {
            return Err(protocol_error("an SDO shorter than eight bytes"));
        }
        let specifier = bytes[0] >> 5;
        let index = u16::from_le_bytes([bytes[1], bytes[2]]);
        let subindex = bytes[3];
        let toggle = bytes[0] & 0x10 != 0;
        match (specifier, request) {
            (0, true) => {
                let (data, last) = segment_data(bytes);
                Ok(Self::DownloadSegment { toggle, data, last })
            }
            (0, false) => {
                let (data, last) = segment_data(bytes);
                Ok(Self::UploadData { toggle, data, last })
            }
            (1, true) => {
                let (expedited, size) = initiate_data(bytes);
                Ok(Self::InitiateDownload {
                    index,
                    subindex,
                    expedited,
                    size,
                })
            }
            (1, false) => Ok(Self::SegmentAccepted { toggle }),
            (2, true) => Ok(Self::InitiateUpload { index, subindex }),
            (2, false) => {
                let (expedited, size) = initiate_data(bytes);
                Ok(Self::UploadOpened {
                    index,
                    subindex,
                    expedited,
                    size,
                })
            }
            (3, true) => Ok(Self::UploadSegment { toggle }),
            (3, false) => Ok(Self::DownloadAccepted { index, subindex }),
            (4, _) => Ok(Self::Abort {
                index,
                subindex,
                code: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            }),
            (other, _) => Err(protocol_error(format!(
                "an SDO command specifier of {other}"
            ))),
        }
    }
}

fn multiplexer(out: &mut [u8; 8], index: u16, subindex: u8) {
    out[1..3].copy_from_slice(&index.to_le_bytes());
    out[3] = subindex;
}

const fn toggle_bit(toggle: bool) -> u8 {
    if toggle { 0x10 } else { 0x00 }
}

/// The n, e and s bits of an initiate, writing expedited data in place.
fn initiate_bits(expedited: Option<&[u8]>, out: &mut [u8; 8]) -> u8 {
    match expedited {
        Some(data) => {
            let data = &data[..data.len().min(EXPEDITED_DATA)];
            out[4..4 + data.len()].copy_from_slice(data);
            let unused = u8::try_from(EXPEDITED_DATA - data.len()).unwrap_or(0);
            0x03 | (unused << 2)
        }
        None => 0x01,
    }
}

/// The t, n and c bits of a segment, writing its data in place.
fn segment_bits(toggle: bool, data: &[u8], last: bool, out: &mut [u8; 8]) -> u8 {
    let data = &data[..data.len().min(SEGMENT_DATA)];
    out[1..=data.len()].copy_from_slice(data);
    let unused = u8::try_from(SEGMENT_DATA - data.len()).unwrap_or(0);
    toggle_bit(toggle) | (unused << 1) | u8::from(last)
}

fn initiate_data(bytes: &[u8]) -> (Option<Vec<u8>>, u32) {
    let expedited = bytes[0] & 0x02 != 0;
    let sized = bytes[0] & 0x01 != 0;
    if expedited {
        let unused = usize::from((bytes[0] >> 2) & 0x03);
        let length = if sized {
            EXPEDITED_DATA - unused
        } else {
            EXPEDITED_DATA
        };
        let data = bytes[4..4 + length].to_vec();
        let size = u32::try_from(length).unwrap_or(0);
        return (Some(data), size);
    }
    let size = if sized {
        u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]])
    } else {
        0
    };
    (None, size)
}

fn segment_data(bytes: &[u8]) -> (Vec<u8>, bool) {
    let unused = usize::from((bytes[0] >> 1) & 0x07);
    let last = bytes[0] & 0x01 != 0;
    (bytes[1..=SEGMENT_DATA - unused].to_vec(), last)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_expedited_download_carries_its_bytes_in_the_initiate() {
        let sdo = Sdo::InitiateDownload {
            index: 0x1017,
            subindex: 0,
            expedited: Some(vec![0xe8, 0x03]),
            size: 2,
        };
        let bytes = sdo.encode();
        assert_eq!(bytes, [0x2b, 0x17, 0x10, 0x00, 0xe8, 0x03, 0, 0]);
        assert_eq!(Sdo::decode(&bytes, true).expect("request"), sdo);
        let accepted = Sdo::DownloadAccepted {
            index: 0x1017,
            subindex: 0,
        };
        assert_eq!(accepted.encode(), [0x60, 0x17, 0x10, 0, 0, 0, 0, 0]);
        assert_eq!(
            Sdo::decode(&accepted.encode(), false).expect("answer"),
            accepted
        );
    }

    #[test]
    fn a_segmented_download_says_its_size_then_toggles_its_segments() {
        let open = Sdo::InitiateDownload {
            index: 0x2000,
            subindex: 0,
            expedited: None,
            size: 300,
        };
        assert_eq!(open.encode(), [0x21, 0x00, 0x20, 0x00, 0x2c, 0x01, 0, 0]);
        assert_eq!(Sdo::decode(&open.encode(), true).expect("open"), open);
        let segment = Sdo::DownloadSegment {
            toggle: true,
            data: vec![1, 2, 3],
            last: true,
        };
        assert_eq!(segment.encode(), [0x19, 1, 2, 3, 0, 0, 0, 0]);
        assert_eq!(
            Sdo::decode(&segment.encode(), true).expect("segment"),
            segment
        );
        let took = Sdo::SegmentAccepted { toggle: true };
        assert_eq!(took.encode()[0], 0x30);
        assert_eq!(Sdo::decode(&took.encode(), false).expect("took"), took);
    }

    #[test]
    fn an_upload_is_the_mirror_and_an_abort_carries_its_reason() {
        let ask = Sdo::InitiateUpload {
            index: 0x1000,
            subindex: 0,
        };
        assert_eq!(ask.encode(), [0x40, 0x00, 0x10, 0, 0, 0, 0, 0]);
        assert_eq!(Sdo::decode(&ask.encode(), true).expect("ask"), ask);
        let opened = Sdo::UploadOpened {
            index: 0x2000,
            subindex: 0,
            expedited: None,
            size: 9,
        };
        assert_eq!(
            Sdo::decode(&opened.encode(), false).expect("opened"),
            opened
        );
        let next = Sdo::UploadSegment { toggle: false };
        assert_eq!(Sdo::decode(&next.encode(), true).expect("next"), next);
        let data = Sdo::UploadData {
            toggle: false,
            data: vec![9; 7],
            last: false,
        };
        assert_eq!(Sdo::decode(&data.encode(), false).expect("data"), data);
        let abort = Sdo::Abort {
            index: 0x9999,
            subindex: 1,
            code: ABORT_NO_OBJECT,
        };
        assert_eq!(
            abort.encode(),
            [0x80, 0x99, 0x99, 1, 0x00, 0x00, 0x02, 0x06]
        );
        assert_eq!(Sdo::decode(&abort.encode(), true).expect("abort"), abort);
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
    }
}
