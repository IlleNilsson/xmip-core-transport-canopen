//! The server's side of SDO: an object dictionary of byte vectors keyed by
//! index and subindex, and the one download and one upload in progress
//! against it — expedited and segmented, the toggle checked and the aborts
//! `CiA 301` names. A `CANopen` node and an `EtherCAT` slave answer through
//! it, each over its carrier's [`Window`].

use std::collections::HashMap;

use super::{ABORT_COMMAND, ABORT_LENGTH, ABORT_NO_OBJECT, ABORT_TOGGLE, Opening, Sdo, Window};

/// The device type every dictionary holds, `CiA 301` section 7.5.2.1.
const DEVICE_TYPE: (u16, u8) = (0x1000, 0);

/// A segmented transfer in progress.
struct Transfer {
    index: u16,
    subindex: u8,
    toggle: bool,
    bytes: Vec<u8>,
    /// How far an upload has been read.
    at: usize,
}

/// An object dictionary and the transfers against it.
pub struct Server {
    window: Window,
    dictionary: HashMap<(u16, u8), Vec<u8>>,
    download: Option<Transfer>,
    upload: Option<Transfer>,
}

impl Server {
    /// A server over `window` holding device type `0x0000_0000` at
    /// `0x1000:00` and nothing else.
    #[must_use]
    pub fn new(window: Window) -> Self {
        let mut dictionary = HashMap::new();
        dictionary.insert(DEVICE_TYPE, vec![0, 0, 0, 0]);
        Self {
            window,
            dictionary,
            download: None,
            upload: None,
        }
    }

    /// Hold `bytes` at `index:subindex`.
    #[must_use]
    pub fn with_object(mut self, index: u16, subindex: u8, bytes: impl Into<Vec<u8>>) -> Self {
        self.insert(index, subindex, bytes.into());
        self
    }

    /// Hold `bytes` at `index:subindex` from now on.
    pub fn insert(&mut self, index: u16, subindex: u8, bytes: Vec<u8>) {
        self.dictionary.insert((index, subindex), bytes);
    }

    /// The bytes held at `index:subindex`, as they are now.
    #[must_use]
    pub fn object(&self, index: u16, subindex: u8) -> Option<&[u8]> {
        self.dictionary.get(&(index, subindex)).map(Vec::as_slice)
    }

    /// Give up any transfer in progress.
    pub fn abandon(&mut self) {
        self.download = None;
        self.upload = None;
    }

    /// The answer to `request`. What a client may not send now ends any
    /// transfer in progress and is aborted.
    pub fn serve(&mut self, request: Sdo) -> Sdo {
        match request {
            Sdo::InitiateDownload {
                index,
                subindex,
                opening,
            } => self.open_download(index, subindex, opening),
            Sdo::DownloadSegment { toggle, data, last } => self.segment(toggle, &data, last),
            Sdo::InitiateUpload { index, subindex } => self.open_upload(index, subindex),
            Sdo::UploadSegment { toggle } => self.next_segment(toggle),
            _ => {
                self.abandon();
                abort(0, 0, ABORT_COMMAND)
            }
        }
    }

    fn open_download(&mut self, index: u16, sub: u8, opening: Opening) -> Sdo {
        if !self.dictionary.contains_key(&(index, sub)) {
            return abort(index, sub, ABORT_NO_OBJECT);
        }
        match opening {
            Opening::Expedited(data) => self.insert(index, sub, data),
            Opening::Sized { size, data } => {
                let whole = usize::try_from(size).unwrap_or(usize::MAX);
                if data.len() > whole {
                    return abort(index, sub, ABORT_LENGTH);
                }
                if self.window.initiate > 0 && data.len() == whole {
                    self.insert(index, sub, data);
                } else {
                    self.download = Some(Transfer {
                        index,
                        subindex: sub,
                        toggle: false,
                        bytes: data,
                        at: 0,
                    });
                }
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
        if last && let Some(done) = self.download.take() {
            self.insert(done.index, done.subindex, done.bytes);
        }
        Sdo::SegmentAccepted { toggle }
    }

    fn open_upload(&mut self, index: u16, sub: u8) -> Sdo {
        let Some(bytes) = self.dictionary.get(&(index, sub)).cloned() else {
            return abort(index, sub, ABORT_NO_OBJECT);
        };
        if !bytes.is_empty() && bytes.len() <= self.window.expedited {
            return Sdo::UploadOpened {
                index,
                subindex: sub,
                opening: Opening::Expedited(bytes),
            };
        }
        let size = u32::try_from(bytes.len()).unwrap_or(u32::MAX);
        let first = bytes.len().min(self.window.initiate);
        let data = bytes[..first].to_vec();
        if self.window.initiate == 0 || first < bytes.len() {
            self.upload = Some(Transfer {
                index,
                subindex: sub,
                toggle: false,
                bytes,
                at: first,
            });
        }
        Sdo::UploadOpened {
            index,
            subindex: sub,
            opening: Opening::Sized { size, data },
        }
    }

    fn next_segment(&mut self, toggle: bool) -> Sdo {
        let Some(transfer) = self.upload.as_mut() else {
            return abort(0, 0, ABORT_COMMAND);
        };
        if toggle != transfer.toggle {
            let (index, sub) = (transfer.index, transfer.subindex);
            self.upload = None;
            return abort(index, sub, ABORT_TOGGLE);
        }
        transfer.toggle = !toggle;
        let end = (transfer.at + self.window.segment.max(1)).min(transfer.bytes.len());
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdo::CAN;

    fn sized(size: u32, data: Vec<u8>) -> Opening {
        Opening::Sized { size, data }
    }

    #[test]
    fn a_missing_object_a_bad_toggle_and_an_exchange_out_of_turn_are_aborted() {
        let mut server = Server::new(CAN).with_object(0x2000, 0, Vec::new());
        assert_eq!(server.object(0x1000, 0), Some([0u8; 4].as_slice()));
        let open = |index| Sdo::InitiateDownload {
            index,
            subindex: 0,
            opening: sized(3, Vec::new()),
        };
        assert_eq!(
            server.serve(open(0x9999)),
            abort(0x9999, 0, ABORT_NO_OBJECT)
        );
        server.serve(open(0x2000));
        let wrong = Sdo::DownloadSegment {
            toggle: true,
            data: vec![1],
            last: true,
        };
        assert_eq!(server.serve(wrong.clone()), abort(0x2000, 0, ABORT_TOGGLE));
        assert_eq!(
            server.serve(wrong),
            abort(0, 0, ABORT_COMMAND),
            "no transfer"
        );
        assert_eq!(
            server.serve(Sdo::UploadSegment { toggle: false }),
            abort(0, 0, ABORT_COMMAND)
        );
        server.serve(open(0x2000));
        assert_eq!(
            server.serve(Sdo::SegmentAccepted { toggle: false }),
            abort(0, 0, ABORT_COMMAND)
        );
        let after = Sdo::DownloadSegment {
            toggle: false,
            data: vec![1],
            last: true,
        };
        assert_eq!(server.serve(after), abort(0, 0, ABORT_COMMAND), "abandoned");
    }

    #[test]
    fn an_initiate_carrying_more_than_its_size_is_aborted() {
        let window = Window {
            expedited: 0,
            initiate: 10,
            segment: 10,
        };
        let mut server = Server::new(window).with_object(0x2000, 0, Vec::new());
        let over = Sdo::InitiateDownload {
            index: 0x2000,
            subindex: 0,
            opening: sized(2, vec![1, 2, 3]),
        };
        assert_eq!(server.serve(over), abort(0x2000, 0, ABORT_LENGTH));
    }
}
