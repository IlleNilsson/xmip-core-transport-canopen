//! The client's side of an SDO transfer, over any carrier: a download
//! opens with an initiate and follows with segments under a toggle, an
//! upload asks and then asks for segments until the last. What each SDO
//! carries is the carrier's [`Window`]; how an SDO reaches the server and
//! its answer comes back is the caller's `call`.

use transport::TransportError;
use transport::error::{Result, protocol_error};

use super::{Opening, Sdo, Window};

/// Write `bytes` to `index:subindex`: expedited where the window takes
/// them whole, otherwise sized, with what the initiate carries and then
/// segments. Where the initiate carries nothing — CAN — at least one
/// segment follows, the last.
///
/// # Errors
/// More than a 32-bit size, a server that aborts or answers out of turn,
/// or what `call` returns.
pub fn download(
    window: &Window,
    index: u16,
    subindex: u8,
    bytes: &[u8],
    mut call: impl FnMut(&Sdo) -> Result<Sdo>,
) -> Result<()> {
    let size = u32::try_from(bytes.len()).map_err(|_| protocol_error("over what an SDO sizes"))?;
    let (opening, rest) = if !bytes.is_empty() && bytes.len() <= window.expedited {
        (Opening::Expedited(bytes.to_vec()), None)
    } else {
        let first = bytes.len().min(window.initiate);
        let data = bytes[..first].to_vec();
        (Opening::Sized { size, data }, Some(&bytes[first..]))
    };
    let open = Sdo::InitiateDownload {
        index,
        subindex,
        opening,
    };
    match call(&open)? {
        Sdo::DownloadAccepted { .. } => {}
        other => return Err(unexpected(&other)),
    }
    let Some(rest) = rest else {
        return Ok(());
    };
    let segments: Vec<&[u8]> = if !rest.is_empty() {
        rest.chunks(window.segment.max(1)).collect()
    } else if window.initiate == 0 {
        vec![&[]]
    } else {
        Vec::new()
    };
    let total = segments.len();
    let mut toggle = false;
    for (n, chunk) in segments.into_iter().enumerate() {
        let segment = Sdo::DownloadSegment {
            toggle,
            data: chunk.to_vec(),
            last: n + 1 == total,
        };
        match call(&segment)? {
            Sdo::SegmentAccepted { toggle: took } if took == toggle => {}
            other => return Err(unexpected(&other)),
        }
        toggle = !toggle;
    }
    Ok(())
}

/// Read `index:subindex`: the expedited bytes, or what the initiate
/// carried and every segment after it.
///
/// # Errors
/// A server that aborts, answers out of turn, or sends more or fewer bytes
/// than the size it indicated; or what `call` returns.
pub fn upload(
    window: &Window,
    index: u16,
    subindex: u8,
    mut call: impl FnMut(&Sdo) -> Result<Sdo>,
) -> Result<Vec<u8>> {
    let (size, mut bytes) = match call(&Sdo::InitiateUpload { index, subindex })? {
        Sdo::UploadOpened {
            opening: Opening::Expedited(data),
            ..
        } => return Ok(data),
        Sdo::UploadOpened {
            opening: Opening::Sized { size, data },
            ..
        } => (usize::try_from(size).unwrap_or(usize::MAX), data),
        other => return Err(unexpected(&other)),
    };
    let mut done = window.initiate > 0 && bytes.len() >= size;
    let mut toggle = false;
    while !done {
        match call(&Sdo::UploadSegment { toggle })? {
            Sdo::UploadData {
                toggle: got,
                data,
                last,
            } if got == toggle => {
                bytes.extend_from_slice(&data);
                done = last;
            }
            other => return Err(unexpected(&other)),
        }
        toggle = !toggle;
    }
    if size != 0 && bytes.len() != size {
        return Err(protocol_error(format!(
            "an upload of {} bytes that said it was {size}",
            bytes.len()
        )));
    }
    Ok(bytes)
}

fn unexpected(answer: &Sdo) -> TransportError {
    match answer {
        Sdo::Abort { code, .. } => protocol_error(format!("the server aborted with {code:#010x}")),
        other => protocol_error(format!("the server answered out of turn: {other:?}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdo::server::Server;
    use crate::sdo::{ABORT_NO_OBJECT, CAN};

    const MAILBOX: Window = Window {
        expedited: 0,
        initiate: 20,
        segment: 30,
    };

    /// A client and a server over `window`, every SDO through its bytes.
    fn wire(server: &mut Server) -> impl FnMut(&Sdo) -> Result<Sdo> + '_ {
        |request| {
            let asked = Sdo::decode(&request.encode()?, true)?;
            Sdo::decode(&server.serve(asked).encode()?, false)
        }
    }

    #[test]
    fn every_length_round_trips_over_either_window() {
        for window in [CAN, MAILBOX] {
            let mut server = Server::new(window).with_object(0x2000, 0, Vec::new());
            for length in [0, 1, 4, 5, 7, 8, 20, 21, 50, 51, 300] {
                let bytes: Vec<u8> = (0..length)
                    .map(|i| u8::try_from(i % 251).unwrap_or(0))
                    .collect();
                download(&window, 0x2000, 0, &bytes, wire(&mut server)).expect("download");
                assert_eq!(server.object(0x2000, 0), Some(bytes.as_slice()), "{length}");
                let back = upload(&window, 0x2000, 0, wire(&mut server)).expect("upload");
                assert_eq!(back, bytes, "{window:?} {length}");
            }
        }
    }

    #[test]
    fn an_abort_is_the_servers_reason_and_a_wrong_size_is_refused() {
        let mut server = Server::new(CAN);
        let error = download(&CAN, 0x9999, 0, b"x", wire(&mut server)).expect_err("no object");
        assert!(
            error.message.contains(&format!("{ABORT_NO_OBJECT:#010x}")),
            "{error}"
        );
        let lying = |request: &Sdo| {
            Ok(match request {
                Sdo::InitiateUpload { .. } => Sdo::UploadOpened {
                    index: 1,
                    subindex: 0,
                    opening: Opening::Sized {
                        size: 3,
                        data: Vec::new(),
                    },
                },
                _ => Sdo::UploadData {
                    toggle: false,
                    data: vec![1],
                    last: true,
                },
            })
        };
        assert!(upload(&CAN, 1, 0, lying).is_err(), "one byte of three");
    }
}
