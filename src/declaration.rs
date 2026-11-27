//! This technology's declaration: the transport guide's "The declaration",
//! `module/core/capability/transport/doc/adding-a-transport.md`. The object
//! and the wait a Location takes unless told otherwise are declared here, and
//! the code takes them from here.

use std::time::Duration;

use xcore::settings::{Applies, Fixed, Kind, Presence, Setting, Settings};

/// The object a Stream travels as unless a target says otherwise: the
/// first manufacturer-specific index, a domain.
pub const STREAM_OBJECT: (u16, u8) = (0x2000, 0);

/// How long a node that stops answering is waited on when a Location says
/// nothing else.
pub const TIMEOUT: Duration = Duration::from_secs(1);

/// The object's index in the dictionary, [`STREAM_OBJECT`]'s unless a
/// Location says: declared once, here, for every technology reading this
/// kind of dictionary, the ethercat technology's mailbox among them.
pub const INDEX: Setting = Setting {
    name: "index",
    kind: Kind::Integer {
        minimum: 0,
        maximum: 0xffff,
    },
    presence: Presence::Default(Fixed::Integer(STREAM_OBJECT.0 as i64)),
    meaning: "The object's index in the dictionary.",
    applies: Applies::Both,
};

/// The object's subindex under its index, [`STREAM_OBJECT`]'s unless a
/// Location says; declared once, as [`INDEX`] is.
pub const SUBINDEX: Setting = Setting {
    name: "subindex",
    kind: Kind::Integer {
        minimum: 0,
        maximum: 255,
    },
    presence: Presence::Default(Fixed::Integer(STREAM_OBJECT.1 as i64)),
    meaning: "The object's subindex under its index.",
    applies: Applies::Both,
};

/// Every setting a Location takes beyond its address, the CAN bus the
/// master is on, as `can_bus::open_bus` opens it: `can0`.
pub const SETTINGS: &Settings = &Settings {
    technology: env!("CARGO_PKG_NAME"),
    settings: &[
        Setting {
            name: "node",
            kind: Kind::Integer {
                minimum: 1,
                maximum: 127,
            },
            presence: Presence::Required,
            meaning: "The node whose object dictionary a Location reads or writes.",
            applies: Applies::Both,
        },
        INDEX,
        SUBINDEX,
        transport::configured::timeout(
            Applies::Both,
            Some(TIMEOUT),
            "How long a node that stops answering is waited on.",
        ),
    ],
};

/// This technology's schemes, as [`transport::Configured::SCHEMES`] defines
/// them.
pub const SCHEMES: &[&str] = &["canopen"];
