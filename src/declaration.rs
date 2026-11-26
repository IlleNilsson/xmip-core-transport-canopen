//! What a Location of this technology is configured with: its declaration,
//! which `Configured` reads, built with the crate's `declaration` feature
//! alone and the code behind the default (ADR-0064, amendments 2026-10-22
//! and 2026-11-26), for the library the surfaces load to carry (open
//! problem 48). The object and the wait a Location takes unless told
//! otherwise are declared here, and the code takes them from here.

use std::time::Duration;

use xcore::settings::{Applies, Fixed, Kind, Presence, Setting, Settings};

/// The object a Stream travels as unless a target says otherwise: the
/// first manufacturer-specific index, a domain.
pub const STREAM_OBJECT: (u16, u8) = (0x2000, 0);

/// How long a node that stops answering is waited on when a Location says
/// nothing else.
pub const TIMEOUT: Duration = Duration::from_secs(1);

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
        Setting {
            name: "index",
            kind: Kind::Integer {
                minimum: 0,
                maximum: 0xffff,
            },
            presence: Presence::Default(Fixed::Integer(STREAM_OBJECT.0 as i64)),
            meaning: "The object's index in the dictionary.",
            applies: Applies::Both,
        },
        Setting {
            name: "subindex",
            kind: Kind::Integer {
                minimum: 0,
                maximum: 255,
            },
            presence: Presence::Default(Fixed::Integer(STREAM_OBJECT.1 as i64)),
            meaning: "The object's subindex under its index.",
            applies: Applies::Both,
        },
        transport::configured::timeout(
            Applies::Both,
            Some(TIMEOUT),
            "How long a node that stops answering is waited on.",
        ),
    ],
};

/// The schemes a Location's address is entered with to choose this technology.
pub const SCHEMES: &[&str] = &["canopen"];
