# xmip-core-transport-canopen

CANopen transport: CiA 301 over CAN — NMT, expedited and segmented SDO to an object dictionary index and subindex, PDO mapping at its minimum; a Stream travels as a domain object. The SDO here — codec, client and server — is the one the ethercat technology carries as CoE. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls: scheme, authority, path and decoded query. Until 2026-09-28 it was read through the transport capability's `socket::target`, which split it on its first slash and left the query in the path.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Acknowledgement

A receive is an SDO upload of the object, which consumes nothing at the node.
Its verdict therefore has nothing to tell the node, whichever it is: a receive
cycle that did not complete loses nothing, and the next upload reads the
object again. The object's bytes arrive whole.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
