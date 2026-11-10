# Wire protocol L2K1

The contract between a **sender** (the Lidar2Kinect app on an iPhone or iPad) and a **receiver**
(the Windows fake Kinect or spice2x hook, the debug tool `l2k_dump`, or the iPad game port when
it takes its sensor from another device).

The reference implementation is `shared/src/l2k_protocol.c` and `shared/src/l2k_depth.c`. Golden
packets are in `shared/tests/vectors`; `shared/tests/test_l2k.c` builds and checks them. If this
document and the code disagree, that is a bug: fix both in one change.

## 1. Transport

- UDP, IPv4. The receiver listens on port **24810** by default.
- One packet per datagram, at most **1248 bytes**, so nothing is fragmented by IP on a normal
  network.
- The receiver replies to the address and port a packet came from, from its listening socket.
- All integers and floats are **little-endian**; floats are IEEE-754 single precision.
- Nothing is retransmitted. A lost skeleton packet is replaced by the next one 33 ms later; an
  image with a lost chunk is dropped and the receiver keeps showing the previous image.
- The same packets can be carried over a byte stream (TCP, USB): the header holds the packet
  length, so packets are simply written back to back. Not used in version 1.

A receiver may advertise itself with Bonjour / DNS-SD as service type `_l2k._udp` so the app can
list it. This is optional; the address can always be typed into the app.

## 2. Packet header (16 bytes)

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u32 | magic | `0x314B324C`, the bytes `L` `2` `K` `1` |
| 4 | u8 | version | 1 |
| 5 | u8 | type | see below |
| 6 | u16 | length | whole packet including this header |
| 8 | u32 | session | random number chosen when the program starts; a different value means the peer restarted |
| 12 | u32 | seq | +1 for every packet this program sends, of any type (loss counting) |

Types: `0x01` HELLO, `0x02` CONFIG, `0x03` BYE, `0x10` SKELETON, `0x11` DEPTH, `0x12` COLOR.
A receiver ignores packets with a wrong magic, another version, or a type it does not know.

## 3. Session

1. The sender sends **HELLO** to the receiver once a second, for as long as it runs.
2. The receiver answers every HELLO with a **CONFIG** that says which streams it wants.
3. While the sender has a CONFIG younger than 3 s, it streams what that CONFIG asks for, limited
   to what it announced in HELLO. With no fresh CONFIG it streams nothing and keeps sending HELLO.
4. The receiver treats the sender as gone after 3 s without any packet.
5. **BYE** (header only) tells the other side to stop at once. It is a courtesy; the timeouts
   cover a lost BYE.

A receiver changes what it wants (for example when the game opens or closes a stream) simply by
putting different values in its next CONFIG. It may also send a CONFIG without waiting for a
HELLO once it knows the sender's address.

### HELLO (type 0x01, 64 bytes)

| Offset | Type | Field |
|---|---|---|
| 16 | u64 | sender_time_us — the sender's clock now (microseconds, monotonic) |
| 24 | u32 | caps — stream bits the sender can produce |
| 28 | u8 | max_tracked — bodies it can fully track at once |
| 29 | u8 | source_kind — 0 synthetic, 1 ARKit body tracking, 2 Vision 3D, 3 Vision 2D + depth, 4 replay |
| 30 | u16 | reserved, 0 |
| 32 | char[32] | name — UTF-8, zero padded |

### CONFIG (type 0x02, 76 bytes)

| Offset | Type | Field |
|---|---|---|
| 16 | u64 | echo_sender_time_us — `sender_time_us` of the HELLO being answered (the sender measures the round trip) |
| 24 | u64 | receiver_time_us — the receiver's clock now |
| 32 | u32 | streams — stream bits wanted |
| 36 | u32 | skeleton_flags — the game's `NUI_SKELETON_TRACKING_FLAG_*` bits (informational in version 1) |
| 40 | u8 | consumer_kind — 0 unknown, 1 monitor, 2 spice2x hook, 3 fake Kinect10.dll, 4 game port |
| 41 | u8 | depth_mode — 0 depth on player pixels only, 1 full depth |
| 42 | u8 | color_quality — JPEG quality 1…100, 0 = sender's default |
| 43 | u8 | reserved, 0 |
| 44 | char[32] | name |

Stream bits: `0x1` skeleton, `0x2` depth + player index (320×240), `0x4` colour (640×480).

## 4. SKELETON (type 0x10)

One packet is one whole skeleton frame.

| Offset | Type | Field |
|---|---|---|
| 16 | u64 | timestamp_us — capture time on the sender's clock |
| 24 | u32 | frame_number — +1 per frame the sender produces |
| 28 | u32 | flags — bit 0: floor plane valid |
| 32 | f32[4] | floor plane A, B, C, D (all zero when not valid) |
| 48 | f32[3] | up — unit vector opposite to gravity, in skeleton space |
| 60 | u8 | body_count — number of body records that follow (0…6) |
| 61 | u8 | source_kind |
| 62 | u16 | reserved, 0 |
| 64 | … | body records |

Body record:

| Type | Field |
|---|---|
| u8 | slot — 0…5, the index in the Kinect's skeleton array; each slot at most once |
| u8 | state — 1 position only, 2 tracked |
| u16 | quality — clipped-edge bits: 1 right, 2 left, 4 top, 8 bottom |
| u32 | tracking_id — nonzero, stable while the body stays tracked |
| f32[3] | position |
| f32[20][3] | joint positions — **only when state = 2** |
| u8[20] | joint states (0 not tracked, 1 inferred, 2 tracked) — **only when state = 2** |

A position-only record is 20 bytes, a tracked one 280. Slots not listed are empty. Joint order is
the Kinect v1 order (hip centre, spine, shoulder centre, head, left arm ×4, right arm ×4, left
leg ×4, right leg ×4). Units and axes: `docs/03-virtual-kinect.md`.

A frame with more tracked bodies than fit in 1248 bytes (five or more) cannot be sent; the
sender tracks at most two, like a Kinect.

The receiver keeps the frame with the highest `frame_number` and drops older ones that arrive late.

## 5. DEPTH and COLOR (types 0x11, 0x12)

An image is encoded, cut into chunks of 1200 bytes (the last one may be shorter), and each chunk
is sent as one packet.

| Offset | Type | Field |
|---|---|---|
| 16 | u64 | timestamp_us |
| 24 | u32 | frame_number — the number of the skeleton frame captured at the same instant |
| 28 | u16 | width |
| 30 | u16 | height |
| 32 | u8 | encoding |
| 33 | u8 | flags, 0 |
| 34 | u16 | chunk_index — 0 … chunk_count−1 |
| 36 | u16 | chunk_count — at most 1024 |
| 38 | u16 | chunk_len — payload bytes in this packet |
| 40 | u32 | total_len — size of the encoded image |
| 44 | u32 | offset — `chunk_index × 1200` |
| 48 | … | payload |

The receiver collects the chunks of the newest `frame_number` it has seen; a chunk of a newer
frame discards an unfinished older one (`l2k_assembler`).

**Depth encodings.** The decoded image is `width × height` 16-bit pixels, row by row from the top
left, each `depth_mm << 3 | player_index` (the Kinect's own packing; player index 0 = no player,
n = skeleton slot n−1; depth 0 = unknown or out of the 800…4000 mm range).

- `0` raw: the pixels as u16.
- `1` **D1**: a byte stream, with `prev` starting at 0:
  - `0x00…0x7F` — repeat `prev` (byte + 1) times;
  - `0x80…0xFE` — `d = byte − 0xBF` (−63…+63, never 0): add `d` mm to `prev`'s depth, keep its
    player index, output one pixel;
  - `0xFF lo hi` — `prev = lo | hi << 8`, output one pixel.

  The stream must produce exactly `width × height` pixels. A silhouette image is about 10 kB.

**Colour encodings.** `1` = baseline JPEG, 640×480. (`0` = raw BGRX, for tests only.) The colour
image is registered to the depth image: colour pixel (2x, 2y) shows the same point as depth
pixel (x, y).

## 6. Rates and sizes

Measured with the synthetic two-player figure on a local connection: skeleton 30 packets/s of
624 bytes; depth 30 images/s of about 10 kB (9 packets). Colour depends on the picture: the test
card is 8–11 kB; camera images at quality 60 have not been measured yet and are expected to be
30–60 kB (25–50 packets), which would put the total at 1.2–2.1 MB/s.

## 7. Changing the protocol

- Adding a stream bit, an encoding value, a `source_kind` or a `consumer_kind` is compatible:
  old peers ignore what they do not ask for.
- Any change to an existing layout needs a new `version`, new golden packets, and an entry in
  `docs/04-status.md` so both sides move together.
