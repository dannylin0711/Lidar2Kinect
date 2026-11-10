/*
 * Lidar2Kinect shared library - wire protocol "L2K1".
 *
 * Normative description: docs/02-wire-protocol.md. This header and l2k_protocol.c are the
 * reference implementation; shared/tests/vectors holds golden packets both sides must match.
 *
 * Every packet is one UDP datagram (or, over a byte stream, back-to-back packets: the header
 * carries the packet length). All integers and floats are little-endian. The functions here
 * only read and write byte buffers - sockets belong to the caller.
 */
#ifndef L2K_PROTOCOL_H
#define L2K_PROTOCOL_H

#include "l2k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define L2K_MAGIC 0x314B324Cu /* bytes 'L','2','K','1' */
#define L2K_VERSION 1
#define L2K_DEFAULT_PORT 24810 /* UDP port the receiver (PC / game) listens on */
#define L2K_BONJOUR_TYPE "_l2k._udp" /* the receiver may advertise this service */

#define L2K_HEADER_SIZE 16
#define L2K_CHUNK_HEADER_SIZE 32   /* after the packet header */
#define L2K_CHUNK_PAYLOAD 1200     /* max image bytes per datagram */
#define L2K_MAX_DATAGRAM (L2K_HEADER_SIZE + L2K_CHUNK_HEADER_SIZE + L2K_CHUNK_PAYLOAD) /* 1248 */
#define L2K_MAX_CHUNKS 1024        /* => at most 1 228 800 encoded bytes per image */
#define L2K_NAME_SIZE 32           /* UTF-8, NUL padded on the wire */

/* Timing (see docs/02): the sender repeats HELLO every L2K_HELLO_INTERVAL_MS, the receiver answers
 * each HELLO with CONFIG; either side treats the other as gone after L2K_PEER_TIMEOUT_MS of silence. */
#define L2K_HELLO_INTERVAL_MS 1000
#define L2K_PEER_TIMEOUT_MS 3000

/* Packet types */
enum {
    L2K_PKT_HELLO = 0x01,    /* sender -> receiver */
    L2K_PKT_CONFIG = 0x02,   /* receiver -> sender */
    L2K_PKT_BYE = 0x03,      /* either direction */
    L2K_PKT_SKELETON = 0x10, /* sender -> receiver, one whole skeleton frame */
    L2K_PKT_DEPTH = 0x11,    /* sender -> receiver, one chunk of a depth image */
    L2K_PKT_COLOR = 0x12     /* sender -> receiver, one chunk of a colour image */
};

/* Stream bits (l2k_hello.caps = what the sender can produce, l2k_config.streams = what the receiver wants) */
enum {
    L2K_STREAM_SKELETON = 0x1,
    L2K_STREAM_DEPTH = 0x2, /* 320x240 depth + player index */
    L2K_STREAM_COLOR = 0x4  /* 640x480 colour, registered to the depth image */
};

/* Image encodings (l2k_chunk.encoding) */
enum {
    L2K_DEPTH_RAW = 0, /* width*height uint16 LE */
    L2K_DEPTH_D1 = 1   /* run/delta code, see l2k_depth.h */
};
enum {
    L2K_COLOR_BGRX = 0, /* width*height*4 bytes, B G R X (test use only: far too large for Wi-Fi) */
    L2K_COLOR_JPEG = 1  /* baseline JPEG */
};

/* l2k_config.depth_mode */
enum {
    L2K_DEPTH_MODE_PLAYERS = 0, /* depth only on player pixels, 0 elsewhere (small) */
    L2K_DEPTH_MODE_FULL = 1     /* depth for the whole image */
};

/* l2k_config.consumer_kind */
enum {
    L2K_CONSUMER_UNKNOWN = 0,
    L2K_CONSUMER_MONITOR = 1,     /* debug tool */
    L2K_CONSUMER_SPICE_HOOK = 2,  /* windows: hook DLL loaded by spice2x */
    L2K_CONSUMER_FAKE_KINECT = 3, /* windows: replacement Kinect10.dll */
    L2K_CONSUMER_GAME_PORT = 4    /* iPad game port receiving from another device */
};

/* l2k_config.skeleton_flags: same bits as NUI_SKELETON_TRACKING_FLAG_* (informational in v1) */

typedef struct l2k_header {
    uint8_t version;
    uint8_t type;
    uint16_t length;  /* whole packet, header included */
    uint32_t session; /* random per run of the sending program; a change means "peer restarted" */
    uint32_t seq;     /* +1 for every packet a program sends, any type */
} l2k_header;

typedef struct l2k_hello {
    uint64_t sender_time_us; /* sender monotonic clock now */
    uint32_t caps;           /* L2K_STREAM_* the sender can produce */
    uint8_t max_tracked;     /* bodies it can fully track at once */
    uint8_t source_kind;     /* L2K_SOURCE_* */
    char name[L2K_NAME_SIZE + 1]; /* device name, always NUL terminated after decode */
} l2k_hello;

typedef struct l2k_config {
    uint64_t echo_sender_time_us; /* sender_time_us of the HELLO being answered (round-trip time) */
    uint64_t receiver_time_us;    /* receiver monotonic clock now */
    uint32_t streams;             /* L2K_STREAM_* wanted */
    uint32_t skeleton_flags;
    uint8_t consumer_kind;        /* L2K_CONSUMER_* */
    uint8_t depth_mode;           /* L2K_DEPTH_MODE_* */
    uint8_t color_quality;        /* JPEG quality 1..100, 0 = sender default */
    uint8_t reserved;
    char name[L2K_NAME_SIZE + 1];
} l2k_config;

/* One chunk of an encoded image (L2K_PKT_DEPTH / L2K_PKT_COLOR). */
typedef struct l2k_chunk {
    uint64_t timestamp_us;
    uint32_t frame_number;
    uint16_t width, height;
    uint8_t encoding;
    uint8_t flags;
    uint16_t chunk_index; /* 0 .. chunk_count-1 */
    uint16_t chunk_count;
    uint16_t chunk_len;   /* payload bytes in this packet, <= L2K_CHUNK_PAYLOAD */
    uint32_t total_len;   /* encoded image size */
    uint32_t offset;      /* of this payload in the encoded image = chunk_index * L2K_CHUNK_PAYLOAD */
    const uint8_t *payload; /* decode: points into the packet buffer */
} l2k_chunk;

/* Error codes (negative) */
enum {
    L2K_OK = 0,
    L2K_ERR_SHORT = -1,   /* buffer too small / packet truncated */
    L2K_ERR_MAGIC = -2,
    L2K_ERR_VERSION = -3,
    L2K_ERR_TYPE = -4,
    L2K_ERR_FORMAT = -5,  /* field out of range */
    L2K_ERR_STALE = -6    /* older than what is already held */
};

/* --- encoding: each returns the packet length, or 0 if `cap` is too small / input invalid --- */
size_t l2k_encode_hello(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_hello *hello);
size_t l2k_encode_config(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_config *config);
size_t l2k_encode_bye(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq);
size_t l2k_encode_skeleton(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_skeleton_frame *frame);
/* `type` is L2K_PKT_DEPTH or L2K_PKT_COLOR; chunk->payload/chunk_len are copied into the packet. */
size_t l2k_encode_chunk(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, uint8_t type, const l2k_chunk *chunk);

/* Number of chunks an encoded image of `total_len` bytes needs (0 if it is too large). */
uint32_t l2k_chunk_count(size_t total_len);
/* Fill `chunk` for chunk number `index` of `data[0..total_len)`; the caller sets timestamp_us,
 * frame_number, width, height, encoding, flags beforehand. Returns 0 if index is out of range. */
int l2k_chunk_slice(l2k_chunk *chunk, const uint8_t *data, size_t total_len, uint32_t index);

/* --- decoding: each returns L2K_OK or a negative error --- */
int l2k_decode_header(const uint8_t *buf, size_t len, l2k_header *out);
int l2k_decode_hello(const uint8_t *buf, size_t len, l2k_hello *out);
int l2k_decode_config(const uint8_t *buf, size_t len, l2k_config *out);
int l2k_decode_skeleton(const uint8_t *buf, size_t len, l2k_skeleton_frame *out);
int l2k_decode_chunk(const uint8_t *buf, size_t len, l2k_chunk *out);

/* --- image reassembly on the receiver ---
 * One assembler per image stream. It holds the newest frame being received; a chunk of a newer
 * frame discards an unfinished older one (latest wins, no retransmission). */
typedef struct l2k_assembler {
    uint8_t *data;   /* caller storage */
    size_t capacity;
    int active;      /* a frame is being collected */
    int complete;    /* `data[0..info.total_len)` holds a whole encoded image described by `info` */
    l2k_chunk info;  /* header fields of the frame held (payload pointer unused) */
    uint32_t received;
    uint32_t frames_completed;
    uint32_t frames_dropped; /* frames replaced before they completed */
    uint8_t have[L2K_MAX_CHUNKS / 8];
} l2k_assembler;

void l2k_assembler_init(l2k_assembler *a, uint8_t *storage, size_t capacity);
/* Returns 1 when this chunk completed a frame, 0 when more are needed, negative on error. */
int l2k_assembler_push(l2k_assembler *a, const l2k_chunk *chunk);

/* Sequence comparison with wrap-around: nonzero if a is newer than b. */
static inline int l2k_seq_newer(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) > 0;
}

#ifdef __cplusplus
}
#endif

#endif /* L2K_PROTOCOL_H */
