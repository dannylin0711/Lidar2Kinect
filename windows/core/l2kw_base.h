/*
 * Lidar2Kinect, Windows side - the part of the core that does not depend on the Kinect ABI:
 * configuration, log, clock, the L2K1 receiver and the hub interface it feeds. Builds for x86
 * and x64.
 *
 * The receiver (l2kw_receiver.cpp) only talks to its consumer through the hub_* functions below.
 * Each consumer links its own hub: the virtual Kinect (l2kw_sensor.cpp, x86, Dance Evolution) or
 * the vp4u wrapper (windows/vp4u, x64, DanceAround).
 */
#ifndef L2KW_BASE_H
#define L2KW_BASE_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <stddef.h>
#include <stdint.h>

extern "C" {
#include "l2k/l2k.h"
}

namespace l2kw {

/* What the game sees while no sender is connected (windows/docs/no-sender.md). */
enum NoSenderMode {
    NO_SENDER_MISSING = 0, /* like an unplugged Kinect: no sensor; the status callback announces it once a sender appears */
    NO_SENDER_IDLE = 1     /* a sensor is always there; it delivers empty frames until a sender appears */
};
/* Where the depth image comes from. */
enum DepthSource {
    DEPTH_AUTO = 0,     /* the sender's depth image while it sends one, else drawn from the skeleton */
    DEPTH_SENDER = 1,   /* only the sender's */
    DEPTH_SKELETON = 2  /* always drawn from the skeleton (l2k_depth_rasterize) */
};

struct Config {
    unsigned port;
    char log_path[MAX_PATH]; /* empty: no log */
    char ini_path[MAX_PATH];
    int no_sender;     /* NoSenderMode */
    int depth_source;  /* DepthSource */
    int depth_mode;    /* L2K_DEPTH_MODE_* asked of the sender */
    int color_quality; /* 0 = sender default */
    int dnssd;         /* advertise _l2k._udp */
    int verbose;       /* log every API call, not just summaries */
    char name[L2K_NAME_SIZE + 1];
    unsigned consumer_kind; /* L2K_CONSUMER_* sent in CONFIG */
};

/* DllMain: remember which module we are (l2k.ini and the log live next to it). Stores only.
 * `product` names the consumer in the log's first line (NULL: derived from consumer_kind). */
void init_module(HMODULE module, unsigned consumer_kind, const char *product = nullptr);
/* Reads l2k.ini next to `module` (env overrides: L2K_INI, L2K_PORT, L2K_LOG). Called by start(). */
void config_load(HMODULE module, unsigned consumer_kind);
const Config &config();
void set_product_name(const char *product);

void log_open();
bool log_is_open();
/* Opens the log at `path` if none is open yet (a consumer's fallback when the configured one fails). */
bool log_open_at(const char *path);
void log(const char *fmt, ...);
double uptime_s();
uint64_t now_us();

/* Starts the receiver thread (idempotent, thread safe). Called from the first API call, never from DllMain. */
void start();

/* JPEG (baseline) -> BGRX via WIC. Returns false if it cannot be decoded or is not w x h. */
bool jpeg_decode_bgrx(const uint8_t *data, size_t len, uint8_t *out, int w, int h);

/* DNS-SD advertisement of _l2k._udp (dnsapi.dll, loaded at run time; absent = silently off). */
void dnssd_start(unsigned port, const char *name);
void dnssd_stop();

/* ---- the hub: what the receiver gives its consumer ---- */
struct SenderInfo {
    bool present;
    char name[L2K_NAME_SIZE + 1];
    uint32_t caps;
    uint8_t source_kind;
};

/* receiver -> hub */
void hub_sender_present(bool present, const SenderInfo &info);
void hub_skeleton(const l2k_skeleton_frame &frame);
void hub_depth(const uint16_t *pixels, uint32_t frame_number); /* 320x240 */
void hub_color(const uint8_t *bgrx, uint32_t frame_number);    /* 640x480 x 4 */
void hub_tick();       /* called every few ms by the receiver: frame-end liveness, empty frames, summaries */
uint32_t hub_wanted(); /* L2K_STREAM_* the consumer currently needs */
uint32_t hub_skeleton_flags();

} // namespace l2kw

#endif /* L2KW_BASE_H */
