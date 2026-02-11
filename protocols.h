/**
 * protocols.h - Mouse Bridge Protocol Abstraction
 *
 * ═══════════════════════════════════════════════════════════════════════
 * PROTOCOL OVERVIEW
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Three devices, two protocol families:
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │ Device       │ ASCII (Legacy) │ Binary (V2)  │ Default Baud     │
 * ├──────────────┼────────────────┼──────────────┼──────────────────┤
 * │ KMBox B+     │ ✓ (native)     │ ✗            │ 2,000,000        │
 * │ Ferrum One   │ ✓ (compatible) │ ✗            │ 2,000,000        │
 * │ MAKCU        │ ✓ (compatible) │ ✓ (native)   │ 115,200          │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * Ferrum One is a KMBox B+ Pro drop-in replacement — same text protocol,
 * same baud, same command set. The Ferrum "Software API" (custom binary
 * extensions) requires the Ferrum Windows companion app and is not
 * available over direct serial on macOS/Linux.
 *   Source: https://ferrumllc.github.io/
 *           https://xferrum.ai/products/ferrum-one
 *
 * ─── ASCII / Legacy Protocol ───────────────────────────────────────
 *
 * Text-based serial. All three devices support this identically for
 * basic mouse/keyboard injection.
 *
 *   TX (host → device):  km.command(args)\n   or  .command(args)\n
 *   RX (device → host):  km.<echo>\r\n>>>
 *
 *   The "km." prefix is optional on MAKCU (".move(1,1)" works).
 *   Responses start with "km." and end with "\r\n>>>".
 *   Setters echo their input as ACK (suppressible via km.echo(0)).
 *   "ok" and ">>>" indicate success. Lines starting "ERR" are errors.
 *
 *   ── Mouse Commands ──
 *
 *   km.move(dx, dy)
 *       Relative mouse move. dx/dy clamped to [-127, 127] by HID limits.
 *       Supported: KMBox ✓  Ferrum ✓  MAKCU ✓
 *
 *   km.move(dx, dy, segments)
 *       Segmented move — firmware interpolates across N USB frames.
 *       segments: 1-512 (MAKCU), varies on KMBox.
 *       Supported: KMBox ✓  Ferrum ✓  MAKCU ✓
 *
 *   km.move(dx, dy, segments, cx1, cy1, cx2, cy2)
 *       Cubic Bézier curve move. Control points define the curve shape.
 *       The firmware interpolates through the Bézier path over N segments.
 *       Supported: KMBox ?  Ferrum ?  MAKCU ✓
 *
 *   km.moveto(x, y [, segments [, cx1, cy1, cx2, cy2]])
 *       Absolute move — device internally computes delta from current pos.
 *       Requires km.screen(w, h) to be set first.
 *       Supported: KMBox ✗  Ferrum ✗  MAKCU ✓
 *
 *   km.left(state) / km.right(state) / km.middle(state)
 *   km.side1(state) / km.side2(state)
 *       Mouse button control. state: 0=release, 1=down, 2=silent_release.
 *       Silent release sets internal state to 0 without sending a USB frame.
 *       GET (no args) returns lock state: 0=none, 1=raw, 2=injected, 3=both.
 *       Supported: KMBox ✓  Ferrum ✓  MAKCU ✓
 *
 *   km.click(button [, count [, delay_ms]])
 *       Schedule click(s) with optional count and inter-click delay.
 *       If delay_ms omitted, uses random 35-75ms internal timing.
 *       Supported: KMBox ✓  Ferrum ✓  MAKCU ✓
 *
 *   km.wheel(delta)
 *       Scroll wheel. On MAKCU, delta is clamped to ±1 per call.
 *       Supported: KMBox ✓  Ferrum ✓  MAKCU ✓ (clamped)
 *
 *   km.mo(buttons, x, y, wheel, pan, tilt)
 *       Send a complete raw HID mouse frame in one command.
 *       buttons: bitmask (bit0=left, bit1=right, bit2=middle, ...).
 *       x/y/wheel/pan/tilt are one-shot deltas; button mask is stateful.
 *       mo(0) clears all state.
 *       Supported: KMBox ?  Ferrum ?  MAKCU ✓
 *
 *   km.turbo(button [, delay_ms])
 *       Enable rapid-fire on held button. delay auto-rounds to bInterval.
 *       turbo(0) disables all. turbo() queries active settings.
 *       Supported: KMBox ✗  Ferrum ✗  MAKCU ✓
 *
 *   km.silent(x, y)
 *       Move to absolute position then perform silent left click.
 *       Supported: KMBox ✗  Ferrum ✗  MAKCU ✓
 *
 *   km.getpos()
 *       Returns current pointer position as (x, y).
 *       Supported: KMBox ✗  Ferrum ✗  MAKCU ✓
 *
 *   km.screen(w, h)
 *       Set virtual screen dimensions (needed for moveto/getpos).
 *       Supported: KMBox ✗  Ferrum ✗  MAKCU ✓
 *
 *   ── KMBox/Ferrum-Specific ──
 *
 *   km.transform(sx, sy, enable)
 *       Input passthrough transform. sx/sy=0 with enable=1 blocks physical
 *       mouse. sx=256,sy=256 with enable=0 restores passthrough.
 *       Supported: KMBox ✓  Ferrum ?  MAKCU ✗ (use km.bypass instead)
 *
 *   ── MAKCU-Specific Mouse Extensions ──
 *
 *   km.lock_<target>(state)
 *       Lock button or axis. target is part of the command name:
 *       Buttons: ml, mm, mr, ms1, ms2
 *       Axes: mx, my, mw (full), mx+/mx-/my+/my-/mw+/mw- (directional)
 *       state: 1=lock, 0=unlock. GET returns current lock state.
 *
 *   km.catch_<target>(mode)
 *       Enable catch on a locked button (buttons only, not axes).
 *       Requires corresponding km.lock_<target> to be set first.
 *       mode: 0=auto, 1=manual
 *
 *   km.bypass(mode)
 *       Bypass mouse/keyboard USB endpoint.
 *       0=off, 1=mouse bypass (+streaming), 2=keyboard bypass (+streaming)
 *
 *   km.remap_button(src, dst)
 *       Remap physical buttons. Auto-clears conflicts. (0) resets all.
 *       1=left, 2=right, 3=middle, 4=side1, 5=side2
 *
 *   km.remap_axis(inv_x, inv_y, swap)
 *       Remap physical axes. Each flag 0 or 1. (0) resets all.
 *
 *   ── Keyboard Commands ──
 *
 *   km.down(key)        Key down. key: HID code (u8) or 'name'/"name"
 *   km.up(key)          Key up.
 *   km.press(key [, hold_ms [, rand_ms]])
 *                       Press + release. hold_ms default: random 35-75ms.
 *                       Duration auto-rounds to keyboard bInterval.
 *   km.string("text")   Type ASCII string with automatic Shift handling.
 *                       Max 256 chars. Inter-char delay: 10ms. (MAKCU only)
 *   km.init()           Release all pressed keys.
 *   km.isdown(key)      Query key state: 0=up, 1=down.
 *   km.disable(key1, key2, ...)  Block keys from passing through.
 *   km.mask(key, mode)  Mask key. mode: 0=off, 1=on.
 *   km.remap(src, dst)  Remap physical key. (src, 0) clears.
 *
 *   ── Streaming (MAKCU Only) ──
 *
 *   km.mouse(mode, period_ms)    Stream mouse as 8-byte binary frames
 *   km.buttons(mode, period_ms)  Stream button mask (1 byte)
 *   km.axis(mode, period_ms)     Stream axis data as "raw(x,y,w)"
 *   km.keyboard(mode, period_ms) Stream keys as "keyboard(raw,shift,'h')"
 *     mode: 1=raw (physical), 2=constructed (post-remap). (0) disables.
 *     Streaming replies still end with \r\n>>> but payload is binary.
 *
 *   ── Misc Commands ──
 *
 *   km.info()           System info (MAC, temp, RAM, FW, CPU, uptime)
 *   km.version()        Firmware version string
 *   km.device()         Which device is primary: (keyboard)/(mouse)/(none)
 *   km.reboot()         Reboot (responds then reboots)
 *   km.baud(rate)       Change baud. 115200-4000000. (0) resets to 115200.
 *                       Applies immediately; host must reopen at new speed.
 *   km.echo(0|1)        Suppress/enable setter echo ACKs (MAKCU)
 *   km.release(ms)      Auto-release timer. 500-300000ms. (0) disables.
 *   km.serial("text")   Get/set attached device serial number (MAKCU)
 *   km.led(target, mode) LED control: target 1=device, 2=host (MAKCU)
 *   km.log(level)       Log verbosity 0-5 (MAKCU)
 *   km.fault()          Dump parse fault info (MAKCU)
 *   km.hs(0|1)          USB high-speed compat mode (MAKCU, persistent)
 *   km.help()           List available commands
 *
 * ─── MAKCU V2 Binary Protocol ──────────────────────────────────────
 *
 * Lower-latency binary framing. MAKCU-only.
 *
 *   TX frame:  [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
 *   RX setter: [0x50] [CMD] [LEN_LO] [LEN_HI] [0x00=OK | 0x01=ERR]
 *   RX getter: [0x50] [CMD] [LEN_LO] [LEN_HI] [data...]
 *
 *   All multi-byte values are little-endian.
 *
 *   Known command bytes (from MAKCU API docs v3.9):
 *     0x08  left(state)          state: u8 (0/1/2)
 *     0x0A  middle(state)        state: u8 (0/1/2)
 *     0x11  right(state)         state: u8 (0/1/2)
 *     0x12  side1(state)         state: u8 (0/1/2)
 *     0x13  side2(state)         state: u8 (0/1/2)
 *     0xB1  baud(rate)           rate: u32 LE
 *
 *   Binary move/mo command bytes are not yet documented in the public
 *   MAKCU API reference. Use the ASCII km.move() protocol for mouse
 *   movement — it works on all devices including MAKCU.
 *
 *   Legacy baud frame (also supported on MAKCU):
 *     [0xDE] [0xAD] [LEN_LO] [LEN_HI] [0xA5] [baud_LE32]
 *
 * ═══════════════════════════════════════════════════════════════════════
 */

#ifndef PROTOCOLS_H
#define PROTOCOLS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Types
// ============================================================================

typedef enum {
    PROTO_ASCII,        // Text protocol (km.move(), etc.) — all devices
    PROTO_MAKCU_BIN,    // MAKCU V2 binary (0x50 framing) — MAKCU only
    PROTO_COUNT
} proto_type_t;

typedef enum {
    DEV_KMBOX,          // KMBox B/B+ — native ASCII, 2M baud
    DEV_FERRUM,         // Ferrum One  — KMBox-compatible, 2M baud
    DEV_MAKCU,          // MAKCU       — ASCII + binary V2, 115200 baud
    DEV_COUNT
} device_type_t;

typedef enum {
    PROTO_OK,
    PROTO_ERR_WRITE,
    PROTO_ERR_TIMEOUT,
    PROTO_ERR_NACK,
    PROTO_ERR_INVALID,
} proto_result_t;

typedef void (*proto_response_cb_t)(proto_result_t result, const uint8_t* data,
                                    uint16_t len, void* ctx);

// ============================================================================
// Device Profile
// ============================================================================

typedef struct {
    const char*   name;           // "KMBox B+", "Ferrum One", "MAKCU"
    const char*   short_name;     // "kmbox", "ferrum", "makcu"
    device_type_t device;
    int           default_baud;
    int16_t       max_delta;      // Max single-axis delta (127 for HID)
    bool          has_transform;  // km.transform()
    bool          has_bezier;     // Bézier curve moves
    bool          has_moveto;     // km.moveto()
    bool          has_binary_v2;  // MAKCU V2 binary frames
    bool          has_streaming;  // Mouse/button/axis streaming
    bool          has_lock;       // Button/axis locking
    bool          has_turbo;      // Rapid-fire mode
    bool          has_raw_frame;  // km.mo() raw frame command
    uint32_t      min_interval_us;
} device_profile_t;

// ============================================================================
// Protocol Operations & State
// ============================================================================

typedef struct proto_state proto_state_t;

typedef struct {
    int (*fmt_move)(uint8_t* buf, size_t buflen, int16_t dx, int16_t dy);
    int (*fmt_button)(uint8_t* buf, size_t buflen, const char* btn, uint8_t state);
    int (*fmt_scroll)(uint8_t* buf, size_t buflen, int8_t delta);
    int (*fmt_transform)(uint8_t* buf, size_t buflen, int16_t sx, int16_t sy, bool en);
    int (*parse_response)(proto_state_t* s, const uint8_t* data, size_t len,
                          proto_response_cb_t cb, void* ctx);
    void (*reset_parser)(proto_state_t* s);
} proto_ops_t;

struct proto_state {
    proto_type_t            type;
    const device_profile_t* profile;
    proto_ops_t             ops;
    uint8_t                 rx_buf[512];
    uint16_t                rx_pos;
    uint8_t                 parse_state;
    uint16_t                parse_expect;
};

// ============================================================================
// API
// ============================================================================

void                    proto_init(proto_state_t* state, device_type_t device);
const device_profile_t* proto_get_profile(device_type_t device);
device_type_t           device_from_name(const char* name);

// Inline convenience wrappers
static inline int proto_fmt_move(proto_state_t* s, uint8_t* b, size_t l,
                                 int16_t dx, int16_t dy)
    { return s->ops.fmt_move(b, l, dx, dy); }

static inline int proto_fmt_button(proto_state_t* s, uint8_t* b, size_t l,
                                   const char* btn, uint8_t st)
    { return s->ops.fmt_button ? s->ops.fmt_button(b, l, btn, st) : -1; }

static inline int proto_fmt_scroll(proto_state_t* s, uint8_t* b, size_t l,
                                   int8_t d)
    { return s->ops.fmt_scroll ? s->ops.fmt_scroll(b, l, d) : -1; }

static inline int proto_fmt_transform(proto_state_t* s, uint8_t* b, size_t l,
                                      int16_t sx, int16_t sy, bool en)
    { return s->ops.fmt_transform ? s->ops.fmt_transform(b, l, sx, sy, en) : -1; }

static inline int proto_parse(proto_state_t* s, const uint8_t* d, size_t l,
                              proto_response_cb_t cb, void* ctx)
    { return s->ops.parse_response(s, d, l, cb, ctx); }

static inline void proto_reset(proto_state_t* s)
    { if (s->ops.reset_parser) s->ops.reset_parser(s); }

#endif // PROTOCOLS_H
