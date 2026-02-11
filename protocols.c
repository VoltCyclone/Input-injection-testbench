/**
 * protocols.c - Mouse Bridge Protocol Implementations
 *
 * Two protocol implementations, three device profiles:
 *
 *   1. ASCII protocol  — shared by KMBox B+, Ferrum One, and MAKCU
 *      TX: km.move(dx, dy)\n
 *      RX: km.<echo>\r\n>>>
 *
 *   2. MAKCU V2 Binary — MAKCU only
 *      TX: [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
 *      RX: [0x50] [CMD] [LEN_LO] [LEN_HI] [status|data...]
 *
 * References:
 *   MAKCU API v3.9:  https://www.makcu.com/en/api
 *   Ferrum docs:     https://ferrumllc.github.io/
 */

#include "protocols.h"
#include <string.h>
#include <stdio.h>
#include <strings.h>  // strcasecmp

// ============================================================================
// Device Profiles
// ============================================================================

static const device_profile_t profiles[DEV_COUNT] = {
    [DEV_KMBOX] = {
        .name           = "KMBox B+",
        .short_name     = "kmbox",
        .device         = DEV_KMBOX,
        .default_baud   = 2000000,
        .max_delta      = 127,
        .has_transform  = true,
        .has_bezier     = false,
        .has_moveto     = false,
        .has_binary_v2  = false,
        .has_streaming  = false,
        .has_lock       = false,
        .has_turbo      = false,
        .has_raw_frame  = false,
        .min_interval_us = 4000,   // ~250 Hz safe rate at 2M baud
    },
    [DEV_FERRUM] = {
        .name           = "Ferrum One",
        .short_name     = "ferrum",
        .device         = DEV_FERRUM,
        .default_baud   = 2000000,
        .max_delta      = 127,
        .has_transform  = true,      // Assumed KMBox-compatible
        .has_bezier     = false,
        .has_moveto     = false,
        .has_binary_v2  = false,
        .has_streaming  = false,
        .has_lock       = false,
        .has_turbo      = false,
        .has_raw_frame  = false,
        .min_interval_us = 4000,
    },
    [DEV_MAKCU] = {
        .name           = "MAKCU",
        .short_name     = "makcu",
        .device         = DEV_MAKCU,
        .default_baud   = 115200,
        .max_delta      = 127,
        .has_transform  = false,     // Uses km.bypass() instead
        .has_bezier     = true,      // km.move(dx,dy,segs,cx1,cy1,cx2,cy2)
        .has_moveto     = true,      // km.moveto(x,y)
        .has_binary_v2  = true,      // [0x50] framed binary protocol
        .has_streaming  = true,      // km.mouse(mode,period), km.buttons()...
        .has_lock       = true,      // km.lock_<target>(), km.catch_<target>()
        .has_turbo      = true,      // km.turbo(btn,delay_ms)
        .has_raw_frame  = true,      // km.mo(btns,x,y,whl,pan,tilt)
        .min_interval_us = 2000,     // ~500 Hz at 115200 baud
    },
};

// ============================================================================
// ASCII Protocol (KMBox / Ferrum / MAKCU shared)
// ============================================================================
//
// All three devices accept the same text command format over serial:
//   TX: km.command(args)\n
//   RX: km.<echo>\r\n>>>
//
// The "km." prefix is optional on MAKCU (".move(1,1)" works).
// Ferrum One is a documented KMBox B+ Pro drop-in replacement.

static int ascii_fmt_move(uint8_t* buf, size_t buflen, int16_t dx, int16_t dy) {
    return snprintf((char*)buf, buflen, "km.move(%d,%d)\r\n", dx, dy);
}

static int ascii_fmt_button(uint8_t* buf, size_t buflen,
                            const char* button, uint8_t state) {
    // button: "left", "right", "middle", "side1", "side2"
    // state:  0=release, 1=down, 2=silent_release (MAKCU only)
    return snprintf((char*)buf, buflen, "km.%s(%u)\r\n", button, state);
}

static int ascii_fmt_scroll(uint8_t* buf, size_t buflen, int8_t delta) {
    return snprintf((char*)buf, buflen, "km.wheel(%d)\r\n", delta);
}

static int ascii_fmt_transform(uint8_t* buf, size_t buflen,
                               int16_t sx, int16_t sy, bool enable) {
    // KMBox/Ferrum: km.transform(sx, sy, enable)
    // Controls physical mouse passthrough.
    //   sx=0, sy=0, enable=1  -> block physical mouse
    //   sx=256, sy=256, enable=0 -> restore passthrough
    return snprintf((char*)buf, buflen, "km.transform(%d,%d,%d)\r\n",
                    sx, sy, enable ? 1 : 0);
}

// ── ASCII Response Parser ──
//
// Responses are line-based text ending in \r\n, with a ">>>" prompt
// after each complete response. We accumulate bytes until we see \n,
// then classify the line:
//   ">>>"           -> PROTO_OK (prompt, command completed)
//   "ok"            -> PROTO_OK
//   "km.<echo>"     -> PROTO_OK (setter echo)
//   "ERR..."        -> PROTO_ERR_NACK
//
// Binary streaming responses (km.mouse<8 bytes>, km.buttons<1 byte>)
// from MAKCU also arrive in this stream but are terminated the same way.

static int ascii_parse_response(proto_state_t* s, const uint8_t* data, size_t len,
                                proto_response_cb_t cb, void* ctx) {
    int consumed = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        consumed++;

        if (c == '\n') {
            if (s->rx_pos > 0) {
                // Strip trailing \r
                if (s->rx_pos > 0 && s->rx_buf[s->rx_pos - 1] == '\r')
                    s->rx_pos--;
                s->rx_buf[s->rx_pos] = '\0';

                proto_result_t result;
                if (strcmp((char*)s->rx_buf, ">>>") == 0) {
                    result = PROTO_OK;
                } else if (strcmp((char*)s->rx_buf, "ok") == 0) {
                    result = PROTO_OK;
                } else if (strncmp((char*)s->rx_buf, "ERR", 3) == 0) {
                    result = PROTO_ERR_NACK;
                } else if (strncmp((char*)s->rx_buf, "km.", 3) == 0) {
                    // Setter echo or getter response — both OK
                    result = PROTO_OK;
                } else {
                    result = PROTO_OK; // Unknown line, treat as info
                }

                if (cb) cb(result, s->rx_buf, s->rx_pos, ctx);
                s->rx_pos = 0;
            }
        } else if (c == '\r') {
            // Buffer it; will be stripped when \n arrives
            if (s->rx_pos < sizeof(s->rx_buf) - 1)
                s->rx_buf[s->rx_pos++] = c;
        } else {
            if (s->rx_pos < sizeof(s->rx_buf) - 1)
                s->rx_buf[s->rx_pos++] = c;
        }
    }
    return consumed;
}

static void ascii_reset_parser(proto_state_t* s) {
    s->rx_pos = 0;
    s->parse_state = 0;
}

// ============================================================================
// MAKCU V2 Binary Protocol
// ============================================================================
//
// Frame format (from MAKCU API v3.9 docs):
//   TX: [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
//   RX setter: [0x50] [CMD] [LEN_LO] [LEN_HI] [0x00=OK | 0x01=ERR]
//   RX getter: [0x50] [CMD] [LEN_LO] [LEN_HI] [data...]
//
// All multi-byte values are little-endian.
//
// Also supports legacy baud rate frame:
//   [0xDE] [0xAD] [LEN_LO] [LEN_HI] [0xA5] [baud_LE32]
//
// Known command bytes (documented in MAKCU API):
//   0x08 = left(state)
//   0x0A = middle(state)
//   0x11 = right(state)
//   0x12 = side1(state)
//   0x13 = side2(state)
//   0xB1 = baud(rate)
//
// Note: Binary mouse move command byte is not documented in the public
// MAKCU API as of v3.9. For mouse movement, the ASCII km.move() command
// works on MAKCU and is what this protocol falls back to. The binary
// framing is used here for button commands and future expansion as
// more command bytes are documented.

#define MAKCU_FRAME_START    0x50
#define MAKCU_CMD_LEFT       0x08
#define MAKCU_CMD_MIDDLE     0x0A
#define MAKCU_CMD_RIGHT      0x11
#define MAKCU_CMD_SIDE1      0x12
#define MAKCU_CMD_SIDE2      0x13
#define MAKCU_CMD_BAUD       0xB1

// Build a V2 binary frame: [0x50] [cmd] [len_lo] [len_hi] [payload...]
static int makcu_bin_frame(uint8_t* buf, size_t buflen,
                           uint8_t cmd, const uint8_t* payload, uint16_t plen) {
    size_t total = 4 + plen;  // start + cmd + len(2) + payload
    if (total > buflen) return -1;

    buf[0] = MAKCU_FRAME_START;
    buf[1] = cmd;
    buf[2] = (uint8_t)(plen & 0xFF);         // LEN_LO
    buf[3] = (uint8_t)((plen >> 8) & 0xFF);  // LEN_HI
    if (plen > 0 && payload)
        memcpy(&buf[4], payload, plen);
    return (int)total;
}

// Binary move: not yet documented, fall back to ASCII
static int makcu_bin_fmt_move(uint8_t* buf, size_t buflen,
                              int16_t dx, int16_t dy) {
    // The MAKCU V2 binary protocol does not document a mouse move
    // command byte. Fall back to ASCII km.move() which MAKCU supports.
    return ascii_fmt_move(buf, buflen, dx, dy);
}

// Binary button: uses documented command bytes
static int makcu_bin_fmt_button(uint8_t* buf, size_t buflen,
                                const char* button, uint8_t state) {
    uint8_t cmd;
    if      (strcasecmp(button, "left")   == 0) cmd = MAKCU_CMD_LEFT;
    else if (strcasecmp(button, "right")  == 0) cmd = MAKCU_CMD_RIGHT;
    else if (strcasecmp(button, "middle") == 0) cmd = MAKCU_CMD_MIDDLE;
    else if (strcasecmp(button, "side1")  == 0) cmd = MAKCU_CMD_SIDE1;
    else if (strcasecmp(button, "side2")  == 0) cmd = MAKCU_CMD_SIDE2;
    else return ascii_fmt_button(buf, buflen, button, state); // Unknown, use ASCII

    return makcu_bin_frame(buf, buflen, cmd, &state, 1);
}

// Binary scroll: not yet documented, fall back to ASCII
static int makcu_bin_fmt_scroll(uint8_t* buf, size_t buflen, int8_t delta) {
    return ascii_fmt_scroll(buf, buflen, delta);
}

// ── MAKCU V2 Binary Parser ──
//
// State machine for [0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
// Since MAKCU also responds to ASCII commands with ASCII responses,
// this parser handles both: if we see 0x50, we parse binary; otherwise
// we fall through to accumulating text lines.

enum {
    MBIN_IDLE = 0,    // Waiting for 0x50 or text
    MBIN_CMD,         // Reading CMD byte
    MBIN_LEN_LO,     // Reading LEN_LO
    MBIN_LEN_HI,     // Reading LEN_HI
    MBIN_PAYLOAD,     // Reading payload bytes
};

static int makcu_bin_parse_response(proto_state_t* s, const uint8_t* data, size_t len,
                                    proto_response_cb_t cb, void* ctx) {
    int consumed = 0;

    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];
        consumed++;

        switch (s->parse_state) {
        case MBIN_IDLE:
            if (c == MAKCU_FRAME_START) {
                // Start of binary frame
                s->parse_state = MBIN_CMD;
                s->rx_pos = 0;
                s->parse_expect = 0;
            } else {
                // ASCII text — accumulate and parse as text line
                if (c == '\n') {
                    if (s->rx_pos > 0) {
                        if (s->rx_pos > 0 && s->rx_buf[s->rx_pos - 1] == '\r')
                            s->rx_pos--;
                        s->rx_buf[s->rx_pos] = '\0';

                        proto_result_t result;
                        if (strcmp((char*)s->rx_buf, ">>>") == 0)
                            result = PROTO_OK;
                        else if (strncmp((char*)s->rx_buf, "ERR", 3) == 0)
                            result = PROTO_ERR_NACK;
                        else
                            result = PROTO_OK;

                        if (cb) cb(result, s->rx_buf, s->rx_pos, ctx);
                        s->rx_pos = 0;
                    }
                } else if (s->rx_pos < sizeof(s->rx_buf) - 1) {
                    s->rx_buf[s->rx_pos++] = c;
                }
            }
            break;

        case MBIN_CMD:
            s->rx_buf[0] = c;  // Store CMD
            s->parse_state = MBIN_LEN_LO;
            break;

        case MBIN_LEN_LO:
            s->rx_buf[1] = c;  // LEN_LO
            s->parse_state = MBIN_LEN_HI;
            break;

        case MBIN_LEN_HI:
            s->rx_buf[2] = c;  // LEN_HI
            s->parse_expect = (uint16_t)s->rx_buf[1] | ((uint16_t)c << 8);
            if (s->parse_expect > sizeof(s->rx_buf) - 10) {
                // Sanity: too large, reset
                s->parse_state = MBIN_IDLE;
                s->rx_pos = 0;
                break;
            }
            s->rx_pos = 0;
            if (s->parse_expect == 0) {
                // Zero-length payload — complete frame
                if (cb) cb(PROTO_OK, s->rx_buf, 3, ctx);
                s->parse_state = MBIN_IDLE;
            } else {
                s->parse_state = MBIN_PAYLOAD;
            }
            break;

        case MBIN_PAYLOAD:
            if (s->rx_pos + 3 < sizeof(s->rx_buf)) {
                s->rx_buf[3 + s->rx_pos] = c;
            }
            s->rx_pos++;
            if (s->rx_pos >= s->parse_expect) {
                // Complete frame received
                // rx_buf layout: [CMD] [LEN_LO] [LEN_HI] [payload...]
                // For setters, payload[0] is 0x00=OK or 0x01=ERR
                proto_result_t result = PROTO_OK;
                if (s->parse_expect >= 1 && s->rx_buf[3] == 0x01)
                    result = PROTO_ERR_NACK;

                uint16_t total_len = 3 + s->parse_expect;
                if (cb) cb(result, s->rx_buf, total_len, ctx);

                s->parse_state = MBIN_IDLE;
                s->rx_pos = 0;
            }
            break;
        }
    }
    return consumed;
}

static void makcu_bin_reset_parser(proto_state_t* s) {
    s->rx_pos = 0;
    s->parse_state = MBIN_IDLE;
    s->parse_expect = 0;
}

// ============================================================================
// Protocol API
// ============================================================================

void proto_init(proto_state_t* state, device_type_t device) {
    memset(state, 0, sizeof(*state));

    if (device >= DEV_COUNT) device = DEV_KMBOX;
    state->profile = &profiles[device];

    // All three devices support ASCII. MAKCU can also use binary V2.
    // Default to ASCII for KMBox and Ferrum, binary V2 for MAKCU.
    if (device == DEV_MAKCU) {
        state->type = PROTO_MAKCU_BIN;
        state->ops = (proto_ops_t){
            .fmt_move       = makcu_bin_fmt_move,
            .fmt_button     = makcu_bin_fmt_button,
            .fmt_scroll     = makcu_bin_fmt_scroll,
            .fmt_transform  = NULL,  // MAKCU uses km.bypass() instead
            .parse_response = makcu_bin_parse_response,
            .reset_parser   = makcu_bin_reset_parser,
        };
    } else {
        state->type = PROTO_ASCII;
        state->ops = (proto_ops_t){
            .fmt_move       = ascii_fmt_move,
            .fmt_button     = ascii_fmt_button,
            .fmt_scroll     = ascii_fmt_scroll,
            .fmt_transform  = ascii_fmt_transform,
            .parse_response = ascii_parse_response,
            .reset_parser   = ascii_reset_parser,
        };
    }
}

const device_profile_t* proto_get_profile(device_type_t device) {
    if (device >= DEV_COUNT) return NULL;
    return &profiles[device];
}

device_type_t device_from_name(const char* name) {
    if (!name) return DEV_KMBOX;
    for (int i = 0; i < DEV_COUNT; i++) {
        if (strcasecmp(name, profiles[i].short_name) == 0) return (device_type_t)i;
        if (strcasecmp(name, profiles[i].name) == 0)       return (device_type_t)i;
    }
    return DEV_KMBOX;
}
