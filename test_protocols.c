/**
 * test_protocols.c - Unit tests for protocol formatting and parsing
 *
 * Lightweight assert-based test harness. No external dependencies.
 * Tests the pure-C protocol layer (protocols.c) without hardware or
 * macOS frameworks.
 *
 * Build:  make test
 * Run:    ./test_protocols
 */

#include "protocols.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// ============================================================================
// Minimal test framework
// ============================================================================

static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;
static const char* g_current_test = NULL;

#define TEST(name) \
    static void test_##name(void); \
    static void run_test_##name(void) { \
        g_current_test = #name; \
        g_tests_run++; \
        test_##name(); \
        g_tests_passed++; \
        printf("  ✓ %s\n", #name); \
    } \
    static void test_##name(void)

#define ASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "  ✗ %s — ASSERT FAILED: %s (line %d)\n", \
                g_current_test, #cond, __LINE__); \
        g_tests_failed++; \
        g_tests_passed--;  /* undo the pre-increment in run_ wrapper */ \
        return; \
    } \
} while(0)

#define ASSERT_EQ_INT(a, b) do { \
    long _a = (long)(a), _b = (long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "  ✗ %s — ASSERT_EQ_INT: %ld != %ld (line %d)\n", \
                g_current_test, _a, _b, __LINE__); \
        g_tests_failed++; \
        g_tests_passed--; \
        return; \
    } \
} while(0)

#define ASSERT_STR_EQ(a, b) do { \
    if (strcmp((a), (b)) != 0) { \
        fprintf(stderr, "  ✗ %s — ASSERT_STR_EQ: \"%s\" != \"%s\" (line %d)\n", \
                g_current_test, (a), (b), __LINE__); \
        g_tests_failed++; \
        g_tests_passed--; \
        return; \
    } \
} while(0)

// ============================================================================
// Callback helpers for parse tests
// ============================================================================

typedef struct {
    proto_result_t results[32];
    uint16_t       lengths[32];
    char           lines[32][256];
    int            count;
} parse_log_t;

static void log_cb(proto_result_t result, const uint8_t* data,
                   uint16_t len, void* ctx) {
    parse_log_t* log = (parse_log_t*)ctx;
    if (log->count >= 32) return;
    int i = log->count++;
    log->results[i] = result;
    log->lengths[i] = len;
    size_t copy = (len < 255) ? len : 255;
    memcpy(log->lines[i], data, copy);
    log->lines[i][copy] = '\0';
}

// ============================================================================
// Device profile tests
// ============================================================================

TEST(profile_count) {
    // All three devices should be retrievable
    ASSERT(proto_get_profile(DEV_KMBOX)  != NULL);
    ASSERT(proto_get_profile(DEV_FERRUM) != NULL);
    ASSERT(proto_get_profile(DEV_MAKCU)  != NULL);
    ASSERT(proto_get_profile(DEV_COUNT)  == NULL);
}

TEST(profile_kmbox) {
    const device_profile_t* p = proto_get_profile(DEV_KMBOX);
    ASSERT_STR_EQ(p->short_name, "kmbox");
    ASSERT_EQ_INT(p->default_baud, 2000000);
    ASSERT_EQ_INT(p->max_delta, 127);
    ASSERT(p->has_transform == true);
    ASSERT(p->has_bezier == false);
    ASSERT(p->has_binary_v2 == false);
}

TEST(profile_ferrum) {
    const device_profile_t* p = proto_get_profile(DEV_FERRUM);
    ASSERT_STR_EQ(p->short_name, "ferrum");
    ASSERT_EQ_INT(p->default_baud, 2000000);
    ASSERT(p->has_transform == true);
    ASSERT(p->has_binary_v2 == false);
}

TEST(profile_makcu) {
    const device_profile_t* p = proto_get_profile(DEV_MAKCU);
    ASSERT_STR_EQ(p->short_name, "makcu");
    ASSERT_EQ_INT(p->default_baud, 115200);
    ASSERT(p->has_transform == false);
    ASSERT(p->has_bezier == true);
    ASSERT(p->has_moveto == true);
    ASSERT(p->has_binary_v2 == true);
    ASSERT(p->has_streaming == true);
    ASSERT(p->has_lock == true);
    ASSERT(p->has_turbo == true);
    ASSERT(p->has_raw_frame == true);
}

TEST(device_from_name_short) {
    ASSERT_EQ_INT(device_from_name("kmbox"),  DEV_KMBOX);
    ASSERT_EQ_INT(device_from_name("ferrum"), DEV_FERRUM);
    ASSERT_EQ_INT(device_from_name("makcu"),  DEV_MAKCU);
}

TEST(device_from_name_full) {
    ASSERT_EQ_INT(device_from_name("KMBox B+"),   DEV_KMBOX);
    ASSERT_EQ_INT(device_from_name("Ferrum One"),  DEV_FERRUM);
    ASSERT_EQ_INT(device_from_name("MAKCU"),       DEV_MAKCU);
}

TEST(device_from_name_case_insensitive) {
    ASSERT_EQ_INT(device_from_name("KMBOX"),  DEV_KMBOX);
    ASSERT_EQ_INT(device_from_name("Ferrum"), DEV_FERRUM);
    ASSERT_EQ_INT(device_from_name("MAKCU"),  DEV_MAKCU);
}

TEST(device_from_name_null_and_unknown) {
    // NULL and unknown strings default to DEV_KMBOX
    ASSERT_EQ_INT(device_from_name(NULL),      DEV_KMBOX);
    ASSERT_EQ_INT(device_from_name("unknown"), DEV_KMBOX);
    ASSERT_EQ_INT(device_from_name(""),        DEV_KMBOX);
}

// ============================================================================
// ASCII format tests
// ============================================================================

TEST(ascii_fmt_move_basic) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];
    int len = proto_fmt_move(&s, buf, sizeof(buf), 10, -5);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.move(10,-5)\r\n");
}

TEST(ascii_fmt_move_zero) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];
    int len = proto_fmt_move(&s, buf, sizeof(buf), 0, 0);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.move(0,0)\r\n");
}

TEST(ascii_fmt_move_extremes) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];

    int len = proto_fmt_move(&s, buf, sizeof(buf), 127, 127);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.move(127,127)\r\n");

    len = proto_fmt_move(&s, buf, sizeof(buf), -127, -127);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.move(-127,-127)\r\n");
}

TEST(ascii_fmt_move_buffer_too_small) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[5];  // Way too small
    // snprintf returns what would have been written, but truncates
    int len = proto_fmt_move(&s, buf, sizeof(buf), 10, 20);
    ASSERT(len > (int)sizeof(buf) - 1);  // Would have needed more space
}

TEST(ascii_fmt_button) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];

    int len = proto_fmt_button(&s, buf, sizeof(buf), "left", 1);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.left(1)\r\n");

    len = proto_fmt_button(&s, buf, sizeof(buf), "right", 0);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.right(0)\r\n");

    len = proto_fmt_button(&s, buf, sizeof(buf), "middle", 2);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.middle(2)\r\n");
}

TEST(ascii_fmt_scroll) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];

    int len = proto_fmt_scroll(&s, buf, sizeof(buf), 1);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.wheel(1)\r\n");

    len = proto_fmt_scroll(&s, buf, sizeof(buf), -1);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.wheel(-1)\r\n");
}

TEST(ascii_fmt_transform) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    uint8_t buf[64];

    // Block physical mouse
    int len = proto_fmt_transform(&s, buf, sizeof(buf), 0, 0, true);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.transform(0,0,1)\r\n");

    // Restore passthrough
    len = proto_fmt_transform(&s, buf, sizeof(buf), 256, 256, false);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.transform(256,256,0)\r\n");
}

TEST(makcu_no_transform) {
    // MAKCU should not have fmt_transform — should return -1
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_transform(&s, buf, sizeof(buf), 0, 0, true);
    ASSERT_EQ_INT(len, -1);
}

// ============================================================================
// ASCII parse tests
// ============================================================================

TEST(ascii_parse_ok_prompt) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = ">>>\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
    ASSERT_STR_EQ(log.lines[0], ">>>");
}

TEST(ascii_parse_echo_response) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = "km.move(10,-5)\r\n>>>\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 2);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);  // echo
    ASSERT_STR_EQ(log.lines[0], "km.move(10,-5)");
    ASSERT_EQ_INT(log.results[1], PROTO_OK);  // prompt
}

TEST(ascii_parse_error) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = "ERR: invalid command\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_ERR_NACK);
}

TEST(ascii_parse_ok_text) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = "ok\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(ascii_parse_incremental) {
    // Feed data byte-by-byte — parser should accumulate correctly
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = ">>>\r\n";
    for (size_t i = 0; i < strlen(data); i++) {
        proto_parse(&s, (const uint8_t*)&data[i], 1, log_cb, &log);
    }

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(ascii_parse_multiple_lines) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    const char* data = "km.left(1)\r\nok\r\n>>>\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 3);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
    ASSERT_EQ_INT(log.results[1], PROTO_OK);
    ASSERT_EQ_INT(log.results[2], PROTO_OK);
}

TEST(ascii_parse_null_callback) {
    // Should not crash with NULL callback
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);

    const char* data = ">>>\r\n";
    int consumed = proto_parse(&s, (const uint8_t*)data, strlen(data), NULL, NULL);
    ASSERT_EQ_INT(consumed, (int)strlen(data));
}

TEST(ascii_parse_empty_lines) {
    // Bare \r\n lines: the parser buffers \r, then strips it on \n,
    // resulting in a zero-length line which does NOT fire the callback
    // (rx_pos check). However, \r alone is buffered, so the parser
    // treats each \r\n as a line with just \r that gets stripped.
    // In practice, the parser fires callbacks for any line where
    // rx_pos > 0 before the strip — \r counts.
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    parse_log_t log = {0};

    // Three lines: \r\n, \r\n, >>>\r\n
    // Each \r\n has rx_pos=1 (\r buffered), then \r stripped -> empty string
    // but rx_pos was > 0 so callback fires with empty string
    const char* data = "\r\n\r\n>>>\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 3);  // Two empty-line callbacks + ">>>"
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
    ASSERT_EQ_INT(log.results[1], PROTO_OK);
    ASSERT_EQ_INT(log.results[2], PROTO_OK);
}

TEST(ascii_reset_clears_state) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);

    // Feed partial data
    const char* partial = "km.mov";
    proto_parse(&s, (const uint8_t*)partial, strlen(partial), NULL, NULL);
    ASSERT(s.rx_pos > 0);

    // Reset
    proto_reset(&s);
    ASSERT_EQ_INT(s.rx_pos, 0);
}

// ============================================================================
// MAKCU binary format tests
// ============================================================================

TEST(makcu_bin_fmt_move_falls_back_to_ascii) {
    // MAKCU binary move is undocumented; should produce ASCII km.move()
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_move(&s, buf, sizeof(buf), 10, -5);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.move(10,-5)\r\n");
}

TEST(makcu_bin_fmt_button_left) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_button(&s, buf, sizeof(buf), "left", 1);
    ASSERT_EQ_INT(len, 5);  // [0x50][CMD][LEN_LO][LEN_HI][state]
    ASSERT_EQ_INT(buf[0], 0x50);
    ASSERT_EQ_INT(buf[1], 0x08);  // MAKCU_CMD_LEFT
    ASSERT_EQ_INT(buf[2], 1);     // payload length low
    ASSERT_EQ_INT(buf[3], 0);     // payload length high
    ASSERT_EQ_INT(buf[4], 1);     // state = down
}

TEST(makcu_bin_fmt_button_right) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_button(&s, buf, sizeof(buf), "right", 0);
    ASSERT_EQ_INT(len, 5);
    ASSERT_EQ_INT(buf[1], 0x11);  // MAKCU_CMD_RIGHT
    ASSERT_EQ_INT(buf[4], 0);     // state = release
}

TEST(makcu_bin_fmt_button_middle) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_button(&s, buf, sizeof(buf), "middle", 2);
    ASSERT_EQ_INT(len, 5);
    ASSERT_EQ_INT(buf[1], 0x0A);  // MAKCU_CMD_MIDDLE
    ASSERT_EQ_INT(buf[4], 2);     // state = silent_release
}

TEST(makcu_bin_fmt_button_side) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];

    int len = proto_fmt_button(&s, buf, sizeof(buf), "side1", 1);
    ASSERT_EQ_INT(len, 5);
    ASSERT_EQ_INT(buf[1], 0x12);  // MAKCU_CMD_SIDE1

    len = proto_fmt_button(&s, buf, sizeof(buf), "side2", 1);
    ASSERT_EQ_INT(len, 5);
    ASSERT_EQ_INT(buf[1], 0x13);  // MAKCU_CMD_SIDE2
}

TEST(makcu_bin_fmt_button_unknown_falls_back_to_ascii) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_button(&s, buf, sizeof(buf), "unknown_btn", 1);
    ASSERT(len > 0);
    // Should fall back to ASCII format
    ASSERT_STR_EQ((char*)buf, "km.unknown_btn(1)\r\n");
}

TEST(makcu_bin_fmt_scroll_falls_back_to_ascii) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    uint8_t buf[64];
    int len = proto_fmt_scroll(&s, buf, sizeof(buf), 1);
    ASSERT(len > 0);
    ASSERT_STR_EQ((char*)buf, "km.wheel(1)\r\n");
}

// ============================================================================
// MAKCU binary parse tests
// ============================================================================

TEST(makcu_bin_parse_ok_response) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    // Binary response: [0x50] [CMD=0x08] [LEN=1,0] [0x00=OK]
    uint8_t data[] = {0x50, 0x08, 0x01, 0x00, 0x00};
    proto_parse(&s, data, sizeof(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(makcu_bin_parse_err_response) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    // Binary response with error: [0x50] [CMD=0x08] [LEN=1,0] [0x01=ERR]
    uint8_t data[] = {0x50, 0x08, 0x01, 0x00, 0x01};
    proto_parse(&s, data, sizeof(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_ERR_NACK);
}

TEST(makcu_bin_parse_zero_length_payload) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    // Binary response with zero payload: [0x50] [CMD=0x08] [LEN=0,0]
    uint8_t data[] = {0x50, 0x08, 0x00, 0x00};
    proto_parse(&s, data, sizeof(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(makcu_bin_parse_ascii_fallthrough) {
    // MAKCU binary parser should also handle ASCII text lines
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    const char* data = "km.move(10,-5)\r\n>>>\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 2);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
    ASSERT_EQ_INT(log.results[1], PROTO_OK);
}

TEST(makcu_bin_parse_ascii_error_fallthrough) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    const char* data = "ERR: bad cmd\r\n";
    proto_parse(&s, (const uint8_t*)data, strlen(data), log_cb, &log);

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_ERR_NACK);
}

TEST(makcu_bin_parse_mixed_binary_and_ascii) {
    // Binary frame followed by ASCII prompt
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    uint8_t data[32];
    int pos = 0;
    // Binary OK response
    data[pos++] = 0x50; data[pos++] = 0x08;
    data[pos++] = 0x01; data[pos++] = 0x00;
    data[pos++] = 0x00; // OK
    // ASCII prompt
    const char* prompt = ">>>\r\n";
    memcpy(&data[pos], prompt, strlen(prompt));
    pos += strlen(prompt);

    proto_parse(&s, data, pos, log_cb, &log);

    ASSERT_EQ_INT(log.count, 2);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);  // binary
    ASSERT_EQ_INT(log.results[1], PROTO_OK);  // ASCII prompt
}

TEST(makcu_bin_parse_incremental) {
    // Feed binary frame byte-by-byte
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    uint8_t data[] = {0x50, 0x08, 0x01, 0x00, 0x00};
    for (size_t i = 0; i < sizeof(data); i++) {
        proto_parse(&s, &data[i], 1, log_cb, &log);
    }

    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(makcu_bin_parse_oversized_resets) {
    // A frame claiming a huge payload should reset, not overflow
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    parse_log_t log = {0};

    // Claim payload length of 0xFFFF (> rx_buf capacity)
    uint8_t data[] = {0x50, 0x08, 0xFF, 0xFF};
    proto_parse(&s, data, sizeof(data), log_cb, &log);

    // Should reset and not fire callback
    ASSERT_EQ_INT(log.count, 0);

    // Parser should recover — feed a normal ASCII line after
    const char* recovery = ">>>\r\n";
    proto_parse(&s, (const uint8_t*)recovery, strlen(recovery), log_cb, &log);
    ASSERT_EQ_INT(log.count, 1);
    ASSERT_EQ_INT(log.results[0], PROTO_OK);
}

TEST(makcu_bin_reset_clears_state) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);

    // Feed partial binary frame
    uint8_t partial[] = {0x50, 0x08};
    proto_parse(&s, partial, sizeof(partial), NULL, NULL);

    proto_reset(&s);
    ASSERT_EQ_INT(s.rx_pos, 0);
    ASSERT_EQ_INT(s.parse_state, 0);  // MBIN_IDLE
    ASSERT_EQ_INT(s.parse_expect, 0);
}

// ============================================================================
// Proto init tests
// ============================================================================

TEST(proto_init_kmbox_ascii) {
    proto_state_t s;
    proto_init(&s, DEV_KMBOX);
    ASSERT_EQ_INT(s.type, PROTO_ASCII);
    ASSERT(s.profile != NULL);
    ASSERT_STR_EQ(s.profile->short_name, "kmbox");
    ASSERT(s.ops.fmt_move != NULL);
    ASSERT(s.ops.fmt_button != NULL);
    ASSERT(s.ops.fmt_scroll != NULL);
    ASSERT(s.ops.fmt_transform != NULL);
    ASSERT(s.ops.parse_response != NULL);
    ASSERT(s.ops.reset_parser != NULL);
}

TEST(proto_init_ferrum_ascii) {
    proto_state_t s;
    proto_init(&s, DEV_FERRUM);
    ASSERT_EQ_INT(s.type, PROTO_ASCII);
    ASSERT_STR_EQ(s.profile->short_name, "ferrum");
}

TEST(proto_init_makcu_binary) {
    proto_state_t s;
    proto_init(&s, DEV_MAKCU);
    ASSERT_EQ_INT(s.type, PROTO_MAKCU_BIN);
    ASSERT_STR_EQ(s.profile->short_name, "makcu");
    ASSERT(s.ops.fmt_transform == NULL);  // MAKCU uses bypass, not transform
}

TEST(proto_init_invalid_defaults_to_kmbox) {
    proto_state_t s;
    proto_init(&s, DEV_COUNT);  // Out of range
    ASSERT_EQ_INT(s.type, PROTO_ASCII);
    ASSERT_STR_EQ(s.profile->short_name, "kmbox");
}

TEST(proto_init_zeroes_state) {
    proto_state_t s;
    memset(&s, 0xFF, sizeof(s));  // Fill with garbage
    proto_init(&s, DEV_KMBOX);
    ASSERT_EQ_INT(s.rx_pos, 0);
    ASSERT_EQ_INT(s.parse_state, 0);
    ASSERT_EQ_INT(s.parse_expect, 0);
}

// ============================================================================
// Ferrum == KMBox format compatibility tests
// ============================================================================

TEST(ferrum_same_format_as_kmbox) {
    // Ferrum is a KMBox drop-in; both should produce identical output
    proto_state_t km, fe;
    proto_init(&km, DEV_KMBOX);
    proto_init(&fe, DEV_FERRUM);
    uint8_t buf_km[64], buf_fe[64];

    int len_km = proto_fmt_move(&km, buf_km, sizeof(buf_km), 42, -17);
    int len_fe = proto_fmt_move(&fe, buf_fe, sizeof(buf_fe), 42, -17);
    ASSERT_EQ_INT(len_km, len_fe);
    ASSERT(memcmp(buf_km, buf_fe, len_km) == 0);

    len_km = proto_fmt_button(&km, buf_km, sizeof(buf_km), "left", 1);
    len_fe = proto_fmt_button(&fe, buf_fe, sizeof(buf_fe), "left", 1);
    ASSERT_EQ_INT(len_km, len_fe);
    ASSERT(memcmp(buf_km, buf_fe, len_km) == 0);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    printf("── Protocol Unit Tests ──\n\n");
    printf("Device Profiles:\n");
    run_test_profile_count();
    run_test_profile_kmbox();
    run_test_profile_ferrum();
    run_test_profile_makcu();
    run_test_device_from_name_short();
    run_test_device_from_name_full();
    run_test_device_from_name_case_insensitive();
    run_test_device_from_name_null_and_unknown();

    printf("\nASCII Format:\n");
    run_test_ascii_fmt_move_basic();
    run_test_ascii_fmt_move_zero();
    run_test_ascii_fmt_move_extremes();
    run_test_ascii_fmt_move_buffer_too_small();
    run_test_ascii_fmt_button();
    run_test_ascii_fmt_scroll();
    run_test_ascii_fmt_transform();
    run_test_makcu_no_transform();

    printf("\nASCII Parse:\n");
    run_test_ascii_parse_ok_prompt();
    run_test_ascii_parse_echo_response();
    run_test_ascii_parse_error();
    run_test_ascii_parse_ok_text();
    run_test_ascii_parse_incremental();
    run_test_ascii_parse_multiple_lines();
    run_test_ascii_parse_null_callback();
    run_test_ascii_parse_empty_lines();
    run_test_ascii_reset_clears_state();

    printf("\nMAKCU Binary Format:\n");
    run_test_makcu_bin_fmt_move_falls_back_to_ascii();
    run_test_makcu_bin_fmt_button_left();
    run_test_makcu_bin_fmt_button_right();
    run_test_makcu_bin_fmt_button_middle();
    run_test_makcu_bin_fmt_button_side();
    run_test_makcu_bin_fmt_button_unknown_falls_back_to_ascii();
    run_test_makcu_bin_fmt_scroll_falls_back_to_ascii();

    printf("\nMAKCU Binary Parse:\n");
    run_test_makcu_bin_parse_ok_response();
    run_test_makcu_bin_parse_err_response();
    run_test_makcu_bin_parse_zero_length_payload();
    run_test_makcu_bin_parse_ascii_fallthrough();
    run_test_makcu_bin_parse_ascii_error_fallthrough();
    run_test_makcu_bin_parse_mixed_binary_and_ascii();
    run_test_makcu_bin_parse_incremental();
    run_test_makcu_bin_parse_oversized_resets();
    run_test_makcu_bin_reset_clears_state();

    printf("\nProto Init:\n");
    run_test_proto_init_kmbox_ascii();
    run_test_proto_init_ferrum_ascii();
    run_test_proto_init_makcu_binary();
    run_test_proto_init_invalid_defaults_to_kmbox();
    run_test_proto_init_zeroes_state();

    printf("\nCross-Device:\n");
    run_test_ferrum_same_format_as_kmbox();

    printf("\n── %d passed, %d failed, %d total ──\n",
           g_tests_passed, g_tests_failed, g_tests_run);

    return g_tests_failed > 0 ? 1 : 0;
}
