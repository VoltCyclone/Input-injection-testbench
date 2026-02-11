/**
 * app_windows.c - Windows Native Application (Win32 + GDI)
 *
 * Full-featured Windows port of the KMBox Trace Analyzer with:
 *   - Win32 window with dark instrumentation aesthetic
 *   - GDI/GDI+ trace rendering with speed coloring
 *   - Windows serial port (COM port) support
 *   - QueryPerformanceCounter high-res timing
 *   - Pan/zoom canvas, sidebar with stats
 *
 * Build (MSYS2/MinGW):
 *   gcc -O2 -o kmbox_tester.exe app_windows.c app_common.c protocols.c \
 *       -lgdi32 -luser32 -lcomctl32 -lcomdlg32 -lm -mwindows
 *
 * Build (MSVC):
 *   cl /O2 app_windows.c app_common.c protocols.c \
 *      /link user32.lib gdi32.lib comctl32.lib comdlg32.lib
 */

#define UNICODE
#define _UNICODE
#define WIN32_LEAN_AND_MEAN
#define _USE_MATH_DEFINES

#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <commdlg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "app_common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Platform Implementation: Timing
// ============================================================================

static LARGE_INTEGER g_perf_freq;
static LARGE_INTEGER g_perf_start;

void plat_time_init(void) {
    QueryPerformanceFrequency(&g_perf_freq);
    QueryPerformanceCounter(&g_perf_start);
}

uint64_t plat_time_us(void) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)((now.QuadPart - g_perf_start.QuadPart) * 1000000ULL / g_perf_freq.QuadPart);
}

// ============================================================================
// Platform Implementation: Serial Port
// ============================================================================

plat_serial_t plat_serial_open(const char* port, int baud) {
    /* Convert to wide string and prepend \\.\\ for COM port names */
    char full_path[256];
    if (strncmp(port, "\\\\.\\", 4) != 0 && strncmp(port, "COM", 3) == 0) {
        snprintf(full_path, sizeof(full_path), "\\\\.\\%s", port);
    } else {
        snprintf(full_path, sizeof(full_path), "%s", port);
    }

    wchar_t wpath[256];
    MultiByteToWideChar(CP_UTF8, 0, full_path, -1, wpath, 256);

    HANDLE h = CreateFileW(wpath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) return PLAT_SERIAL_INVALID;

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) { CloseHandle(h); return PLAT_SERIAL_INVALID; }

    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_ENABLE;
    dcb.fRtsControl  = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;

    if (!SetCommState(h, &dcb)) { CloseHandle(h); return PLAT_SERIAL_INVALID; }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 1;
    timeouts.ReadTotalTimeoutMultiplier  = 0;
    timeouts.ReadTotalTimeoutConstant    = 1;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 100;
    SetCommTimeouts(h, &timeouts);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return h;
}

void plat_serial_close(plat_serial_t s) {
    if (s != PLAT_SERIAL_INVALID) CloseHandle(s);
}

int plat_serial_write(plat_serial_t s, const uint8_t* data, int len) {
    DWORD written = 0;
    if (!WriteFile(s, data, (DWORD)len, &written, NULL)) return -1;
    return (int)written;
}

int plat_serial_read(plat_serial_t s, uint8_t* buf, int buflen) {
    DWORD bytesRead = 0;
    if (!ReadFile(s, buf, (DWORD)buflen, &bytesRead, NULL)) return -1;
    return (int)bytesRead;
}

// ============================================================================
// Platform Implementation: Cursor Position
// ============================================================================

bool plat_get_cursor_pos(plat_cursor_pos_t* pos) {
    POINT pt;
    if (!GetCursorPos(&pt)) return false;
    pos->x = (double)pt.x;
    pos->y = (double)pt.y;
    return true;
}

// ============================================================================
// Color Constants
// ============================================================================

#define RGB_BG0     RGB(6, 6, 10)
#define RGB_BG1     RGB(11, 11, 18)
#define RGB_BG2     RGB(16, 16, 26)
#define RGB_BG3     RGB(22, 22, 34)
#define RGB_BORDER  RGB(30, 30, 48)
#define RGB_TEXT0   RGB(232, 232, 240)
#define RGB_TEXT1   RGB(176, 176, 192)
#define RGB_TEXT2   RGB(112, 112, 136)
#define RGB_TEXT3   RGB(72, 72, 96)
#define RGB_CMD     RGB(78, 154, 240)
#define RGB_OBS     RGB(240, 128, 64)
#define RGB_DEV     RGB(240, 80, 160)
#define RGB_GREEN   RGB(64, 216, 128)
#define RGB_RED     RGB(240, 64, 96)
#define RGB_YELLOW  RGB(232, 200, 64)
#define RGB_CYAN    RGB(64, 200, 232)

// ============================================================================
// Application State
// ============================================================================

static HWND g_hwnd_main;
static HWND g_hwnd_canvas;
static HWND g_hwnd_sidebar;
static HWND g_combo_proto;
static HWND g_combo_test;
static HWND g_edit_port;
static HWND g_btn_connect;
static HWND g_btn_run;
static HWND g_label_status;

static HFONT g_font_mono;
static HFONT g_font_mono_sm;
static HFONT g_font_ui;
static HFONT g_font_big;

static bool g_connected = false;
static bool g_test_running = false;

/* Canvas view state */
static double g_view_x = 0, g_view_y = 0, g_view_scale = 1.0;
static bool g_show_cmd = true, g_show_obs = true, g_show_dots = true;
static bool g_show_dev = false, g_show_grid = true, g_show_speed = true;
static double g_line_width = 1.5;
static bool g_dragging = false;
static POINT g_drag_start;
static double g_drag_vx, g_drag_vy;

/* Control IDs */
#define IDC_PROTO      1001
#define IDC_PORT       1002
#define IDC_CONNECT    1003
#define IDC_TEST       1004
#define IDC_RUN        1005
#define IDC_CANVAS     1010
#define IDC_SIDEBAR    1011
#define IDC_CHK_CMD    1020
#define IDC_CHK_OBS    1021
#define IDC_CHK_DOTS   1022
#define IDC_CHK_DEV    1023
#define IDC_CHK_GRID   1024
#define IDC_CHK_SPEED  1025
#define IDT_REFRESH    2001

// ============================================================================
// Drawing Helpers
// ============================================================================

static void speed_color(double s, double mx, BYTE* r, BYTE* g, BYTE* b) {
    double t = fmin(s / (mx * 0.5 + 0.001), 1.0);
    if (t < 0.33) {
        *r = (BYTE)(70 + t*3*100); *g = (BYTE)(140 + t*3*60); *b = 240;
    } else if (t < 0.66) {
        double u = (t - 0.33) * 3;
        *r = (BYTE)(70 + u*180); *g = (BYTE)(200 - u*20); *b = (BYTE)(240 - u*200);
    } else {
        double u = (t - 0.66) * 3;
        *r = 240; *g = (BYTE)(180 - u*140); *b = (BYTE)(40 - u*20);
    }
}

static void screen_pt(double px, double py, int* sx, int* sy) {
    *sx = (int)(px * g_view_scale + g_view_x);
    *sy = (int)(py * g_view_scale + g_view_y);
}

static void fit_view(int W, int H) {
    double rx = g_trace.all_max_x - g_trace.all_min_x;
    double ry = g_trace.all_max_y - g_trace.all_min_y;
    if (rx < 10) rx = 10; if (ry < 10) ry = 10;
    double pad = 60;
    g_view_scale = fmin((W - pad*2) / rx, (H - pad*2) / ry) * 0.9;
    double cx = g_trace.all_min_x + rx/2, cy = g_trace.all_min_y + ry/2;
    g_view_x = W/2.0 - cx * g_view_scale;
    g_view_y = H/2.0 - cy * g_view_scale;
}

// ============================================================================
// Canvas Paint
// ============================================================================

static void paint_canvas(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT rc;
    GetClientRect(hwnd, &rc);
    int W = rc.right, H = rc.bottom;

    /* Double buffer */
    HDC mem = CreateCompatibleDC(hdc);
    HBITMAP bmp = CreateCompatibleBitmap(hdc, W, H);
    HBITMAP oldBmp = (HBITMAP)SelectObject(mem, bmp);

    /* Background */
    HBRUSH bgBrush = CreateSolidBrush(RGB_BG0);
    FillRect(mem, &rc, bgBrush);
    DeleteObject(bgBrush);

    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;

    /* Grid */
    if (g_show_grid && g_view_scale > 0) {
        double gs = pow(10, floor(log10(100.0 / g_view_scale)));
        HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(22, 22, 32));
        HPEN axisPen = CreatePen(PS_SOLID, 1, RGB(37, 37, 53));
        HPEN oldPen = (HPEN)SelectObject(mem, gridPen);

        double d0x = (0 - g_view_x) / g_view_scale;
        double d1x = (W - g_view_x) / g_view_scale;
        double d0y = (0 - g_view_y) / g_view_scale;
        double d1y = (H - g_view_y) / g_view_scale;

        SelectObject(mem, g_font_mono_sm);
        SetBkMode(mem, TRANSPARENT);
        SetTextColor(mem, RGB(42, 42, 54));

        for (double gx = floor(d0x/gs)*gs; gx <= d1x; gx += gs) {
            int sx = (int)(gx * g_view_scale + g_view_x);
            if (fabs(gx) < 0.01) SelectObject(mem, axisPen);
            else SelectObject(mem, gridPen);
            MoveToEx(mem, sx, 0, NULL); LineTo(mem, sx, H);
            wchar_t lbl[32]; swprintf(lbl, 32, L"%.0f", gx);
            TextOutW(mem, sx+2, 2, lbl, (int)wcslen(lbl));
        }
        for (double gy = floor(d0y/gs)*gs; gy <= d1y; gy += gs) {
            int sy = (int)(gy * g_view_scale + g_view_y);
            if (fabs(gy) < 0.01) SelectObject(mem, axisPen);
            else SelectObject(mem, gridPen);
            MoveToEx(mem, 0, sy, NULL); LineTo(mem, W, sy);
            wchar_t lbl[32]; swprintf(lbl, 32, L"%.0f", gy);
            TextOutW(mem, 2, sy+2, lbl, (int)wcslen(lbl));
        }

        SelectObject(mem, oldPen);
        DeleteObject(gridPen);
        DeleteObject(axisPen);
    }

    /* Origin crosshair */
    {
        int ox, oy;
        screen_pt(0, 0, &ox, &oy);
        HPEN chPen = CreatePen(PS_SOLID, 1, RGB(51, 51, 64));
        HPEN oldPen = (HPEN)SelectObject(mem, chPen);
        MoveToEx(mem, ox-8, oy, NULL); LineTo(mem, ox+8, oy);
        MoveToEx(mem, ox, oy-8, NULL); LineTo(mem, ox, oy+8);
        Arc(mem, ox-6, oy-6, ox+6, oy+6, ox+6, oy, ox+6, oy);
        SelectObject(mem, oldPen);
        DeleteObject(chPen);
    }

    if (ncmd < 2 && nobs < 2) {
        /* Empty state */
        SelectObject(mem, g_font_mono);
        SetTextColor(mem, RGB(72, 72, 96));
        SetBkMode(mem, TRANSPARENT);
        const wchar_t* msg = L"Run a test to see trace data";
        SIZE sz; GetTextExtentPoint32W(mem, msg, (int)wcslen(msg), &sz);
        TextOutW(mem, W/2 - sz.cx/2, H/2 - sz.cy/2, msg, (int)wcslen(msg));
        goto done;
    }

    /* Compute max speeds for coloring */
    double cmdMaxSpd = 0, obsMaxSpd = 0;
    if (g_show_speed) {
        for (uint32_t i = 1; i < ncmd; i++) {
            double dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
            if (dt <= 0) continue;
            double dx = g_trace.cmds[i].cum_x - g_trace.cmds[i-1].cum_x;
            double dy = g_trace.cmds[i].cum_y - g_trace.cmds[i-1].cum_y;
            double s = hypot(dx,dy) / (dt/1000.0);
            if (s > cmdMaxSpd) cmdMaxSpd = s;
        }
        for (uint32_t i = 1; i < nobs; i++) {
            double dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
            if (dt <= 0) continue;
            double dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
            double dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
            double s = hypot(dx,dy) / (dt/1000.0);
            if (s > obsMaxSpd) obsMaxSpd = s;
        }
    }
    double globalMaxSpd = fmax(cmdMaxSpd, obsMaxSpd);

    /* Draw paths */
    for (int pass = 0; pass < 2; pass++) {
        /* pass 0 = observed, pass 1 = commanded */
        bool isCmd = (pass == 1);
        if (isCmd && !g_show_cmd) continue;
        if (!isCmd && !g_show_obs) continue;
        uint32_t n = isCmd ? ncmd : nobs;
        if (n < 2) continue;

        double prevSpd = 0;
        for (uint32_t i = 1; i < n; i++) {
            int x0, y0, x1, y1;
            if (isCmd) {
                screen_pt(g_trace.cmds[i-1].cum_x, g_trace.cmds[i-1].cum_y, &x0, &y0);
                screen_pt(g_trace.cmds[i].cum_x, g_trace.cmds[i].cum_y, &x1, &y1);
            } else {
                screen_pt(g_trace.obs[i-1].rel_x, g_trace.obs[i-1].rel_y, &x0, &y0);
                screen_pt(g_trace.obs[i].rel_x, g_trace.obs[i].rel_y, &x1, &y1);
            }

            COLORREF col;
            if (g_show_speed) {
                double dt, dx, dy;
                if (isCmd) {
                    dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
                    dx = g_trace.cmds[i].cum_x - g_trace.cmds[i-1].cum_x;
                    dy = g_trace.cmds[i].cum_y - g_trace.cmds[i-1].cum_y;
                } else {
                    dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
                    dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
                    dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
                }
                double spd = (dt > 0) ? hypot(dx,dy)/(dt/1000.0) : prevSpd;
                prevSpd = spd;
                BYTE r,g,b; speed_color(spd, globalMaxSpd, &r, &g, &b);
                col = RGB(r, g, b);
            } else {
                col = isCmd ? RGB_CMD : RGB_OBS;
            }

            HPEN pen = CreatePen(PS_SOLID, 2, col);
            HPEN old = (HPEN)SelectObject(mem, pen);
            MoveToEx(mem, x0, y0, NULL);
            LineTo(mem, x1, y1);
            SelectObject(mem, old);
            DeleteObject(pen);
        }

        /* Start/end markers */
        int sx, sy, ex, ey;
        if (isCmd) {
            screen_pt(g_trace.cmds[0].cum_x, g_trace.cmds[0].cum_y, &sx, &sy);
            screen_pt(g_trace.cmds[n-1].cum_x, g_trace.cmds[n-1].cum_y, &ex, &ey);
        } else {
            screen_pt(g_trace.obs[0].rel_x, g_trace.obs[0].rel_y, &sx, &sy);
            screen_pt(g_trace.obs[n-1].rel_x, g_trace.obs[n-1].rel_y, &ex, &ey);
        }
        {
            HBRUSH gb = CreateSolidBrush(RGB_GREEN);
            HBRUSH old = (HBRUSH)SelectObject(mem, gb);
            SelectObject(mem, GetStockObject(NULL_PEN));
            Ellipse(mem, sx-4, sy-4, sx+4, sy+4);
            SelectObject(mem, old);
            DeleteObject(gb);
        }
        {
            HBRUSH rb = CreateSolidBrush(RGB_RED);
            HBRUSH old = (HBRUSH)SelectObject(mem, rb);
            SelectObject(mem, GetStockObject(NULL_PEN));
            Ellipse(mem, ex-4, ey-4, ex+4, ey+4);
            SelectObject(mem, old);
            DeleteObject(rb);
        }
    }

done:
    BitBlt(hdc, 0, 0, W, H, mem, 0, 0, SRCCOPY);
    SelectObject(mem, oldBmp);
    DeleteObject(bmp);
    DeleteDC(mem);
    EndPaint(hwnd, &ps);
}

// ============================================================================
// Sidebar Paint
// ============================================================================

static void paint_sidebar(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT rc;
    GetClientRect(hwnd, &rc);

    HBRUSH bgBrush = CreateSolidBrush(RGB_BG1);
    FillRect(hdc, &rc, bgBrush);
    DeleteObject(bgBrush);

    SetBkMode(hdc, TRANSPARENT);

    int y = 10;
    int w = rc.right - 24;
    trace_analysis_t* a = &g_trace.analysis;

    #define HEADER(text) do { \
        SetTextColor(hdc, RGB(72, 72, 96)); \
        SelectObject(hdc, g_font_mono_sm); \
        TextOutA(hdc, 12, y, text, (int)strlen(text)); \
        y += 18; \
    } while(0)

    #define ROW(label, value, col) do { \
        SetTextColor(hdc, RGB(112, 112, 136)); \
        SelectObject(hdc, g_font_mono_sm); \
        TextOutA(hdc, 12, y, label, (int)strlen(label)); \
        SetTextColor(hdc, col); \
        SelectObject(hdc, g_font_mono); \
        RECT vr = {12 + w/2, y, 12 + w, y + 16}; \
        DrawTextA(hdc, value, -1, &vr, DT_RIGHT | DT_SINGLELINE); \
        y += 17; \
    } while(0)

    if (!g_trace.analysis_valid) {
        HEADER("NO DATA");
        SetTextColor(hdc, RGB(72, 72, 96));
        SelectObject(hdc, g_font_mono_sm);
        TextOutA(hdc, 12, y, "Connect and run a test", 22);
    } else {
        char buf[128];

        HEADER("MOVEMENT");
        snprintf(buf, sizeof(buf), "%u", g_trace.cmd_count); ROW("Commands", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%u", g_trace.obs_count); ROW("Observations", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.1f ms", a->total_ms); ROW("Duration", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.0f Hz", a->cmd_rate); ROW("Cmd rate", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.0f Hz", a->obs_rate); ROW("Obs rate", buf, RGB_TEXT0);
        y += 6;

        HEADER("COMMAND FIDELITY");
        COLORREF devCol = (a->dev_avg < 2) ? RGB_GREEN : (a->dev_avg < 5) ? RGB_YELLOW : RGB_RED;
        snprintf(buf, sizeof(buf), "%.2f px", a->dev_avg); ROW("Avg gap", buf, devCol);
        snprintf(buf, sizeof(buf), "%.2f px", a->dev_max); ROW("Max gap", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.0f%%", a->cmd_rep_pct); ROW("Delta reps", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.3f", a->int_cv); ROW("Interval CV", buf, RGB_TEXT0);
        y += 6;

        HEADER("HUMANIZATION");
        COLORREF hCol = (a->h_score < 25) ? RGB_RED : (a->h_score < 50) ? RGB_YELLOW : RGB_GREEN;
        snprintf(buf, sizeof(buf), "%d/100 (%s)", (int)a->h_score, a->h_grade);
        SetTextColor(hdc, hCol);
        SelectObject(hdc, g_font_big);
        TextOutA(hdc, 12, y, buf, (int)strlen(buf));
        y += 28;

        snprintf(buf, sizeof(buf), "%.3f px", a->perp_scatter); ROW("Perp scatter", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.3f px", a->jit_avg); ROW("Jitter avg", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.1f%%", a->dir_flip_rate); ROW("Dir reversals", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.3f", a->speed_cv); ROW("Speed CV", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.1f%%", a->sub_px_pct); ROW("Sub-pixel", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.4f", a->accel_jerk); ROW("Accel jerk", buf, RGB_TEXT0);
        snprintf(buf, sizeof(buf), "%.4f", a->path_eff); ROW("Path eff", buf, RGB_TEXT0);
    }

    #undef HEADER
    #undef ROW

    EndPaint(hwnd, &ps);
}

// ============================================================================
// Canvas Window Proc
// ============================================================================

static LRESULT CALLBACK CanvasWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_PAINT:
        paint_canvas(hwnd);
        return 0;

    case WM_LBUTTONDOWN:
        g_dragging = true;
        g_drag_start.x = GET_X_LPARAM(lParam);
        g_drag_start.y = GET_Y_LPARAM(lParam);
        g_drag_vx = g_view_x;
        g_drag_vy = g_view_y;
        SetCapture(hwnd);
        return 0;

    case WM_MOUSEMOVE:
        if (g_dragging) {
            g_view_x = g_drag_vx + (GET_X_LPARAM(lParam) - g_drag_start.x);
            g_view_y = g_drag_vy + (GET_Y_LPARAM(lParam) - g_drag_start.y);
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;

    case WM_LBUTTONUP:
        g_dragging = false;
        ReleaseCapture();
        return 0;

    case WM_LBUTTONDBLCLK: {
        RECT rc; GetClientRect(hwnd, &rc);
        fit_view(rc.right, rc.bottom);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }

    case WM_MOUSEWHEEL: {
        int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        POINT pt; pt.x = GET_X_LPARAM(lParam); pt.y = GET_Y_LPARAM(lParam);
        ScreenToClient(hwnd, &pt);
        double factor = (delta > 0) ? 1.15 : (1.0/1.15);
        g_view_x = pt.x - (pt.x - g_view_x) * factor;
        g_view_y = pt.y - (pt.y - g_view_y) * factor;
        g_view_scale *= factor;
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }

    case WM_ERASEBKGND:
        return 1; /* Prevent flicker */
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ============================================================================
// Sidebar Window Proc
// ============================================================================

static LRESULT CALLBACK SidebarWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_PAINT:
        paint_sidebar(hwnd);
        return 0;
    case WM_ERASEBKGND:
        return 1;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ============================================================================
// Test Runner Thread
// ============================================================================

static int g_run_test_idx = -1;

static DWORD WINAPI test_thread_fn(LPVOID arg) {
    (void)arg;
    int idx = g_run_test_idx;
    if (idx >= 0 && idx < NUM_TESTS) {
        all_tests[idx].run();
        trace_analyze();
    }
    PostMessageW(g_hwnd_main, WM_APP + 1, 0, 0);
    return 0;
}

// ============================================================================
// Main Window Proc
// ============================================================================

static void do_connect(void) {
    if (g_connected) {
        serial_reader_stop();
        plat_serial_close(g_serial_fd);
        g_serial_fd = PLAT_SERIAL_INVALID;
        g_connected = false;
        SetWindowTextW(g_btn_connect, L"Connect");
        SetWindowTextW(g_label_status, L"Disconnected");
        return;
    }

    char port[256];
    GetWindowTextA(g_edit_port, port, sizeof(port));
    int baud = g_proto.profile->default_baud;
    g_serial_fd = plat_serial_open(port, baud);
    if (g_serial_fd == PLAT_SERIAL_INVALID) {
        SetWindowTextW(g_label_status, L"Failed to open port");
        return;
    }
    serial_reader_start();
    g_connected = true;
    SetWindowTextW(g_btn_connect, L"Disconnect");

    wchar_t status[256];
    swprintf(status, 256, L"Connected: %hs @ %d", g_proto.profile->name, baud);
    SetWindowTextW(g_label_status, status);
}

static void do_run_test(void) {
    if (g_test_running) return;
    int idx = (int)SendMessageW(g_combo_test, CB_GETCURSEL, 0, 0);
    if (idx < 0 || idx >= NUM_TESTS) return;

    g_test_running = true;
    g_run_test_idx = idx;
    g_stat_ok = 0; g_stat_err = 0; g_stat_sent = 0;
    EnableWindow(g_btn_run, FALSE);
    SetWindowTextW(g_btn_run, L"Running...");

    HANDLE t = CreateThread(NULL, 0, test_thread_fn, NULL, 0, NULL);
    if (t) CloseHandle(t);
}

static LRESULT CALLBACK MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE:
        return 0;

    case WM_SIZE: {
        int W = LOWORD(lParam), H = HIWORD(lParam);
        int topH = 52, sideW = 320;
        if (g_hwnd_canvas)
            MoveWindow(g_hwnd_canvas, 0, topH, W - sideW, H - topH, TRUE);
        if (g_hwnd_sidebar)
            MoveWindow(g_hwnd_sidebar, W - sideW, topH, sideW, H - topH, TRUE);
        return 0;
    }

    case WM_COMMAND: {
        int id = LOWORD(wParam);
        int code = HIWORD(wParam);
        if (id == IDC_CONNECT) do_connect();
        else if (id == IDC_RUN) do_run_test();
        else if (id == IDC_PROTO && code == CBN_SELCHANGE) {
            int idx = (int)SendMessageW(g_combo_proto, CB_GETCURSEL, 0, 0);
            device_type_t type = (idx == 0) ? DEV_KMBOX : (idx == 1) ? DEV_FERRUM : DEV_MAKCU;
            proto_init(&g_proto, type);
        }
        else if (id == IDC_CHK_CMD) g_show_cmd = !g_show_cmd;
        else if (id == IDC_CHK_OBS) g_show_obs = !g_show_obs;
        else if (id == IDC_CHK_DOTS) g_show_dots = !g_show_dots;
        else if (id == IDC_CHK_DEV) g_show_dev = !g_show_dev;
        else if (id == IDC_CHK_GRID) g_show_grid = !g_show_grid;
        else if (id == IDC_CHK_SPEED) g_show_speed = !g_show_speed;
        if (g_hwnd_canvas) InvalidateRect(g_hwnd_canvas, NULL, FALSE);
        return 0;
    }

    case WM_TIMER:
        if (wParam == IDT_REFRESH && g_test_running && g_trace.recording) {
            /* Update bounds for live rendering */
            uint32_t ncmd = g_trace.cmd_count, nobs = g_trace.obs_count;
            double mnx=0,mxx=0,mny=0,mxy=0;
            for (uint32_t i = 0; i < ncmd; i++) {
                if (g_trace.cmds[i].cum_x < mnx) mnx = g_trace.cmds[i].cum_x;
                if (g_trace.cmds[i].cum_x > mxx) mxx = g_trace.cmds[i].cum_x;
                if (g_trace.cmds[i].cum_y < mny) mny = g_trace.cmds[i].cum_y;
                if (g_trace.cmds[i].cum_y > mxy) mxy = g_trace.cmds[i].cum_y;
            }
            for (uint32_t i = 0; i < nobs; i++) {
                if (g_trace.obs[i].rel_x < mnx) mnx = g_trace.obs[i].rel_x;
                if (g_trace.obs[i].rel_x > mxx) mxx = g_trace.obs[i].rel_x;
                if (g_trace.obs[i].rel_y < mny) mny = g_trace.obs[i].rel_y;
                if (g_trace.obs[i].rel_y > mxy) mxy = g_trace.obs[i].rel_y;
            }
            g_trace.all_min_x = mnx; g_trace.all_max_x = mxx;
            g_trace.all_min_y = mny; g_trace.all_max_y = mxy;

            RECT rc; GetClientRect(g_hwnd_canvas, &rc);
            fit_view(rc.right, rc.bottom);
            InvalidateRect(g_hwnd_canvas, NULL, FALSE);

            wchar_t status[256];
            swprintf(status, 256, L"Recording: %u cmd, %u obs - %lld sent",
                     ncmd, nobs, (long long)g_stat_sent);
            SetWindowTextW(g_label_status, status);
        }
        return 0;

    case WM_APP + 1:
        /* Test completed */
        g_test_running = false;
        EnableWindow(g_btn_run, TRUE);
        SetWindowTextW(g_btn_run, L"\x25B6 Run");
        {
            RECT rc; GetClientRect(g_hwnd_canvas, &rc);
            fit_view(rc.right, rc.bottom);
            InvalidateRect(g_hwnd_canvas, NULL, FALSE);
            InvalidateRect(g_hwnd_sidebar, NULL, FALSE);

            wchar_t status[256];
            swprintf(status, 256, L"Done: %hs - %lld sent, %lld ok, %lld err",
                     all_tests[g_run_test_idx].name,
                     (long long)g_stat_sent, (long long)g_stat_ok, (long long)g_stat_err);
            SetWindowTextW(g_label_status, status);
        }
        return 0;

    case WM_CLOSE:
        if (g_connected) {
            serial_reader_stop();
            plat_serial_close(g_serial_fd);
        }
        trace_dealloc();
        DestroyWindow(hwnd);
        return 0;

    case WM_DESTROY:
        KillTimer(hwnd, IDT_REFRESH);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ============================================================================
// WinMain / main
// ============================================================================

static void create_ui(HINSTANCE hInst) {
    /* Fonts */
    g_font_mono = CreateFontW(-13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
    g_font_mono_sm = CreateFontW(-11, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
    g_font_ui = CreateFontW(-12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Segoe UI");
    g_font_big = CreateFontW(-20, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");

    /* Register canvas class */
    WNDCLASSEXW wcCanvas = {sizeof(wcCanvas)};
    wcCanvas.style = CS_DBLCLKS | CS_HREDRAW | CS_VREDRAW;
    wcCanvas.lpfnWndProc = CanvasWndProc;
    wcCanvas.hInstance = hInst;
    wcCanvas.hCursor = LoadCursor(NULL, IDC_CROSS);
    wcCanvas.lpszClassName = L"KMBoxCanvas";
    RegisterClassExW(&wcCanvas);

    /* Register sidebar class */
    WNDCLASSEXW wcSide = {sizeof(wcSide)};
    wcSide.lpfnWndProc = SidebarWndProc;
    wcSide.hInstance = hInst;
    wcSide.hCursor = LoadCursor(NULL, IDC_ARROW);
    wcSide.lpszClassName = L"KMBoxSidebar";
    RegisterClassExW(&wcSide);

    /* Main window */
    int winW = 1400, winH = 900;
    g_hwnd_main = CreateWindowExW(0, L"KMBoxMain", L"KMBox Trace Analyzer",
        WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, winW, winH,
        NULL, NULL, hInst, NULL);

    int topH = 52, sideW = 320;

    /* Top bar controls */
    int x = 12;

    CreateWindowW(L"STATIC", L"Protocol:", WS_CHILD | WS_VISIBLE,
        x, 4, 60, 20, g_hwnd_main, NULL, hInst, NULL);
    g_combo_proto = CreateWindowW(L"COMBOBOX", NULL,
        WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
        x, 24, 110, 120, g_hwnd_main, (HMENU)IDC_PROTO, hInst, NULL);
    SendMessageA(g_combo_proto, CB_ADDSTRING, 0, (LPARAM)"KMBox B+");
    SendMessageA(g_combo_proto, CB_ADDSTRING, 0, (LPARAM)"Ferrum One");
    SendMessageA(g_combo_proto, CB_ADDSTRING, 0, (LPARAM)"MAKCU");
    SendMessageW(g_combo_proto, CB_SETCURSEL, 0, 0);
    x += 120;

    CreateWindowW(L"STATIC", L"Serial Port:", WS_CHILD | WS_VISIBLE,
        x, 4, 80, 20, g_hwnd_main, NULL, hInst, NULL);
    g_edit_port = CreateWindowA("EDIT", "COM3",
        WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
        x, 24, 160, 22, g_hwnd_main, (HMENU)IDC_PORT, hInst, NULL);
    x += 170;

    g_btn_connect = CreateWindowW(L"BUTTON", L"Connect",
        WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
        x, 22, 80, 26, g_hwnd_main, (HMENU)IDC_CONNECT, hInst, NULL);
    x += 90;

    CreateWindowW(L"STATIC", L"Test:", WS_CHILD | WS_VISIBLE,
        x, 4, 40, 20, g_hwnd_main, NULL, hInst, NULL);
    g_combo_test = CreateWindowW(L"COMBOBOX", NULL,
        WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
        x, 24, 150, 400, g_hwnd_main, (HMENU)IDC_TEST, hInst, NULL);
    for (int i = 0; all_tests[i].name; i++)
        SendMessageA(g_combo_test, CB_ADDSTRING, 0, (LPARAM)all_tests[i].name);
    SendMessageW(g_combo_test, CB_SETCURSEL, 0, 0);
    x += 160;

    g_btn_run = CreateWindowW(L"BUTTON", L"\x25B6 Run",
        WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
        x, 22, 80, 26, g_hwnd_main, (HMENU)IDC_RUN, hInst, NULL);
    x += 90;

    g_label_status = CreateWindowW(L"STATIC", L"Disconnected",
        WS_CHILD | WS_VISIBLE | SS_LEFT,
        x, 28, 300, 20, g_hwnd_main, NULL, hInst, NULL);

    /* Display checkboxes */
    int cy = topH + 4;
    /* These go in the sidebar area for simplicity, but we put them at bottom of main */

    /* Canvas */
    g_hwnd_canvas = CreateWindowW(L"KMBoxCanvas", NULL,
        WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN,
        0, topH, winW - sideW, winH - topH,
        g_hwnd_main, (HMENU)IDC_CANVAS, hInst, NULL);

    /* Sidebar */
    g_hwnd_sidebar = CreateWindowW(L"KMBoxSidebar", NULL,
        WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN,
        winW - sideW, topH, sideW, winH - topH,
        g_hwnd_main, (HMENU)IDC_SIDEBAR, hInst, NULL);

    /* Send fonts to controls */
    SendMessageW(g_combo_proto, WM_SETFONT, (WPARAM)g_font_ui, TRUE);
    SendMessageW(g_combo_test, WM_SETFONT, (WPARAM)g_font_ui, TRUE);
    SendMessageW(g_edit_port, WM_SETFONT, (WPARAM)g_font_mono, TRUE);
    SendMessageW(g_btn_connect, WM_SETFONT, (WPARAM)g_font_ui, TRUE);
    SendMessageW(g_btn_run, WM_SETFONT, (WPARAM)g_font_ui, TRUE);
    SendMessageW(g_label_status, WM_SETFONT, (WPARAM)g_font_mono_sm, TRUE);

    SetTimer(g_hwnd_main, IDT_REFRESH, 50, NULL);
    ShowWindow(g_hwnd_main, SW_SHOW);
    UpdateWindow(g_hwnd_main);
}

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE hPrev, LPWSTR cmdLine, int nCmdShow) {
    (void)hPrev; (void)cmdLine; (void)nCmdShow;

    trace_alloc();
    srand((unsigned)GetTickCount());
    proto_init(&g_proto, DEV_KMBOX);

    /* Check for CLI args */
    int argc;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    char cli_port[256] = {0};
    char cli_proto[64] = "kmbox";
    char cli_test[64] = {0};
    int cli_baud = 0;

    for (int i = 1; i < argc; i++) {
        char arg[256];
        WideCharToMultiByte(CP_UTF8, 0, argv[i], -1, arg, 256, NULL, NULL);
        if (!strcmp(arg, "--help") || !strcmp(arg, "-h")) {
            print_usage("kmbox_tester.exe");
            LocalFree(argv);
            trace_dealloc();
            return 0;
        }
        if (!strcmp(arg, "--port") && i+1 < argc) {
            WideCharToMultiByte(CP_UTF8, 0, argv[++i], -1, cli_port, 256, NULL, NULL);
        } else if (!strcmp(arg, "--proto") && i+1 < argc) {
            WideCharToMultiByte(CP_UTF8, 0, argv[++i], -1, cli_proto, 64, NULL, NULL);
        } else if (!strcmp(arg, "--test") && i+1 < argc) {
            WideCharToMultiByte(CP_UTF8, 0, argv[++i], -1, cli_test, 64, NULL, NULL);
        } else if (!strcmp(arg, "--baud") && i+1 < argc) {
            char b[32];
            WideCharToMultiByte(CP_UTF8, 0, argv[++i], -1, b, 32, NULL, NULL);
            cli_baud = atoi(b);
        }
    }
    LocalFree(argv);

    if (cli_test[0] && cli_port[0]) {
        /* CLI mode */
        /* Attach to parent console for output */
        AttachConsole(ATTACH_PARENT_PROCESS);
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);

        proto_init(&g_proto, device_from_name(cli_proto));
        int baud = cli_baud > 0 ? cli_baud : g_proto.profile->default_baud;

        printf("Opening %s (%s @ %d)...\n", cli_port, g_proto.profile->name, baud);
        g_serial_fd = plat_serial_open(cli_port, baud);
        if (g_serial_fd == PLAT_SERIAL_INVALID) {
            fprintf(stderr, "Failed to open port\n");
            trace_dealloc();
            return 1;
        }

        serial_reader_start();

        bool found = false;
        for (int i = 0; all_tests[i].name; i++) {
            if (!strcmp(all_tests[i].name, cli_test)) {
                printf("Running test: %s\n", cli_test);
                all_tests[i].run();
                trace_analyze();
                cli_print_results(cli_test);
                found = true;
                break;
            }
        }
        if (!found) fprintf(stderr, "Unknown test: %s\n", cli_test);

        serial_reader_stop();
        plat_serial_close(g_serial_fd);
        trace_dealloc();
        return found ? 0 : 1;
    }

    /* GUI mode */
    InitCommonControls();

    WNDCLASSEXW wc = {sizeof(wc)};
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = MainWndProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = CreateSolidBrush(RGB_BG0);
    wc.lpszClassName = L"KMBoxMain";
    RegisterClassExW(&wc);

    create_ui(hInst);

    MSG msg;
    while (GetMessageW(&msg, NULL, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }

    DeleteObject(g_font_mono);
    DeleteObject(g_font_mono_sm);
    DeleteObject(g_font_ui);
    DeleteObject(g_font_big);

    return (int)msg.wParam;
}

/* Allow building with gcc -mconsole or without -mwindows for CLI use */
int main(int argc, char* argv[]) {
    return wWinMain(GetModuleHandle(NULL), NULL, GetCommandLineW(), SW_SHOW);
}
