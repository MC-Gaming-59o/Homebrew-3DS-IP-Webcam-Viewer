// source/main.c
// MJPEG viewer (old2DS) - latest-frame-only + FPS cap to avoid lag.
// Controls: DPAD menu/edit, A/B rotate, Y connect, X flash, SELECT swap bytes, START exit.
// Strategy: always trim to latest full JPEG, but decode only up to FRAME_INTERVAL_MS.

#include <3ds.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

#define READ_CHUNK 4096
#define STREAM_BUF_SIZE (512*1024)
#define CHANNELS 3

// Target maximum decode rate in milliseconds (e.g. 100ms -> ~10 FPS)
#define FRAME_INTERVAL_MS 100

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// helpers
static int find_bytes(const unsigned char *buf, int start, int end, const unsigned char *pat, int patlen) {
    if (patlen <= 0) return -1;
    for (int i = start; i + patlen <= end; ++i) {
        int ok = 1;
        for (int j = 0; j < patlen; ++j) if (buf[i+j] != pat[j]) { ok = 0; break; }
        if (ok) return i;
    }
    return -1;
}
static inline u16 rgb8_to_rgb565_pixel(unsigned char r, unsigned char g, unsigned char b) {
    return (u16)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
static long long now_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
}

// find last full JPEG in buffer and compact to it.
// returns 1 if any full JPEG found and moved to start (buf_start=0, buf_end=frame_len), else 0.
static int compact_to_latest_jpeg_once(unsigned char *stream_buf, int *p_buf_start, int *p_buf_end) {
    int bs = *p_buf_start;
    int be = *p_buf_end;
    if (be - bs <= 0) return 0;

    // last EOI
    int last_eoi = -1;
    for (int i = be - 2; i >= bs; --i) {
        if ((unsigned char)stream_buf[i] == 0xFF && (unsigned char)stream_buf[i+1] == 0xD9) {
            last_eoi = i + 1;
            break;
        }
    }
    if (last_eoi >= 0) {
        int last_soi = -1;
        for (int i = last_eoi - 2; i >= bs; --i) {
            if ((unsigned char)stream_buf[i] == 0xFF && (unsigned char)stream_buf[i+1] == 0xD8) {
                last_soi = i;
                break;
            }
        }
        if (last_soi >= 0) {
            int frame_len = last_eoi + 1 - last_soi;
            memmove(stream_buf, stream_buf + last_soi, frame_len);
            *p_buf_start = 0;
            *p_buf_end = frame_len;
            return 1;
        }
    }
    // fallback: keep small tail to allow later detection
    int tail = 32 * 1024;
    if (be - bs <= tail) return 0;
    int start_pos = be - tail;
    memmove(stream_buf, stream_buf + start_pos, tail);
    *p_buf_start = 0;
    *p_buf_end = tail;
    return 0;
}

static void blit_center_rgb_with_rotation_u16(u16 *fb, int fb_w, int fb_h, int fb_stride,
                                              const unsigned char *rgb, int img_w, int img_h,
                                              int rotation, int swap_bytes)
{
    int rot_w = (rotation % 2 == 0) ? img_w : img_h;
    int rot_h = (rotation % 2 == 0) ? img_h : img_w;

    int copy_w = rot_w < fb_w ? rot_w : fb_w;
    int copy_h = rot_h < fb_h ? rot_h : fb_h;
    int off_x = (fb_w - copy_w) / 2;
    int off_y = (fb_h - copy_h) / 2;
    if (off_x < 0) off_x = 0;
    if (off_y < 0) off_y = 0;

    for (int ry = 0; ry < copy_h; ++ry) {
        int screen_y = off_y + ry;
        u16 *dstrow = fb + screen_y * fb_stride + off_x;
        for (int rx = 0; rx < copy_w; ++rx) {
            int sx = 0, sy = 0;
            switch (rotation) {
                case 0: sx = rx; sy = ry; break;
                case 1: sx = img_w - 1 - ry; sy = rx; break;
                case 2: sx = img_w - 1 - rx; sy = img_h - 1 - ry; break;
                case 3: sx = ry; sy = img_h - 1 - rx; break;
            }
            if (sx < 0) sx = 0; if (sx >= img_w) sx = img_w - 1;
            if (sy < 0) sy = 0; if (sy >= img_h) sy = img_h - 1;
            int sidx = (sy * img_w + sx) * CHANNELS;
            unsigned char r = rgb[sidx + 0];
            unsigned char g = rgb[sidx + 1];
            unsigned char b = rgb[sidx + 2];
            u16 pix = rgb8_to_rgb565_pixel(r,g,b);
            if (swap_bytes) pix = (u16)((pix >> 8) | (pix << 8));
            dstrow[rx] = pix;
        }
    }
}

static void draw_fill_rect(u16 *fb, int fb_w, int fb_h, int fb_stride, int x, int y, int w, int h, u16 color) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x >= fb_w || y >= fb_h) return;
    if (x + w > fb_w) w = fb_w - x;
    if (y + h > fb_h) h = fb_h - y;
    for (int yy = 0; yy < h; ++yy) {
        u16 *row = fb + (y + yy) * fb_stride + x;
        for (int xx = 0; xx < w; ++xx) row[xx] = color;
    }
}

int main(void) {
    gfxInitDefault();
    PrintConsole topScreen, bottomScreen;
    consoleInit(GFX_BOTTOM, &bottomScreen);
    consoleInit(GFX_TOP, &topScreen);
    consoleSelect(&bottomScreen);

    mkdir("/3ds", 0777);
    mkdir("/3ds/ipcamviewer", 0777);

    char host[64] = "192.168.2.198";
    int host_len = (int)strlen(host);
    int host_cursor = 0;
    char port_str[8] = "8080";
    int port_len = (int)strlen(port_str);
    int port_cursor = 0;
    int focus = 0;
    int menu_dirty = 1;

    int connected = 0;
    httpcContext ctx;
    char url[256];

    unsigned char *stream_buf = malloc(STREAM_BUF_SIZE);
    if (!stream_buf) { consoleSelect(&bottomScreen); printf("OOM\n"); goto CLEANUP; }
    int buf_start = 0, buf_end = 0;

    u16 fb_w = 0, fb_h = 0;
    u8 *fb_u8 = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, &fb_w, &fb_h);
    if (!fb_u8) { consoleSelect(&bottomScreen); printf("gfxGetFramebuffer failed\n"); free(stream_buf); goto CLEANUP; }
    u16 *fb = (u16*)fb_u8;
    int fb_stride = (int)fb_w;

    int rotation = 0;
    int flash_state = 0;
    int swap_bytes = 0;

    const unsigned char SOI[2] = {0xFF,0xD8};
    const unsigned char EOI[2] = {0xFF,0xD9};

    httpcInit(0);

    long long last_decode_time = 0;

    menu_dirty = 1;

    while (aptMainLoop()) {
        hidScanInput();
        u32 keys = hidKeysDown();
        if (keys & KEY_START) break;

        if (keys & KEY_SELECT) { swap_bytes = !swap_bytes; menu_dirty = 1; svcSleepThread(160000000); }

        if (keys & KEY_UP) { focus = 0; menu_dirty = 1; svcSleepThread(120000000); }
        if (keys & KEY_DOWN) { focus = 1; menu_dirty = 1; svcSleepThread(120000000); }

        if (keys & KEY_LEFT) {
            if (focus == 0) { if (host_cursor > 0) host_cursor--; else host_cursor = host_len - 1; }
            else { if (port_cursor > 0) port_cursor--; else port_cursor = port_len - 1; }
            menu_dirty = 1; svcSleepThread(100000000);
        }
        if (keys & KEY_RIGHT) {
            if (focus == 0) { if (host_cursor < host_len - 1) host_cursor++; else host_cursor = 0; }
            else { if (port_cursor < port_len - 1) port_cursor++; else port_cursor = 0; }
            menu_dirty = 1; svcSleepThread(100000000);
        }

        if (keys & KEY_A) { rotation = (rotation + 1) & 3; menu_dirty = 1; svcSleepThread(140000000); }
        if (keys & KEY_B) { rotation = (rotation + 3) & 3; menu_dirty = 1; svcSleepThread(140000000); }

        if (keys & KEY_L) {
            if (focus == 0) {
                char c = host[host_cursor];
                if (c == '.') host[host_cursor] = '0';
                else if (c >= '0' && c < '9') host[host_cursor] = c + 1;
                else host[host_cursor] = '.';
            } else {
                char c = port_str[port_cursor];
                if (c >= '0' && c <= '9') port_str[port_cursor] = (char)('0' + ((c - '0' + 1) % 10));
            }
            menu_dirty = 1; svcSleepThread(120000000);
        }
        if (keys & KEY_R) {
            if (focus == 0) {
                char c = host[host_cursor];
                if (c == '.') host[host_cursor] = '9';
                else if (c > '0' && c <= '9') host[host_cursor] = c - 1;
                else host[host_cursor] = '.';
            } else {
                char c = port_str[port_cursor];
                if (c >= '0' && c <= '9') port_str[port_cursor] = (char)('0' + ((c - '0' + 9) % 10));
            }
            menu_dirty = 1; svcSleepThread(120000000);
        }

        if (keys & KEY_Y) {
            snprintf(url, sizeof(url), "http://%s:%s/video", host, port_str);
            consoleSelect(&bottomScreen);
            printf("Connect -> %s\n", url);
            if (connected) httpcCloseContext(&ctx);
            Result r = httpcOpenContext(&ctx, HTTPC_METHOD_GET, url, 1);
            if (r != 0) {
                printf("httpcOpenContext failed: 0x%08x\n", (unsigned)r);
                connected = 0;
            } else {
                httpcAddRequestHeaderField(&ctx, "User-Agent", "ipcam-3ds-minimal");
                r = httpcBeginRequest(&ctx);
                if (r != 0) {
                    printf("httpcBeginRequest failed: 0x%08x\n", (unsigned)r);
                    httpcCloseContext(&ctx);
                    connected = 0;
                } else {
                    u32 st = 0;
                    if (httpcGetResponseStatusCode(&ctx, &st) == 0 && st == 200) {
                        connected = 1;
                        buf_start = buf_end = 0;
                        printf("Connected.\n");
                    } else {
                        printf("HTTP status bad. st=%u\n", (unsigned)st);
                        httpcCloseContext(&ctx);
                        connected = 0;
                    }
                }
            }
            menu_dirty = 1;
            svcSleepThread(200000000);
        }

        if (keys & KEY_X) {
            consoleSelect(&bottomScreen);
            const char *path = flash_state ? "/disabletorch" : "/enabletorch";
            char furl[300];
            snprintf(furl, sizeof(furl), "http://%s:%s%s", host, port_str, path);
            httpcContext tmp;
            Result tr = httpcOpenContext(&tmp, HTTPC_METHOD_GET, furl, 1);
            if (tr == 0) {
                httpcAddRequestHeaderField(&tmp, "User-Agent", "ipcam-3ds-flash");
                tr = httpcBeginRequest(&tmp);
                if (tr == 0) {
                    u32 resp = 0;
                    if (httpcGetResponseStatusCode(&tmp, &resp) == 0) {
                        if (resp >= 200 && resp < 300) flash_state = !flash_state;
                    }
                    unsigned char tmpb[256];
                    for (int i = 0; i < 4; ++i) {
                        Result rcv = httpcReceiveData(&tmp, tmpb, sizeof(tmpb));
                        if (rcv != 0) break;
                    }
                }
                httpcCloseContext(&tmp);
            }
            menu_dirty = 1;
            svcSleepThread(160000000);
        }

        if (menu_dirty) {
            u16 b_w=0,b_h=0;
            u8 *b_fb8 = gfxGetFramebuffer(GFX_BOTTOM, GFX_LEFT, &b_w, &b_h);
            if (b_fb8) {
                u16 *b_fb = (u16*)b_fb8;
                int bstride = (int)b_w;
                u16 bg = rgb8_to_rgb565_pixel(18,18,20);
                u16 panel = rgb8_to_rgb565_pixel(28,28,32);
                u16 box = rgb8_to_rgb565_pixel(40,40,48);
                u16 accent = rgb8_to_rgb565_pixel(80,160,220);
                draw_fill_rect(b_fb, b_w, b_h, bstride, 0, 0, b_w, b_h, bg);
                draw_fill_rect(b_fb, b_w, b_h, bstride, 0, 0, b_w, 18, panel);
                int margin = 8;
                int box_w = b_w - margin * 2;
                draw_fill_rect(b_fb, b_w, b_h, bstride, margin, 24, box_w, 28, box);
                draw_fill_rect(b_fb, b_w, b_h, bstride, margin, 24+36, box_w, 28, box);
                if (focus == 0) draw_fill_rect(b_fb, b_w, b_h, bstride, margin, 24+26, box_w, 2, accent);
                else draw_fill_rect(b_fb, b_w, b_h, bstride, margin, 24+36+26, box_w, 2, accent);
            }
            consoleSelect(&bottomScreen);
            consoleClear();
            printf("IPCam | %s | Rot:%d*90 | Flash:%s | Swap:%s\n",
                   connected ? "Connected" : "Disconnected",
                   rotation,
                   flash_state ? "ON":"OFF",
                   swap_bytes ? "ON":"OFF");
            printf("\nHost: %s\n", host);
            if (focus == 0) {
                int prefix = (int)strlen("Host: ");
                for (int i = 0; i < prefix; ++i) putchar(' ');
                for (int i = 0; i < host_len; ++i) putchar(i == host_cursor ? '^' : ' ');
                putchar('\n');
            }
            printf("Port: %s\n", port_str);
            if (focus == 1) {
                int prefix = (int)strlen("Port: ");
                for (int i = 0; i < prefix; ++i) putchar(' ');
                for (int i = 0; i < port_len; ++i) putchar(i == port_cursor ? '^' : ' ');
                putchar('\n');
            }
            printf("\nDPAD:menu L/R:edit A/B:rot Y:connect X:flash SELECT:swap START:exit\n");
            gfxFlushBuffers(); gfxSwapBuffers(); gspWaitForVBlank();
            menu_dirty = 0;
        }

        if (!connected) { svcSleepThread(20000); continue; }

        // read chunk
        u32 dl_before=0, dl_total=0;
        httpcGetDownloadSizeState(&ctx, &dl_before, &dl_total);
        unsigned char tmpbuf[READ_CHUNK];
        Result rr = httpcReceiveData(&ctx, tmpbuf, READ_CHUNK);
        u32 dl_after=0;
        httpcGetDownloadSizeState(&ctx, &dl_after, &dl_total);
        int got = 0;
        if (dl_after >= dl_before) got = (int)(dl_after - dl_before);
        if (got == 0 && rr == 0) got = READ_CHUNK;

        if (rr != 0 && got == 0) {
            // try single reconnect
            httpcCloseContext(&ctx);
            Result r = httpcOpenContext(&ctx, HTTPC_METHOD_GET, url, 1);
            if (r == 0) {
                httpcAddRequestHeaderField(&ctx, "User-Agent", "ipcam-3ds-minimal");
                r = httpcBeginRequest(&ctx);
                if (r == 0) {
                    u32 st=0;
                    if (httpcGetResponseStatusCode(&ctx, &st) == 0 && st == 200) {
                        connected = 1; buf_start = buf_end = 0;
                    } else connected = 0;
                } else connected = 0;
            } else connected = 0;
            menu_dirty = 1;
            svcSleepThread(200000000);
            continue;
        }

        if (got > 0) {
            int tocopy = got; if (tocopy > READ_CHUNK) tocopy = READ_CHUNK;
            int free_tail = STREAM_BUF_SIZE - buf_end;
            if (tocopy > free_tail) {
                int data_len = buf_end - buf_start;
                if (data_len > 0) memmove(stream_buf, stream_buf + buf_start, data_len);
                buf_start = 0; buf_end = data_len; free_tail = STREAM_BUF_SIZE - buf_end;
                if (tocopy > free_tail) tocopy = free_tail;
            }
            memcpy(stream_buf + buf_end, tmpbuf, tocopy); buf_end += tocopy;

            // keep buffer from growing too big
            if (buf_end - buf_start > (int)(STREAM_BUF_SIZE * 0.75)) {
                compact_to_latest_jpeg_once(stream_buf, &buf_start, &buf_end);
            }

            // find last full JPEG and compact to it (keep only latest)
            int last_eoi = -1;
            for (int i = buf_end - 2; i >= buf_start; --i) {
                if ((unsigned char)stream_buf[i] == 0xFF && (unsigned char)stream_buf[i+1] == 0xD9) { last_eoi = i + 1; break; }
            }
            if (last_eoi >= 0) {
                int last_soi = -1;
                for (int i = last_eoi - 2; i >= buf_start; --i) {
                    if ((unsigned char)stream_buf[i] == 0xFF && (unsigned char)stream_buf[i+1] == 0xD8) { last_soi = i; break; }
                }
                if (last_soi >= 0) {
                    int frame_len = last_eoi + 1 - last_soi;
                    // copy frame out (we will remove everything up to last_eoi+1)
                    unsigned char *frame = malloc(frame_len);
                    if (!frame) {
                        // oom - skip
                        buf_start = 0; buf_end = 0;
                        continue;
                    }
                    memcpy(frame, stream_buf + last_soi, frame_len);
                    int remain = buf_end - (last_eoi + 1);
                    if (remain > 0) memmove(stream_buf, stream_buf + last_eoi + 1, remain);
                    buf_start = 0; buf_end = remain;

                    // only decode if enough time passed since last decode (FPS cap)
                    long long ms = now_ms();
                    if (ms - last_decode_time >= FRAME_INTERVAL_MS) {
                        last_decode_time = ms;
                        int w=0,h=0,n=0;
                        unsigned char *rgb = stbi_load_from_memory(frame, frame_len, &w, &h, &n, CHANNELS);
                        if (rgb) {
                            blit_center_rgb_with_rotation_u16((u16*)fb_u8, (int)fb_w, (int)fb_h, fb_stride, rgb, w, h, rotation, swap_bytes);
                            stbi_image_free(rgb);
                            gfxFlushBuffers(); gfxSwapBuffers(); gspWaitForVBlank();
                            // slight sleep to reduce tight-looping (optional)
                            svcSleepThread(20000000); // 20ms
                        }
                    }
                    free(frame);
                    // continue loop, we already trimmed buffer to latest
                    continue;
                }
            }
        } else {
            svcSleepThread(1000);
            continue;
        }
    } // aptMainLoop

    free(stream_buf);
    if (connected) httpcCloseContext(&ctx);
    httpcExit();

CLEANUP:
    consoleSelect(&bottomScreen);
    printf("Stopped. Press START to return.\n");
    while (aptMainLoop()) {
        hidScanInput();
        if (hidKeysDown() & KEY_START) break;
        gfxFlushBuffers(); gfxSwapBuffers(); gspWaitForVBlank();
    }
    gfxExit();
    return 0;
}
