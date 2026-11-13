/* cam_debug - live on-screen debug viewer for the marker detector.
 * Owner: Tiffany. Debug/measurement aid only; NOT part of cam_proc or the
 * deployed product (built with `make camdebug`).
 *
 * It shows the live webcam picture and overlays what the team's detector
 * (posture_detect.c) decides. All color logic comes from that file, which is
 * #included below so its static helpers and its pixel mask are used in
 * place; this tool adds no color or detection logic of its own.
 *
 *   red tint     pixels the detector marks as EAR   (green sticker range)
 *   yellow tint  pixels the detector marks as C7    (purple sticker range)
 *   white box    a marker blob that survived the detector's size/spread checks
 *   white line   ear-to-C7 line, drawn when the detector reports a valid CVA
 *
 * The window title and stdout (once a second) show, per color, the RAW count
 * of matching pixels (before the detector's min_pixels check) and the count
 * kept for the blob. A raw count of 0 means the color range misses the
 * sticker; a small non-zero count means the sticker is too small/far.
 *
 * Click a pixel: prints its H/S/V (5x5 average, via the detector's own
 * conversion) and whether each color range accepts it. Use these numbers to
 * pick ranges; the choice itself is a team decision (CLAUDE.md section 8).
 *
 * Keys: t = toggle tint, q or Esc = quit.
 *
 * Camera privacy: frames stay in RAM and go only to the screen. There is no
 * save key and no file output. Do not run alongside cam_proc (one camera,
 * one user). Needs a desktop session and libsdl2-dev.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SDL.h>

#include "posture_detect.c" /* team-written detector, used as-is */
#include "v4l2_cap.h"

#define DEBUG_DEV    "/dev/video0"
#define CAM_W        320
#define CAM_H        240
#define CAM_FPS      5
#define WIN_SCALE    2
#define POLL_MS      200
#define SAMPLE_R     2 /* click sampling: (2*R+1)^2 pixel average */

#define TINT_EAR     0xC0FF0000u /* ARGB: red    */
#define TINT_C7      0xC0FFFF00u /* ARGB: yellow */

static uint8_t g_frame[MAX_W * MAX_H * 2];
static uint32_t g_overlay[MAX_W * MAX_H];

struct grab {
    int got;
    size_t len;
};

static void on_frame(const struct cam_frame *f, void *ctx)
{
    struct grab *g = ctx;

    g->got = 0;
    if (f->bytesused > sizeof g_frame)
        return;
    memcpy(g_frame, f->data, f->bytesused);
    g->len = f->bytesused;
    g->got = 1;
}

/* Fills g_overlay from the detector's mask; returns raw per-color counts. */
static void build_overlay(int w, int h, int show_tint, long *ear_px, long *c7_px)
{
    *ear_px = 0;
    *c7_px = 0;
    for (int i = 0; i < w * h; i++) {
        uint8_t m = g_mask[i];

        if (m & 1)
            (*ear_px)++;
        if (m & 2)
            (*c7_px)++;
        g_overlay[i] = !show_tint ? 0u : (m & 1) ? TINT_EAR : (m & 2) ? TINT_C7 : 0u;
    }
}

static void sample_pixel(const posture_cfg_t *cfg, int px, int py, int w, int h)
{
    long sy = 0, su = 0, sv = 0, n = 0;
    float hh, ss, vv;

    for (int y = py - SAMPLE_R; y <= py + SAMPLE_R; y++) {
        for (int x = px - SAMPLE_R; x <= px + SAMPLE_R; x++) {
            const uint8_t *p;

            if (x < 0 || y < 0 || x >= w || y >= h)
                continue;
            p = &g_frame[((size_t)y * (size_t)w + (size_t)(x & ~1)) * 2];
            sy += (x & 1) ? p[2] : p[0];
            su += p[1];
            sv += p[3];
            n++;
        }
    }
    if (n == 0)
        return;
    yuv_to_hsv((int)(sy / n), (int)(su / n), (int)(sv / n), &hh, &ss, &vv);
    printf("click (%d,%d): H=%.0f S=%.2f V=%.2f | detector mask: %s%s | "
           "ear range %s, c7 range %s\n",
           px, py, (double)hh, (double)ss, (double)vv,
           (g_mask[py * w + px] & 1) ? "EAR " : "", (g_mask[py * w + px] & 2) ? "C7" : "",
           in_range(&cfg->ear, hh, ss, vv) ? "ACCEPTS" : "rejects",
           in_range(&cfg->c7, hh, ss, vv) ? "ACCEPTS" : "rejects");
    fflush(stdout);
}

static void draw_blob(SDL_Renderer *r, const blob_t *b)
{
    float half = 2.f * b->sigma;
    SDL_Rect box;

    if (half < 4.f)
        half = 4.f;
    box.x = (int)((b->x - half) * WIN_SCALE);
    box.y = (int)((b->y - half) * WIN_SCALE);
    box.w = (int)(2.f * half * WIN_SCALE);
    box.h = (int)(2.f * half * WIN_SCALE);
    SDL_RenderDrawRect(r, &box);
    SDL_RenderDrawLine(r, (int)(b->x * WIN_SCALE) - 6, (int)(b->y * WIN_SCALE),
                       (int)(b->x * WIN_SCALE) + 6, (int)(b->y * WIN_SCALE));
    SDL_RenderDrawLine(r, (int)(b->x * WIN_SCALE), (int)(b->y * WIN_SCALE) - 6,
                       (int)(b->x * WIN_SCALE), (int)(b->y * WIN_SCALE) + 6);
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--dev PATH] [tuning overrides]\n"
            "  default device %s\n"
            "Tuning overrides try values live; they change only this viewer's copy of the\n"
            "detector config, never posture_detect.c. Defaults come from posture_cfg_default().\n"
            "  --c7-hue LO,HI   --c7-smin F   --c7-vmin F     C7 (purple) range; hue 0-360, S/V 0-1\n"
            "  --ear-hue LO,HI  --ear-smin F  --ear-vmin F    EAR (green) range\n"
            "  --max-sigma F    --min-pixels N                blob spread limit (px), min pixel count\n",
            argv0, DEBUG_DEV);
}

/* Parses a finite float in [lo, hi]; returns 0 on success. */
static int parse_float(const char *s, float lo, float hi, float *out)
{
    char *end = NULL;
    float v;

    errno = 0;
    v = strtof(s, &end);
    if (errno != 0 || end == s || *end != '\0' || !(v >= lo && v <= hi))
        return -1;
    *out = v;
    return 0;
}

/* Parses "LO,HI" hue degrees, each in [0, 360]; returns 0 on success. */
static int parse_hue(const char *s, float *lo, float *hi)
{
    char *comma = strchr(s, ',');
    char buf[32];
    size_t n;

    if (!comma)
        return -1;
    n = (size_t)(comma - s);
    if (n == 0 || n >= sizeof buf)
        return -1;
    memcpy(buf, s, n);
    buf[n] = '\0';
    if (parse_float(buf, 0.f, 360.f, lo) != 0)
        return -1;
    return parse_float(comma + 1, 0.f, 360.f, hi);
}

int main(int argc, char **argv)
{
    const char *dev = DEBUG_DEV;
    struct cam_dev cam;
    struct grab grab = { 0, 0 };
    posture_cfg_t cfg;
    posture_obs_t obs;
    SDL_Window *win = NULL;
    SDL_Renderer *ren = NULL;
    SDL_Texture *video = NULL, *overlay = NULL;
    int w, h, show_tint = 1, running = 1, have_obs = 0, rc = 1;
    long ear_px = 0, c7_px = 0;
    Uint32 next_report = 0;

    cam_init(&cam);
    posture_cfg_default(&cfg);
    setvbuf(stdout, NULL, _IOLBF, 0);

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : NULL;
        float f = 0.f;
        int bad = 1;

        if (!v) {
            /* every option takes a value */
        } else if (strcmp(a, "--dev") == 0) {
            dev = v;
            bad = 0;
        } else if (strcmp(a, "--c7-hue") == 0) {
            bad = parse_hue(v, &cfg.c7.h_lo, &cfg.c7.h_hi);
        } else if (strcmp(a, "--ear-hue") == 0) {
            bad = parse_hue(v, &cfg.ear.h_lo, &cfg.ear.h_hi);
        } else if (strcmp(a, "--c7-smin") == 0) {
            bad = parse_float(v, 0.f, 1.f, &cfg.c7.s_min);
        } else if (strcmp(a, "--c7-vmin") == 0) {
            bad = parse_float(v, 0.f, 1.f, &cfg.c7.v_min);
        } else if (strcmp(a, "--ear-smin") == 0) {
            bad = parse_float(v, 0.f, 1.f, &cfg.ear.s_min);
        } else if (strcmp(a, "--ear-vmin") == 0) {
            bad = parse_float(v, 0.f, 1.f, &cfg.ear.v_min);
        } else if (strcmp(a, "--max-sigma") == 0) {
            bad = parse_float(v, 1.f, 1000.f, &cfg.max_sigma_px);
        } else if (strcmp(a, "--min-pixels") == 0) {
            bad = parse_float(v, 1.f, 100000.f, &f);
            if (!bad)
                cfg.min_pixels = (int)f;
        }
        if (bad) {
            usage(argv[0]);
            return 2;
        }
        i++; /* consumed the value */
    }

    if (cam_open(&cam, dev, CAM_W, CAM_H, CAM_FPS) != CAM_OK) {
        fprintf(stderr, "cam_debug: cannot open %s (op=%s errno=%d). "
                        "Is cam_proc or another viewer using it?\n",
                dev, cam.last_op ? cam.last_op : "?", cam.last_errno);
        return 1;
    }
    w = (int)cam.width;
    h = (int)cam.height;
    if (w <= 0 || h <= 0 || w > MAX_W || h > MAX_H || (w & 1) ||
        cam.bytesperline != (uint32_t)w * 2u) {
        fprintf(stderr, "cam_debug: unsupported geometry %dx%d bytesperline=%u "
                        "(need tightly packed YUYV, as the detector assumes)\n",
                w, h, (unsigned)cam.bytesperline);
        goto cleanup;
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "cam_debug: SDL_Init: %s (need a desktop session)\n", SDL_GetError());
        goto cleanup;
    }
    win = SDL_CreateWindow("cam_debug", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                           w * WIN_SCALE, h * WIN_SCALE, 0);
    if (!win)
        goto sdl_fail;
    ren = SDL_CreateRenderer(win, -1, 0);
    if (!ren)
        goto sdl_fail;
    video = SDL_CreateTexture(ren, SDL_PIXELFORMAT_YUY2, SDL_TEXTUREACCESS_STREAMING, w, h);
    overlay = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, w, h);
    if (!video || !overlay)
        goto sdl_fail;
    SDL_SetTextureBlendMode(overlay, SDL_BLENDMODE_BLEND);

    printf("cam_debug: %s %dx%d, tpf=%u/%u | detector limits: min_pixels=%d "
           "max_sigma_px=%.1f min_sep_px=%.1f\n"
           "  red tint = detector says EAR (green range), yellow tint = C7 (purple range)\n"
           "  click a pixel for its H/S/V; t = toggle tint; q/Esc = quit\n",
           dev, w, h, (unsigned)cam.tpf_num, (unsigned)cam.tpf_den, cfg.min_pixels,
           (double)cfg.max_sigma_px, (double)cfg.min_sep_px);
    /* The ranges in use are printed so a recorded run documents its own settings. */
    printf("  ranges in use: EAR hue %.0f-%.0f S>=%.2f V>=%.2f | C7 hue %.0f-%.0f S>=%.2f V>=%.2f\n",
           (double)cfg.ear.h_lo, (double)cfg.ear.h_hi, (double)cfg.ear.s_min,
           (double)cfg.ear.v_min, (double)cfg.c7.h_lo, (double)cfg.c7.h_hi,
           (double)cfg.c7.s_min, (double)cfg.c7.v_min);

    while (running) {
        SDL_Event e;
        enum cam_result cr = cam_next_frame(&cam, POLL_MS, on_frame, &grab);

        if (cr == CAM_GONE || cr == CAM_FATAL) {
            fprintf(stderr, "cam_debug: camera lost (op=%s errno=%d)\n",
                    cam.last_op ? cam.last_op : "?", cam.last_errno);
            break;
        }
        if (cr == CAM_OK && grab.got && grab.len >= (size_t)w * (size_t)h * 2u) {
            if (posture_detect(g_frame, w, h, &cfg, &obs) == 0)
                have_obs = 1;
            build_overlay(w, h, show_tint, &ear_px, &c7_px);
            SDL_UpdateTexture(video, NULL, g_frame, w * 2);
            SDL_UpdateTexture(overlay, NULL, g_overlay, w * 4);
            SDL_RenderClear(ren);
            SDL_RenderCopy(ren, video, NULL, NULL);
            SDL_RenderCopy(ren, overlay, NULL, NULL);
            if (have_obs) {
                SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
                if (obs.ear.found)
                    draw_blob(ren, &obs.ear);
                if (obs.c7.found)
                    draw_blob(ren, &obs.c7);
                if (obs.status == POSTURE_OK)
                    SDL_RenderDrawLine(ren, (int)(obs.ear.x * WIN_SCALE),
                                       (int)(obs.ear.y * WIN_SCALE),
                                       (int)(obs.c7.x * WIN_SCALE),
                                       (int)(obs.c7.y * WIN_SCALE));
            }
            SDL_RenderPresent(ren);
        }

        if (have_obs && SDL_GetTicks() >= next_report) {
            char title[320];

            /* found=0 with a large sigma means the blob was too spread out
             * (limit cfg.max_sigma_px); at=(x,y) is the centroid. All values
             * are the detector's own outputs. */
            snprintf(title, sizeof title,
                     "EAR raw=%ld kept=%d found=%d sig=%.1f at=(%.0f,%.0f) | "
                     "C7 raw=%ld kept=%d found=%d sig=%.1f at=(%.0f,%.0f) | %s CVA=%.1f",
                     ear_px, obs.ear.npix, obs.ear.found, (double)obs.ear.sigma,
                     (double)obs.ear.x, (double)obs.ear.y,
                     c7_px, obs.c7.npix, obs.c7.found, (double)obs.c7.sigma,
                     (double)obs.c7.x, (double)obs.c7.y,
                     obs.status == POSTURE_OK ? "OK" : "NO_USER",
                     obs.status == POSTURE_OK ? (double)obs.cva_deg : -1.0);
            SDL_SetWindowTitle(win, title);
            printf("%s\n", title);
            next_report = SDL_GetTicks() + 1000;
        }

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = 0;
            } else if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_q || e.key.keysym.sym == SDLK_ESCAPE)
                    running = 0;
                else if (e.key.keysym.sym == SDLK_t)
                    show_tint = !show_tint;
            } else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                int px = e.button.x / WIN_SCALE, py = e.button.y / WIN_SCALE;

                if (have_obs && px >= 0 && py >= 0 && px < w && py < h)
                    sample_pixel(&cfg, px, py, w, h);
            }
        }
    }
    rc = 0;
    goto cleanup;

sdl_fail:
    fprintf(stderr, "cam_debug: SDL error: %s\n", SDL_GetError());
cleanup:
    if (overlay)
        SDL_DestroyTexture(overlay);
    if (video)
        SDL_DestroyTexture(video);
    if (ren)
        SDL_DestroyRenderer(ren);
    if (win)
        SDL_DestroyWindow(win);
    if (SDL_WasInit(SDL_INIT_VIDEO))
        SDL_Quit();
    cam_close(&cam);
    return rc;
}
