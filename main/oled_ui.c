#include "oled_ui.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "oled_font.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD ((float)M_PI / 180.0f)

#define CHAR_W 5 /* 字形宽（不含 1 像素间距） */

/* 页内位序：标准 bit0 = 该页最上面一行；置位后改成 bit7 = 最上面一行 */
static bool s_bitflip;

void oled_ui_set_page_bitflip(bool on)
{
    s_bitflip = on;
}

/* ------------------------------------------------------------------ */
/* 基础绘图                                                            */
/* ------------------------------------------------------------------ */
void oled_ui_clear(uint8_t *fb)
{
    memset(fb, 0, OLED_UI_FB_SIZE);
}

void oled_ui_px(uint8_t *fb, int x, int y, int on)
{
    if (x < 0 || x >= OLED_UI_W || y < 0 || y >= OLED_UI_H) {
        return;
    }
    uint8_t *p = &fb[(y >> 3) * OLED_UI_W + x];
    uint8_t bit = (uint8_t)(1u << (s_bitflip ? (7 - (y & 7)) : (y & 7)));
    if (on) {
        *p |= bit;
    } else {
        *p &= (uint8_t)~bit;
    }
}

void oled_ui_hline(uint8_t *fb, int x, int y, int len, int on)
{
    for (int i = 0; i < len; i++) {
        oled_ui_px(fb, x + i, y, on);
    }
}

/*
 * 任意方向画线（Bresenham）。oled_ui_px 自身带边界检查，
 * 所以线段超出屏幕的部分会被自然裁掉，不需要单独做裁剪。
 */
void oled_ui_line(uint8_t *fb, int x0, int y0, int x1, int y1, int on)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        oled_ui_px(fb, x0, y0, on);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static int glyph_index(char c)
{
    if (c >= '0' && c <= '9') {
        return OLED_FONT_FIRST_DIGIT + (c - '0');
    }
    if (c >= 'A' && c <= 'Z') {
        return OLED_FONT_FIRST_ALPHA + (c - 'A');
    }
    if (c >= 'a' && c <= 'z') {
        return OLED_FONT_FIRST_ALPHA + (c - 'a'); /* 小写按大写显示 */
    }
    switch (c) {
    case ' ': return OLED_FONT_SPACE;
    case '.': return OLED_FONT_DOT;
    case '-': return OLED_FONT_MINUS;
    case '+': return OLED_FONT_PLUS;
    case ':': return OLED_FONT_COLON;
    case '/': return OLED_FONT_SLASH;
    case '%': return OLED_FONT_PERCENT;
    case '(': return OLED_FONT_LPAREN;
    case ')': return OLED_FONT_RPAREN;
    default: return OLED_FONT_SPACE;
    }
}

int oled_ui_text_width(const char *s, int scale)
{
    int n = (int)strlen(s);
    if (n == 0) {
        return 0;
    }
    return n * (CHAR_W + 1) * scale - scale; /* 去掉最后一个字符的间距 */
}

void oled_ui_text(uint8_t *fb, int x, int y, const char *s, int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    for (const char *p = s; *p; p++) {
        const uint8_t *g = oled_font5x7[glyph_index(*p)];
        for (int col = 0; col < CHAR_W; col++) {
            uint8_t bits = g[col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1u << row)) {
                    if (scale == 1) {
                        oled_ui_px(fb, x + col, y + row, 1);
                    } else {
                        for (int dy = 0; dy < scale; dy++) {
                            for (int dx = 0; dx < scale; dx++) {
                                oled_ui_px(fb, x + col * scale + dx, y + row * scale + dy, 1);
                            }
                        }
                    }
                }
            }
        }
        x += (CHAR_W + 1) * scale;
    }
}

void oled_ui_text_right(uint8_t *fb, int x_right, int y, const char *s, int scale)
{
    oled_ui_text(fb, x_right - oled_ui_text_width(s, scale), y, s, scale);
}

/* ------------------------------------------------------------------ */
/* 姿态立方体（3D 线框）                                               */
/* ------------------------------------------------------------------ */
/*
 * 视角固定：先绕世界 Z 转 -AZIM，再绕世界 X 转 -ELEV，然后正交投影。
 * 这样"屏幕向上"始终是世界 +Z（重力反方向），立方体随姿态转动时
 * 人的直觉不会翻掉；平放时同时能看到顶面和前侧面，斜视感明显。
 */
#define ATT_AZIM_DEG 30.0f
#define ATT_ELEV_DEG 20.0f

static const int8_t ATT_VERT[8][3] = {
    {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
    {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},
};

static const uint8_t ATT_EDGE[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},   /* 底面 */
    {4, 5}, {5, 6}, {6, 7}, {7, 4},   /* 顶面 */
    {0, 4}, {1, 5}, {2, 6}, {3, 7},   /* 立柱 */
};

/* 投影一个世界系点：x 向右，z 向上，y 为深度（这里丢弃） */
static void att_project(float wx, float wy, float wz, int *sx, int *sy)
{
    const float ca = 0.8660254f; /* cos 30 */
    const float sa = 0.5f;       /* sin 30 */
    const float ce = 0.9396926f; /* cos 20 */
    const float se = 0.3420201f; /* sin 20 */

    float x1 = wx * ca + wy * sa;
    float y1 = -wx * sa + wy * ca;
    float z2 = -y1 * se + wz * ce;

#if OLED_UI_H >= 64
    const float scale = 20.0f;
    const int cx = 40;
    const int cy = 32;
#else
    const float scale = 10.0f;
    const int cx = 38;
    const int cy = 16;
#endif

    *sx = cx + (int)lroundf(x1 * scale);
    *sy = cy - (int)lroundf(z2 * scale);
}

void oled_ui_render_attitude(uint8_t *fb, const float rot[9])
{
    int px[8], py[8];

    for (int i = 0; i < 8; i++) {
        float bx = (float)ATT_VERT[i][0];
        float by = (float)ATT_VERT[i][1];
        float bz = (float)ATT_VERT[i][2];

        /* 机体系 -> 世界系 */
        float wx = rot[0] * bx + rot[1] * by + rot[2] * bz;
        float wy = rot[3] * bx + rot[4] * by + rot[5] * bz;
        float wz = rot[6] * bx + rot[7] * by + rot[8] * bz;

        att_project(wx, wy, wz, &px[i], &py[i]);
    }

    for (int e = 0; e < 12; e++) {
        int a = ATT_EDGE[e][0];
        int b = ATT_EDGE[e][1];
        oled_ui_line(fb, px[a], py[a], px[b], py[b], 1);
    }

    /* +X 机头尖角：从 (1,0,0) 沿 +X 伸出一段，指示"前" */
    int nx0, ny0, nx1, ny1;
    float wx = rot[0] * 1.0f;
    float wy = rot[3] * 1.0f;
    float wz = rot[6] * 1.0f;
    att_project(wx, wy, wz, &nx0, &ny0);
    wx = rot[0] * 1.7f;
    wy = rot[3] * 1.7f;
    wz = rot[6] * 1.7f;
    att_project(wx, wy, wz, &nx1, &ny1);
    oled_ui_line(fb, nx0, ny0, nx1, ny1, 1);
}

/* ------------------------------------------------------------------ */
/* 人工地平仪                                                          */
/* ------------------------------------------------------------------ */
#define HZ_X 0
#define HZ_Y 34
#define HZ_W OLED_UI_W
#define HZ_H (OLED_UI_H - HZ_Y) /* 30 行 */
#define HZ_PITCH_FULL_SCALE 40.0f

static void draw_horizon(uint8_t *fb, float roll_deg, float pitch_deg)
{
    const int cx = HZ_X + HZ_W / 2;
    const int cy = HZ_Y + HZ_H / 2;
    const float k = (HZ_H * 0.5f) / HZ_PITCH_FULL_SCALE;

    /* 地平线斜率：roll>0（右侧下沉）时机头右倾，视景里的地平线左低右高，
       所以屏幕上 y 随 x 增大而减小 -> 取 -tan(roll)。
       接近 ±90° 时钳位，避免 tan 爆炸 */
    float tr = -tanf(roll_deg * DEG2RAD);
    if (tr > 8.0f) {
        tr = 8.0f;
    } else if (tr < -8.0f) {
        tr = -8.0f;
    }
    const float y_off = -pitch_deg * k; /* pitch>0(+X 边下沉) -> 地平线上移 */

    for (int x = HZ_X; x < HZ_X + HZ_W; x++) {
        int hy = cy + (int)lroundf(y_off + tr * (float)(x - cx));

        /* 地平线本身也只在窗口内绘制，避免覆盖上方文字 */
        if (hy >= HZ_Y && hy < HZ_Y + HZ_H) {
            oled_ui_px(fb, x, hy, 1);
        }

        /* 地面：50% 网点（省电、对比度也够）。必须裁剪到窗口内！ */
        int y0 = hy + 1;
        if (y0 < HZ_Y) {
            y0 = HZ_Y;
        }
        for (int y = y0; y < HZ_Y + HZ_H; y++) {
            if (((x + y) & 1) == 0) {
                oled_ui_px(fb, x, y, 1);
            }
        }
    }

    /* 固定机翼符号 */
    oled_ui_hline(fb, cx - 18, cy, 12, 1);
    oled_ui_hline(fb, cx + 7, cy, 12, 1);
    oled_ui_px(fb, cx, cy, 1);
    oled_ui_px(fb, cx - 1, cy, 1);
    oled_ui_px(fb, cx + 1, cy, 1);

    /* 顶部中心固定三角（滚转参考） */
    oled_ui_px(fb, cx, HZ_Y, 1);
    oled_ui_px(fb, cx - 1, HZ_Y + 1, 1);
    oled_ui_px(fb, cx + 1, HZ_Y + 1, 1);
    oled_ui_px(fb, cx, HZ_Y + 2, 1);

    /* 左右两个滚转刻度 */
    oled_ui_hline(fb, cx - 32, HZ_Y + 1, 5, 1);
    oled_ui_hline(fb, cx + 28, HZ_Y + 1, 5, 1);
}

/* ------------------------------------------------------------------ */
/* 整帧渲染                                                            */
/* ------------------------------------------------------------------ */
static void fmt_signed(char *buf, size_t n, float v)
{
    /* +012.3 / -045.6，固定宽度便于排版 */
    snprintf(buf, n, "%+06.1f", (double)v);
}

/* ------------------------------------------------------------------ */
/* 诊断测试卡                                                          */
/* ------------------------------------------------------------------ */
void oled_ui_render_test(uint8_t *fb)
{
    oled_ui_clear(fb);

    /* 边框：能看出整帧是否完整、有没有被裁掉/偏移 */
    oled_ui_hline(fb, 0, 0, OLED_UI_W, 1);
    oled_ui_hline(fb, 0, OLED_UI_H - 1, OLED_UI_W, 1);
    for (int y = 0; y < OLED_UI_H; y++) {
        oled_ui_px(fb, 0, y, 1);
        oled_ui_px(fb, OLED_UI_W - 1, y, 1);
    }

    /* 每页的第一行画一小段横线：8 条。若页内位序反了，这 8 条会整体下移 7 像素 */
    for (int page = 0; page < OLED_UI_PAGES; page++) {
        oled_ui_hline(fb, 2, page * 8, 6, 1);
    }

    /* 四角单点：判断整屏翻转 */
    oled_ui_px(fb, 2, 2, 1);
    oled_ui_px(fb, OLED_UI_W - 3, 2, 1);
    oled_ui_px(fb, 2, OLED_UI_H - 3, 1);
    oled_ui_px(fb, OLED_UI_W - 3, OLED_UI_H - 3, 1);

    /* 字符：能读出字 => 映射正确；碎点/镜像 => 位序或丢位 */
    oled_ui_text(fb, 12, 4, "0123456789", 1);
    oled_ui_text(fb, 12, 14, "ABCDEFGHIJ", 1);
    oled_ui_text(fb, 12, 24, "SSD1306 TEST", 1);
    oled_ui_text(fb, 12, 36, "OK 128X64", 2);
}

void oled_ui_render_bringup(uint8_t *fb, int phase)
{
    oled_ui_clear(fb);
    switch (phase % OLED_UI_BRINGUP_PHASES) {
    case 0: /* 全屏只有一个亮点：最容易看清、也最能暴露"乱点" */
        oled_ui_px(fb, OLED_UI_W / 2, OLED_UI_H / 2, 1);
        break;

    case 1: /* 1 像素边框：四条边都完整 = 128x64、列/页映射都对 */
        oled_ui_hline(fb, 0, 0, OLED_UI_W, 1);
        oled_ui_hline(fb, 0, OLED_UI_H - 1, OLED_UI_W, 1);
        for (int y = 0; y < OLED_UI_H; y++) {
            oled_ui_px(fb, 0, y, 1);
            oled_ui_px(fb, OLED_UI_W - 1, y, 1);
        }
        break;

    case 2: /* 十字线：水平线在正中、竖线在正中（128x64 -> y=32，128x32 -> y=16） */
        oled_ui_hline(fb, 0, OLED_UI_H / 2, OLED_UI_W, 1);
        for (int y = 0; y < OLED_UI_H; y++) {
            oled_ui_px(fb, OLED_UI_W / 2, y, 1);
        }
        break;

    case 3: /* 整屏 50% 棋盘：应当是均匀的格子，不能有粗条纹/空洞 */
        for (int y = 0; y < OLED_UI_H; y++) {
            for (int x = 0; x < OLED_UI_W; x++) {
                if (((x + y) & 1) == 0) {
                    oled_ui_px(fb, x, y, 1);
                }
            }
        }
        break;

    case 4: /* 左半屏全亮、右半屏全灭：判断列方向对不对 */
        for (int y = 0; y < OLED_UI_H; y++) {
            for (int x = 0; x < OLED_UI_W / 2; x++) {
                oled_ui_px(fb, x, y, 1);
            }
        }
        break;

    case 5: /* 2 倍字：能读出 OK 说明字符映射也没问题 */
#if OLED_UI_H >= 64
        oled_ui_text(fb, 34, 20, "OK", 2);
#else
        oled_ui_text(fb, 34, 8, "OK", 2);
#endif
        break;

    default: /* 整屏全亮，四角各留 1 个黑点 */
        /*
         * 自检用：如果连"所有像素都点亮"都做不到，说明问题不在数据内容
         * （每帧 1024 字节全是 0xFF，不可能丢位丢成这样），而在**写入通路**：
         * 时钟太快/上升沿太慢/CS 或 DC 时序不对，或者屏本身没被正确寻址。
         * 反过来，如果整屏能均匀点亮，而单点图案却是满屏散点，
         * 那问题就落在"数据在传输中被改变"，方向完全不同。
         */
        memset(fb, 0xFF, OLED_UI_FB_SIZE);
        oled_ui_px(fb, 0, 0, 0);
        oled_ui_px(fb, OLED_UI_W - 1, 0, 0);
        oled_ui_px(fb, 0, OLED_UI_H - 1, 0);
        oled_ui_px(fb, OLED_UI_W - 1, OLED_UI_H - 1, 0);
        break;
    }
}

void oled_ui_render(uint8_t *fb, const oled_ui_state_t *st)
{
    oled_ui_clear(fb);

    /* ---- 第一行（2 倍字）：HDG 航向 ---- */
    float yaw = st->yaw_deg;
    while (yaw < 0.0f) {
        yaw += 360.0f;
    }
    while (yaw >= 360.0f) {
        yaw -= 360.0f;
    }
    char buf[32];
    snprintf(buf, sizeof(buf), "HDG%05.1f", (double)yaw);
    oled_ui_text(fb, 0, 0, buf, 2);

    /* 右上角 1 倍字：航向来源 */
    const char *src;
    if (!st->ahrs_ok) {
        src = "ERR";
    } else if (st->calibrating) {
        src = "CAL";
    } else if (st->mag_cal_running) {
        src = "CAL";
    } else if (st->mag_used) {
        src = "MAG";
    } else {
        src = "GYR";
    }
    oled_ui_text_right(fb, OLED_UI_W - 1, 4, src, 1);

    /* ---- 第二行：ROLL / PITCH ---- */
    char rb[16], pb[16], line[40];
    fmt_signed(rb, sizeof(rb), st->roll_deg);
    fmt_signed(pb, sizeof(pb), st->pitch_deg);
    snprintf(line, sizeof(line), "R%s", rb);
    oled_ui_text(fb, 0, 16, line, 1);
    snprintf(line, sizeof(line), "P%s", pb);
    oled_ui_text(fb, 64, 16, line, 1);

    /* ---- 第三行：左状态 + 右网络 ---- */
    const char *status;
    if (!st->ahrs_ok) {
        status = "NO IMU";
    } else if (st->calibrating) {
        status = "CAL GYRO";
    } else if (st->mag_cal_running) {
        status = "CAL MAG";
    } else if (st->mag_used) {
        status = "MAG LOCK";
    } else if (st->mag_calibrated) {
        status = "GYR ONLY";
    } else if (st->mag_present) {
        status = "MAG UNCAL";
    } else {
        status = "NO MAG";
    }
    oled_ui_text(fb, 0, 25, status, 1);
    if (st->net_line && st->net_line[0]) {
        oled_ui_text_right(fb, OLED_UI_W - 1, 25, st->net_line, 1);
    }

    /* ---- 地平仪 ---- */
    if (st->ahrs_ok) {
        draw_horizon(fb, st->roll_deg, st->pitch_deg);
    } else {
        oled_ui_text(fb, 34, 44, "NO DATA", 1);
    }
}
