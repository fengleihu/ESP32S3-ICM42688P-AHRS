/*
 * oled_ui.h — OLED 画面渲染（纯 C，不依赖 ESP-IDF，可在 PC 上渲染验证）
 *
 * 输出格式与 SSD1306 显存一致：128x64，8 页，每页 128 字节，字节的 bit0 是最上面一行。
 *
 * 版面（64 行）：
 *    y  0..13   2 倍字：HDG 航向     右上角 1 倍字：MAG / GYR / CAL / ERR
 *    y 16..23   1 倍字：R+012.3  P-045.6
 *    y 25..32   1 倍字：左状态（MAG LOCK / MAG UNCAL / NO MAG / CAL GYRO …）  右对齐：网络信息
 *    y 34..63   人工地平仪（地平线 + 网点地面 + 中心机翼符号）
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_UI_W        128

/* 屏的有效高度由 menuconfig 的"屏的有效高度"决定：0.96 寸 = 64 行，0.91 寸 = 32 行。
   写错不会损坏屏，但画面会上下重复或只剩一半。 */
#if CONFIG_APP_OLED_HO_32
#define OLED_UI_H        32
#else
#define OLED_UI_H        64
#endif

#define OLED_UI_PAGES    (OLED_UI_H / 8)
#define OLED_UI_FB_SIZE  (OLED_UI_W * OLED_UI_PAGES)

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg; /* 磁北基准航向，任意范围，内部归一化到 0~360 */

    bool ahrs_ok;         /* IMU 正常 */
    bool mag_present;     /* 检测到 BMM350 */
    bool mag_calibrated;  /* 磁力计已校准 */
    bool mag_used;        /* 本帧磁力计是否参与融合（= yaw 被锁定） */
    bool calibrating;     /* 陀螺零偏校准中 */
    bool mag_cal_running; /* 磁力计校准中 */

    const char *net_line; /* 第三行右侧：IP / 热点，可为 NULL */
} oled_ui_state_t;

/** 渲染整帧到 fb（OLED_UI_FB_SIZE 字节） */
void oled_ui_render(uint8_t *fb, const oled_ui_state_t *st);

/**
 * 页内位序：SSD1306 标准是"字节 bit0 = 该页最上面一行"。
 * 少数模块/接线情况下会出现整页上下颠倒（文字变成碎点、地面跑到线上方），
 * 打开这个开关即可纠正，不用改接线。
 */
void oled_ui_set_page_bitflip(bool on);

/** 诊断用测试卡：边框 + 每页首行标记 + 两行字符，用来一眼判断页/位映射和丢位 */
void oled_ui_render_test(uint8_t *fb);

/**
 * 引线/数据通路检查序列（无歧义的 7 步，用于判断"随机散点"到底是接线问题还是映射问题）：
 *   0 = 屏幕正中一个像素（全屏唯一亮点）
 *   1 = 1 像素边框
 *   2 = 十字线（水平 y=32 + 垂直 x=64）
 *   3 = 整屏 50% 棋盘
 *   4 = 左半屏全亮、右半屏全灭
 *   5 = 2 倍字 "OK"
 *   6 = 整屏全亮（四角留黑点）：自检"能不能把所有像素都点亮"
 */
void oled_ui_render_bringup(uint8_t *fb, int phase);

/** 引线检查共几步 */
#define OLED_UI_BRINGUP_PHASES 7

/* ---- 基础绘图（也供测试使用）---- */
void oled_ui_clear(uint8_t *fb);
void oled_ui_px(uint8_t *fb, int x, int y, int on);
void oled_ui_hline(uint8_t *fb, int x, int y, int len, int on);
void oled_ui_text(uint8_t *fb, int x, int y, const char *s, int scale);
void oled_ui_text_right(uint8_t *fb, int x_right, int y, const char *s, int scale);
int  oled_ui_text_width(const char *s, int scale);

/** 任意两点间画线（Bresenham，自动裁剪到屏幕范围内） */
void oled_ui_line(uint8_t *fb, int x0, int y0, int x1, int y1, int on);

/**
 * 姿态立方体：用"机体 -> 世界"的旋转矩阵画一个线框立方体，
 * 表示板子相对世界的姿态；+X（前/照片上沿）方向带一个机头尖角。
 *
 * 视角固定为斜俯视（方位 30°、俯角 20°）：立方体随姿态转动，
 * 平放时看到顶面+前侧面，倾斜/偏航时各面依次显露。
 *
 * @param rot 行主序 3x3，p_world = rot * p_body（即 ahrs_quat_to_matrix）
 */
void oled_ui_render_attitude(uint8_t *fb, const float rot[9]);

#ifdef __cplusplus
}
#endif
