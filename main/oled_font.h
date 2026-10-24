/* 自动生成，请勿手改 —— 源文件与生成脚本：test-host/gen_font.py */
#pragma once

#include <stdint.h>

/* 字形顺序：0-9 A-Z (SP) . - + : / % ( ) */
#define OLED_FONT_FIRST_DIGIT 0
#define OLED_FONT_FIRST_ALPHA 10
#define OLED_FONT_SPACE       36
#define OLED_FONT_DOT         37
#define OLED_FONT_MINUS       38
#define OLED_FONT_PLUS        39
#define OLED_FONT_COLON       40
#define OLED_FONT_SLASH       41
#define OLED_FONT_PERCENT     42
#define OLED_FONT_LPAREN      43
#define OLED_FONT_RPAREN      44
#define OLED_FONT_GLYPHS      45

extern const uint8_t oled_font5x7[OLED_FONT_GLYPHS][5];
