#include <stdint.h>

static uint8_t image[ 1152 ] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 0
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 1
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 2
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 3
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 4
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 5
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 6
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 7
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 8
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 9
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 10
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 11
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 12
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 13
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 14
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 15
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 16
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 17
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 18
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 19
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 20
0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, // row 21
0x01, 0xe0, 0xf9, 0xf0, 0x0f, 0x80, 0xfc, 0x78, 0x3c, 0xfb, 0xfe, 0x00, // row 22
0x01, 0xe1, 0xf0, 0xf0, 0x0f, 0x01, 0xff, 0x78, 0x3f, 0xfb, 0xfe, 0x00, // row 23
0x01, 0xe3, 0xe0, 0xf8, 0x1f, 0x07, 0xff, 0xf8, 0x3f, 0xf3, 0xfe, 0x00, // row 24
0x01, 0xe7, 0xc0, 0x78, 0x1e, 0x07, 0xff, 0xf8, 0x3f, 0xf3, 0xfe, 0x00, // row 25
0x01, 0xef, 0x80, 0x7c, 0x3e, 0x0f, 0x81, 0xf8, 0x3e, 0x00, 0xf0, 0x00, // row 26
0x01, 0xff, 0x80, 0x3c, 0x3c, 0x0f, 0x00, 0xf8, 0x3e, 0x00, 0xf0, 0x00, // row 27
0x01, 0xff, 0x00, 0x3e, 0x3c, 0x1f, 0x00, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 28
0x01, 0xfe, 0x00, 0x1e, 0x78, 0x1e, 0x00, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 29
0x01, 0xfe, 0x00, 0x1e, 0x78, 0x1e, 0x00, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 30
0x01, 0xff, 0x00, 0x0f, 0xf0, 0x1e, 0x00, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 31
0x01, 0xff, 0x80, 0x0f, 0xf0, 0x1e, 0x00, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 32
0x01, 0xef, 0x80, 0x07, 0xe0, 0x1e, 0x00, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 33
0x01, 0xe7, 0xc0, 0x07, 0xe0, 0x1f, 0x00, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 34
0x01, 0xe3, 0xe0, 0x03, 0xc0, 0x0f, 0x81, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 35
0x01, 0xe1, 0xf0, 0x03, 0xc0, 0x0f, 0xc3, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 36
0x01, 0xe1, 0xf8, 0x03, 0xc0, 0x07, 0xff, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 37
0x01, 0xe0, 0xf8, 0x01, 0x80, 0x03, 0xff, 0xf8, 0x3c, 0x00, 0xf0, 0x00, // row 38
0x01, 0xe0, 0x7c, 0x01, 0x80, 0x01, 0xfe, 0x78, 0x3c, 0x00, 0xf0, 0x00, // row 39
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 40
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 41
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 42
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 43
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 44
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 45
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 46
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 47
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 48
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 49
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 50
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 51
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 52
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 53
0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 54
0x00, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 55
0x00, 0x00, 0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 56
0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 57
0x00, 0x70, 0x00, 0x0f, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 58
0x00, 0xf8, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 59
0x00, 0xf8, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 60
0x00, 0xf8, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 61
0x00, 0x70, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 62
0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 63
0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 64
0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 65
0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 66
0x00, 0x78, 0x00, 0x1f, 0xf8, 0x0f, 0xc0, 0x1e, 0x7c, 0x3f, 0x00, 0x00, // row 67
0x00, 0x78, 0x00, 0x1f, 0xf8, 0x3f, 0xf0, 0x1f, 0xfe, 0x7f, 0x80, 0x00, // row 68
0x00, 0x78, 0x00, 0x1f, 0xf8, 0x7f, 0xf8, 0x1f, 0xff, 0xff, 0xc0, 0x00, // row 69
0x00, 0x78, 0x00, 0x1f, 0xf8, 0xff, 0xfc, 0x1f, 0xff, 0xff, 0xc0, 0x00, // row 70
0x00, 0x78, 0x00, 0x0f, 0x00, 0xf8, 0x3c, 0x1f, 0x0f, 0xc3, 0xe0, 0x00, // row 71
0x00, 0x78, 0x00, 0x0f, 0x00, 0xf0, 0x3c, 0x1f, 0x07, 0x81, 0xe0, 0x00, // row 72
0x00, 0x78, 0x00, 0x0f, 0x01, 0xe0, 0x1e, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 73
0x00, 0x78, 0x00, 0x0f, 0x01, 0xff, 0xfe, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 74
0x00, 0x78, 0x00, 0x0f, 0x01, 0xff, 0xfe, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 75
0x00, 0x78, 0x00, 0x0f, 0x01, 0xff, 0xfe, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 76
0x00, 0x78, 0x00, 0x0f, 0x01, 0xff, 0xfe, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 77
0x00, 0x78, 0x00, 0x0f, 0x01, 0xe0, 0x00, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 78
0x00, 0x78, 0x00, 0x0f, 0x01, 0xf0, 0x18, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 79
0x00, 0x78, 0x00, 0x0f, 0x00, 0xf0, 0x1e, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 80
0x00, 0x78, 0x00, 0x0f, 0x00, 0xfc, 0x7e, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 81
0x00, 0x78, 0x00, 0x0f, 0x00, 0x7f, 0xfc, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 82
0x00, 0x78, 0x00, 0x0f, 0x00, 0x3f, 0xf8, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 83
0x00, 0x78, 0x00, 0x0f, 0x00, 0x1f, 0xe0, 0x1e, 0x07, 0x81, 0xe0, 0x00, // row 84
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 85
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 86
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 87
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 88
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 89
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 90
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 91
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 92
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 93
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // row 94
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // row 95
};
