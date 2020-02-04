#ifndef BITMAPFONT_H
#define BITMAPFONT_H

GLubyte biMapFont[][16] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x20: sp */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x08, 0x14, 0x08, 0x00, 0x00, 0x08,	/* 0x21: !  */
    0x08, 0x1c, 0x1c, 0x1c, 0x1c, 0x08, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x22: "  */
    0x00, 0x00, 0x00, 0x66, 0x11, 0x22, 0x55, 0x22 },
  { 0x00, 0x00, 0x02, 0x12, 0x12, 0x7f, 0x12, 0x12,	/* 0x23: #  */
    0x12, 0x7f, 0x12, 0x12, 0x10, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x08, 0x3e, 0x49, 0x49, 0x0a, 0x1c,	/* 0x24: $  */
    0x28, 0x49, 0x49, 0x3e, 0x08, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x06, 0x49, 0x29, 0x16, 0x08,	/* 0x25: %  */
    0x08, 0x34, 0x4a, 0x49, 0x30, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x39, 0x46, 0x42, 0x45, 0x29, 0x10,	/* 0x26: &  */
    0x08, 0x14, 0x22, 0x22, 0x1c, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x27: '  */
    0x00, 0x00, 0x00, 0x18, 0x04, 0x08, 0x14, 0x08 },
  { 0x00, 0x00, 0x00, 0x08, 0x10, 0x20, 0x20, 0x20,	/* 0x28: (  */
    0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x10, 0x08, 0x04, 0x04, 0x04,	/* 0x29: )  */
    0x04, 0x04, 0x04, 0x08, 0x10, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x49, 0x49, 0x3e, 0x08,	/* 0x2a: *  */
    0x3e, 0x49, 0x49, 0x08, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x7f,	/* 0x2b: +  */
    0x08, 0x08, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x10, 0x08, 0x18, 0x18, 0x00, 0x00, 0x00,	/* 0x2c: ,  */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e,	/* 0x2d: -  */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00,	/* 0x2e: .  */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10,	/* 0x2f: /  */
    0x08, 0x08, 0x04, 0x04, 0x02, 0x02, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x24, 0x24, 0x24,	/* 0x30: 0  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x31: 1  */
    0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x3C, 0x20, 0x10, 0x08, 0x04,	/* 0x32: 2  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x04, 0x08, 0x04,	/* 0x33: 3  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x3C, 0x28, 0x18,	/* 0x34: 4  */
    0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x04, 0x38, 0x20,	/* 0x35: 5  */
    0x20, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x24, 0x38, 0x20,	/* 0x36: 6  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x10, 0x10, 0x08, 0x08, 0x04,	/* 0x37: 7  */
    0x04, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x24, 0x18, 0x24,	/* 0x38: 8  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x10, 0x08, 0x04, 0x1C, 0x24,	/* 0x39: 9  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00,	/* 0x3a: :  */
    0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x18, 0x04, 0x08, 0x14, 0x08, 0x00, 0x00,	/* 0x3b: ;  */
    0x00, 0x08, 0x14, 0x08, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x02, 0x04, 0x08, 0x10, 0x20,	/* 0x3c: <  */
    0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00,	/* 0x3d: =  */
    0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x02,	/* 0x3e: >  */
    0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x08, 0x14, 0x08, 0x00, 0x08, 0x08,	/* 0x3f: ?  */
    0x0c, 0x02, 0x41, 0x41, 0x22, 0x1c, 0x00, 0x00 },
  { 0x00, 0x00, 0x1e, 0x20, 0x40, 0x4a, 0x55, 0x55,	/* 0x40: @  */
    0x55, 0x55, 0x49, 0x41, 0x22, 0x1c, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x41, 0x41, 0x41, 0x7F, 0x22,	/* 0x41: A  */
    0x22, 0x22, 0x14, 0x1C, 0x08, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x7C, 0x22, 0x22, 0x22, 0x24,	/* 0x42: B  */
    0x38, 0x24, 0x22, 0x22, 0x7C, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x42, 0x40, 0x40,	/* 0x43: C  */
    0x40, 0x40, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x78, 0x24, 0x22, 0x22, 0x22,	/* 0x44: D  */
    0x22, 0x22, 0x22, 0x24, 0x78, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x7E, 0x20, 0x20, 0x20, 0x20,	/* 0x45: E  */
    0x3C, 0x20, 0x20, 0x20, 0x7E, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20,	/* 0x46: F  */
    0x38, 0x20, 0x20, 0x20, 0x7E, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x3A, 0x46, 0x42, 0x42, 0x46,	/* 0x47: G  */
    0x40, 0x40, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x42, 0x42, 0x42, 0x42, 0x42,	/* 0x48: H  */
    0x7E, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x1C, 0x08, 0x08, 0x08, 0x08,	/* 0x49: I  */
    0x08, 0x08, 0x08, 0x08, 0x1C, 0x00, 0x00, 0x00 },
  { 0x00, 0x20, 0x10, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x4a: J  */
    0x08, 0x08, 0x08, 0x00, 0x1C, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x72, 0x24, 0x24, 0x28, 0x30,	/* 0x4b: K  */
    0x30, 0x28, 0x24, 0x24, 0x72, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x7E, 0x20, 0x20, 0x20, 0x20,	/* 0x4c: L  */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0xC1, 0x41, 0x41, 0x49, 0x49,	/* 0x4d: M  */
    0x55, 0x55, 0x63, 0x63, 0xC1, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x22, 0x22, 0x26, 0x26, 0x2A,	/* 0x4e: N  */
    0x2A, 0x2A, 0x32, 0x32, 0x62, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42,	/* 0x4f: O  */
    0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20,	/* 0x50: P  */
    0x3C, 0x22, 0x22, 0x22, 0x7C, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x01, 0x1A, 0x24, 0x5A, 0x42, 0x42,	/* 0x51: Q  */
    0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x46, 0x44, 0x48, 0x48, 0x50,	/* 0x52: R  */
    0x7C, 0x42, 0x42, 0x42, 0x7C, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x5C, 0x62, 0x42, 0x02, 0x0C,	/* 0x53: S  */
    0x30, 0x40, 0x42, 0x26, 0x1A, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x54: T  */
    0x08, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42,	/* 0x55: U  */
    0x42, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x14, 0x14, 0x22, 0x22,	/* 0x56: V  */
    0x22, 0x41, 0x41, 0x41, 0x41, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0x34, 0x3A,	/* 0x57: W  */
    0x3A, 0x4A, 0x4A, 0x91, 0x91, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x81, 0x42, 0x24, 0x24, 0x18,	/* 0x58: X  */
    0x18, 0x24, 0x24, 0x42, 0x81, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x1C, 0x08, 0x08, 0x08, 0x08,	/* 0x59: Y  */
    0x08, 0x14, 0x22, 0x41, 0xC1, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x7E, 0x40, 0x20, 0x28, 0x10,	/* 0x5a: Z  */
    0x28, 0x04, 0x04, 0x02, 0x7E, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x38, 0x20, 0x20, 0x20, 0x20,	/* 0x5b: [  */
    0x20, 0x20, 0x20, 0x20, 0x38, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x02, 0x02, 0x04, 0x04, 0x08, 0x08,	/* 0x5c: \  */
    0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x1C, 0x04, 0x04, 0x04, 0x04,	/* 0x5d: ]  */
    0x04, 0x04, 0x04, 0x04, 0x1C, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x5e: ^  */
    0x00, 0x00, 0x22, 0x14, 0x08, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00,	/* 0x5f: _  */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x60: `  */
    0x00, 0x00, 0x00, 0x08, 0x14, 0x08, 0x10, 0x0c },
  { 0x00, 0x00, 0x00, 0x32, 0x4C, 0x44, 0x24, 0x1C,	/* 0x61: a  */
    0x44, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x5C, 0x22, 0x22, 0x22, 0x32,	/* 0x62: b  */
    0x2C, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x20, 0x20, 0x24,	/* 0x63: c  */
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x32, 0x4C, 0x44, 0x44, 0x24,	/* 0x64: d  */
    0x1C, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x20, 0x3C, 0x24,	/* 0x65: e  */
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x20, 0x10, 0x10, 0x10, 0x10, 0x10,	/* 0x66: f  */
    0x10, 0x10, 0x38, 0x10, 0x10, 0x10, 0x0C, 0x00 },
  { 0x18, 0x24, 0x04, 0x1C, 0x24, 0x24, 0x24, 0x24,	/* 0x67: g  */
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0x34, 0x28,	/* 0x68: h  */
    0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x69: i  */
    0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00 },
  { 0x20, 0x10, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x6a: j  */
    0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x24, 0x28, 0x30, 0x30, 0x28,	/* 0x6b: k  */
    0x24, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x1C, 0x08, 0x08, 0x08, 0x08,	/* 0x6c: l  */
    0x08, 0x08, 0x08, 0x08, 0x18, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x49, 0x49, 0x49, 0x49, 0x5B,	/* 0x6d: m  */
    0xB6, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x24, 0x24, 0x24, 0x24, 0x24,	/* 0x6e: n  */
    0x34, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x24, 0x24, 0x24,	/* 0x6f: o  */
    0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x20, 0x20, 0x20, 0x20, 0x38, 0x24, 0x24, 0x34,	/* 0x70: p  */
    0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x04, 0x04, 0x04, 0x04, 0x1C, 0x24, 0x24, 0x2C,	/* 0x71: q  */
    0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x18,	/* 0x72: r  */
    0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x18, 0x24, 0x08, 0x10, 0x24,	/* 0x73: s  */
    0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x10, 0x10, 0x10, 0x10,	/* 0x74: t  */
    0x38, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x14, 0x2C, 0x24, 0x24, 0x24,	/* 0x75: u  */
    0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x08, 0x08, 0x14, 0x14, 0x22,	/* 0x76: v  */
    0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x22, 0x22, 0x5A, 0x5A, 0x81,	/* 0x77: w  */
    0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x42, 0x24, 0x18, 0x18, 0x24,	/* 0x78: x  */
    0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x20, 0x10, 0x10, 0x08, 0x08, 0x0C, 0x14, 0x22,	/* 0x79: y  */
    0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x00, 0x00, 0x00, 0x3C, 0x20, 0x10, 0x08, 0x04,	/* 0x7a: z  */
    0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x06, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x30,	/* 0x7b: {  */
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x06, 0x00 },
  { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,	/* 0x7c: |  */
    0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00 },
  { 0x60, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0c,	/* 0x7d: }  */
    0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x60, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x7e: ~  */
    0x00, 0x00, 0x00, 0x06, 0x49, 0x49, 0x30, 0x00 },
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x7f:    */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

#endif // BITMAPFONT_H
