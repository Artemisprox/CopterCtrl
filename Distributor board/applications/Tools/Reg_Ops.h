#ifndef __REG_OPS_H__
#define __REG_OPS_H__

#define bit_setb(reg, bitnum) _bit_setb(reg, bitnum) // reg.bitnum = 1
#define bit_clr(reg, bitnum) _bit_clr(reg, bitnum)   // reg.bitnum = 0

#define _bit_setb(reg, bitnum) reg |= (1 << bitnum)   // reg.bitnum = 1
#define _bit_clr(reg, bitnum) reg &= (~(1 << bitnum)) // reg.bitnum = 0

#define _REG_CHANGER(reg, mask, change) reg = ((reg) & (~(mask))) | ((change) & (mask))
#define _REG_DEFINER(BINval, StartBit) ((BINval) << (StartBit))
#define _REG_H(reg) (((char *)&reg)[0])
#define _REG_L(reg) (((char *)&reg)[1])

#define _REG_HH(reg) (((char *)&reg)[0])
#define _REG_HL(reg) (((char *)&reg)[1])
#define _REG_LH(reg) (((char *)&reg)[2])
#define _REG_LL(reg) (((char *)&reg)[3])

// 32位取最高字节 此表达式可当作变量名使用，可读可写
#define REG_HH(reg) _REG_HH(reg)
// 32位取次高字节 此表达式可当作变量名使用，可读可写
#define REG_HL(reg) _REG_HL(reg)
// 32位取次低字节 此表达式可当作变量名使用，可读可写
#define REG_LH(reg) _REG_LH(reg)
// 32位取最低字节 此表达式可当作变量名使用，可读可写
#define REG_LL(reg) _REG_LL(reg)

// 使用时reg为目标寄存器，mask为1处表示需要修改，change表示需要改为的数值
#define REG_CHANGER(reg, mask, change) _REG_CHANGER(reg, mask, change)
// 使用时BINval写寄存器的段长对应的BIN宏，Startbit写BIN宏需要左移的量
#define REG_DEFINER(BINval, StartBit) _REG_DEFINER(BINval, StartBit)

// 16位取高字节 此表达式可当作变量名使用，可读可写
#define REG_H(reg) _REG_H(reg)
// 16位取低字节 此表达式可当作变量名使用，可读可写
#define REG_L(reg) _REG_L(reg)
// 高低字节拼接 将regH和regL分别放置在TargetReg的高字节和低字节 此函数是对TargetReg进行操作，无返回值
#define REG_CCT(TargetReg, regH, regL) \
    ((char *)&TargetReg)[1] = regL;    \
    ((char *)&TargetReg)[0] = regH

// 得到reg.bit的值
#define REG_GETBIT(reg, bitdat) ((reg & (1 << bitdat)) == (1 << bitdat))

#define BIN00000000 0x0
#define BIN00000001 0x1
#define BIN00000010 0x2
#define BIN00000011 0x3
#define BIN00000100 0x4
#define BIN00000101 0x5
#define BIN00000110 0x6
#define BIN00000111 0x7
#define BIN00001000 0x8
#define BIN00001001 0x9
#define BIN00001010 0xA
#define BIN00001011 0xB
#define BIN00001100 0xC
#define BIN00001101 0xD
#define BIN00001110 0xE
#define BIN00001111 0xF
#define BIN00010000 0x10
#define BIN00010001 0x11
#define BIN00010010 0x12
#define BIN00010011 0x13
#define BIN00010100 0x14
#define BIN00010101 0x15
#define BIN00010110 0x16
#define BIN00010111 0x17
#define BIN00011000 0x18
#define BIN00011001 0x19
#define BIN00011010 0x1A
#define BIN00011011 0x1B
#define BIN00011100 0x1C
#define BIN00011101 0x1D
#define BIN00011110 0x1E
#define BIN00011111 0x1F
#define BIN00100000 0x20
#define BIN00100001 0x21
#define BIN00100010 0x22
#define BIN00100011 0x23
#define BIN00100100 0x24
#define BIN00100101 0x25
#define BIN00100110 0x26
#define BIN00100111 0x27
#define BIN00101000 0x28
#define BIN00101001 0x29
#define BIN00101010 0x2A
#define BIN00101011 0x2B
#define BIN00101100 0x2C
#define BIN00101101 0x2D
#define BIN00101110 0x2E
#define BIN00101111 0x2F
#define BIN00110000 0x30
#define BIN00110001 0x31
#define BIN00110010 0x32
#define BIN00110011 0x33
#define BIN00110100 0x34
#define BIN00110101 0x35
#define BIN00110110 0x36
#define BIN00110111 0x37
#define BIN00111000 0x38
#define BIN00111001 0x39
#define BIN00111010 0x3A
#define BIN00111011 0x3B
#define BIN00111100 0x3C
#define BIN00111101 0x3D
#define BIN00111110 0x3E
#define BIN00111111 0x3F
#define BIN01000000 0x40
#define BIN01000001 0x41
#define BIN01000010 0x42
#define BIN01000011 0x43
#define BIN01000100 0x44
#define BIN01000101 0x45
#define BIN01000110 0x46
#define BIN01000111 0x47
#define BIN01001000 0x48
#define BIN01001001 0x49
#define BIN01001010 0x4A
#define BIN01001011 0x4B
#define BIN01001100 0x4C
#define BIN01001101 0x4D
#define BIN01001110 0x4E
#define BIN01001111 0x4F
#define BIN01010000 0x50
#define BIN01010001 0x51
#define BIN01010010 0x52
#define BIN01010011 0x53
#define BIN01010100 0x54
#define BIN01010101 0x55
#define BIN01010110 0x56
#define BIN01010111 0x57
#define BIN01011000 0x58
#define BIN01011001 0x59
#define BIN01011010 0x5A
#define BIN01011011 0x5B
#define BIN01011100 0x5C
#define BIN01011101 0x5D
#define BIN01011110 0x5E
#define BIN01011111 0x5F
#define BIN01100000 0x60
#define BIN01100001 0x61
#define BIN01100010 0x62
#define BIN01100011 0x63
#define BIN01100100 0x64
#define BIN01100101 0x65
#define BIN01100110 0x66
#define BIN01100111 0x67
#define BIN01101000 0x68
#define BIN01101001 0x69
#define BIN01101010 0x6A
#define BIN01101011 0x6B
#define BIN01101100 0x6C
#define BIN01101101 0x6D
#define BIN01101110 0x6E
#define BIN01101111 0x6F
#define BIN01110000 0x70
#define BIN01110001 0x71
#define BIN01110010 0x72
#define BIN01110011 0x73
#define BIN01110100 0x74
#define BIN01110101 0x75
#define BIN01110110 0x76
#define BIN01110111 0x77
#define BIN01111000 0x78
#define BIN01111001 0x79
#define BIN01111010 0x7A
#define BIN01111011 0x7B
#define BIN01111100 0x7C
#define BIN01111101 0x7D
#define BIN01111110 0x7E
#define BIN01111111 0x7F
#define BIN10000000 0x80
#define BIN10000001 0x81
#define BIN10000010 0x82
#define BIN10000011 0x83
#define BIN10000100 0x84
#define BIN10000101 0x85
#define BIN10000110 0x86
#define BIN10000111 0x87
#define BIN10001000 0x88
#define BIN10001001 0x89
#define BIN10001010 0x8A
#define BIN10001011 0x8B
#define BIN10001100 0x8C
#define BIN10001101 0x8D
#define BIN10001110 0x8E
#define BIN10001111 0x8F
#define BIN10010000 0x90
#define BIN10010001 0x91
#define BIN10010010 0x92
#define BIN10010011 0x93
#define BIN10010100 0x94
#define BIN10010101 0x95
#define BIN10010110 0x96
#define BIN10010111 0x97
#define BIN10011000 0x98
#define BIN10011001 0x99
#define BIN10011010 0x9A
#define BIN10011011 0x9B
#define BIN10011100 0x9C
#define BIN10011101 0x9D
#define BIN10011110 0x9E
#define BIN10011111 0x9F
#define BIN10100000 0xA0
#define BIN10100001 0xA1
#define BIN10100010 0xA2
#define BIN10100011 0xA3
#define BIN10100100 0xA4
#define BIN10100101 0xA5
#define BIN10100110 0xA6
#define BIN10100111 0xA7
#define BIN10101000 0xA8
#define BIN10101001 0xA9
#define BIN10101010 0xAA
#define BIN10101011 0xAB
#define BIN10101100 0xAC
#define BIN10101101 0xAD
#define BIN10101110 0xAE
#define BIN10101111 0xAF
#define BIN10110000 0xB0
#define BIN10110001 0xB1
#define BIN10110010 0xB2
#define BIN10110011 0xB3
#define BIN10110100 0xB4
#define BIN10110101 0xB5
#define BIN10110110 0xB6
#define BIN10110111 0xB7
#define BIN10111000 0xB8
#define BIN10111001 0xB9
#define BIN10111010 0xBA
#define BIN10111011 0xBB
#define BIN10111100 0xBC
#define BIN10111101 0xBD
#define BIN10111110 0xBE
#define BIN10111111 0xBF
#define BIN11000000 0xC0
#define BIN11000001 0xC1
#define BIN11000010 0xC2
#define BIN11000011 0xC3
#define BIN11000100 0xC4
#define BIN11000101 0xC5
#define BIN11000110 0xC6
#define BIN11000111 0xC7
#define BIN11001000 0xC8
#define BIN11001001 0xC9
#define BIN11001010 0xCA
#define BIN11001011 0xCB
#define BIN11001100 0xCC
#define BIN11001101 0xCD
#define BIN11001110 0xCE
#define BIN11001111 0xCF
#define BIN11010000 0xD0
#define BIN11010001 0xD1
#define BIN11010010 0xD2
#define BIN11010011 0xD3
#define BIN11010100 0xD4
#define BIN11010101 0xD5
#define BIN11010110 0xD6
#define BIN11010111 0xD7
#define BIN11011000 0xD8
#define BIN11011001 0xD9
#define BIN11011010 0xDA
#define BIN11011011 0xDB
#define BIN11011100 0xDC
#define BIN11011101 0xDD
#define BIN11011110 0xDE
#define BIN11011111 0xDF
#define BIN11100000 0xE0
#define BIN11100001 0xE1
#define BIN11100010 0xE2
#define BIN11100011 0xE3
#define BIN11100100 0xE4
#define BIN11100101 0xE5
#define BIN11100110 0xE6
#define BIN11100111 0xE7
#define BIN11101000 0xE8
#define BIN11101001 0xE9
#define BIN11101010 0xEA
#define BIN11101011 0xEB
#define BIN11101100 0xEC
#define BIN11101101 0xED
#define BIN11101110 0xEE
#define BIN11101111 0xEF
#define BIN11110000 0xF0
#define BIN11110001 0xF1
#define BIN11110010 0xF2
#define BIN11110011 0xF3
#define BIN11110100 0xF4
#define BIN11110101 0xF5
#define BIN11110110 0xF6
#define BIN11110111 0xF7
#define BIN11111000 0xF8
#define BIN11111001 0xF9
#define BIN11111010 0xFA
#define BIN11111011 0xFB
#define BIN11111100 0xFC
#define BIN11111101 0xFD
#define BIN11111110 0xFE
#define BIN11111111 0xFF

#endif
