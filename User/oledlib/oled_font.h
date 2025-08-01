#ifndef OLED_FONT_H
#define OLED_FONT_H

typedef enum 
{
	TEXT_BK_NULL=0,
	TEXT_BK_NOT_NULL,
}Type_textbk;

struct Cn16CharTypeDef                	// 汉字字模数据结构 
{
	unsigned char  Index[2];            // 汉字内码索引,一个汉字占两个字节	
	unsigned char  Msk[32];             // 点阵码数据(16*16有32个数据) 
};

#define NUM_OFCHINESE	12
extern const unsigned char font5x7[];
extern const unsigned char F8X16[];
extern struct Cn16CharTypeDef const CN16CHAR[NUM_OFCHINESE];



//功能:设置字体的背景模式 0为透明，1为正常。
void SetTextBkMode(Type_textbk value);
//功能:获取字体的背景模式 0为透明，1为正常。
Type_textbk GetTextBkMode(void);

unsigned char GetFontSize(void);
void SetFontSize(unsigned char value);
#endif

