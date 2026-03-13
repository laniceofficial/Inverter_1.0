#ifndef __CRC_H_
#define __CRC_H_

#ifdef __cplusplus   //这里是两个下滑线
extern "C"{
#endif

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned short Get_CRC16_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned short wCRC);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
unsigned int Verify_CRC16_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC16_Check_Sum(unsigned char * pchMessage,unsigned int dwLength);

#ifdef __cplusplus
}
#endif

#endif
