#ifndef REFEREE_HPP
#define REFEREE_HPP

#include "bsp_usart.hpp"
#include "safe_task.hpp"
#include "referee_config.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "string.h"
#include "CRC.h"

#ifdef __cplusplus
}

namespace Referee_n {

    class referee_c {
    public:
        BSP_n::USART_c *referee;
        BSP_n::USART_c *vtm;
        SafeTask_c *referee_safe;
        SafeTask_c *TC_Safe;
        uint8_t is_online_re_;
        uint8_t is_online_tc_;
        uint8_t referee_rx_buffer_[2][REFEREE_RX_Buffer_Num] = {0};
        RefereeData_t referee_data_;
        VT13data_t vt13_data_;

        void RefereeInit();
        void RefereeDataProcess();
        void RefereeClear();
        void RefereeDataCRC16Deal(void *RefereeData, uint8_t *frame_header, uint8_t data_length);
        void VtmInit();
        void VtmDataProcess();
        void VtmClear();

        static referee_c *GetInstance()
        {
            return &RefereeInstance; // 返回静态成员变量 instance 的地址
        }
    public:
        referee_c() = default; // 私有构造函数
        static referee_c RefereeInstance;//唯一实例指针
    };
    void RefereeCallback(const uint8_t *pdata, uint16_t psize);// 裁判系统数据回调函数
    void VtmCallback(const uint8_t *pdata, uint16_t psize);// VT13图传数据回调函数

    // class UI_c {
    //     void UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer);
    //     void Line_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    //     // int UI_ReFresh(int cnt, ...);
    //     // unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
    //     // uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
    //     void Circle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
    //     void Rectangle_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
    //     void Float_Draw(Float_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float);
    //     void Char_Draw(String_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Size, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, const char *Char_Data);
    //     void Arc_Draw(Graph_Data *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t x_Length, uint32_t y_Length);
    //     // void ui_client(char *char_data);
    //     // //只渲染一个图形(简化代码)
    //     // void My_Graph_Refresh(Graph_Data *Graph);
    //     // void My_Char_Refresh(String_Data string);
    //     // void My_Graph_Refresh_seven(Graph_Data *Graph_1,
    //     //                                    Graph_Data *Graph_2,
    //     //                                    Graph_Data *Graph_3,
    //     //                                    Graph_Data *Graph_4,
    //     //                                    Graph_Data *Graph_5,
    //     //                                    Graph_Data *Graph_6,
    //     //                                    Graph_Data *Graph_7);
    // };
};



#endif

#endif
