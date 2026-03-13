#include "referee.hpp"

namespace Referee_n {

    /**************************** 裁判系统文档内容 裁判系统用函数 *********************/

    referee_c referee_c::RefereeInstance = referee_c();

#ifdef USE_REFEREE

    static void ReOnline()
    {
        referee_c::GetInstance()->is_online_re_ = 1;
    }

    static void ReDisonline(void)
    {
        referee_c::GetInstance()->RefereeClear();
    }

    void RefereeCallback(const uint8_t *pdata, uint16_t psize)// 裁判系统数据回调函数
    {
        memcpy(referee_c::GetInstance()->referee_data_.RefereeData, pdata, psize);
        referee_c::GetInstance()->referee_data_.DataLen = psize;
        referee_c::GetInstance()->RefereeDataProcess();
    }

    void referee_c::RefereeClear(void)
    {
        this->is_online_re_ = 0;

        this->referee_data_.PHOTO_ctrl.kb.key_code = 0;
        this->referee_data_.PHOTO_ctrl.mouse.press_l = 0;
        this->referee_data_.PHOTO_ctrl.mouse.press_r = 0;
        this->referee_data_.PHOTO_ctrl.mouse.x = 0;
        this->referee_data_.PHOTO_ctrl.mouse.y = 0;
        this->referee_data_.PHOTO_ctrl.mouse.z = 0;
    }

    void referee_c::RefereeInit()
    {
        // 裁判系统串口配置
        BSP_n::USART_c::Config referee_uart_param = {
            .Handle = &REFEREE_USART,
             .RxBuffSize = REFEREE_RX_Buffer_Num,// 接收区大小
             .RxType = BSP_n::USART_c::RxType_t::DMA_IDLE_DOUBLE,// 接收类型 DMA_IDLE_DOUBLE
             .TxType = BSP_n::USART_c::TxType_t::DMA,// 发送类型
             .pTxCallback = nullptr,// 发送回调,初始化都为nullptr
             .pRxCallback = &RefereeCallback,// 接收回调函数指针
             .pRxBuffer = referee_rx_buffer_[0],
             .pSecRxBuffer = referee_rx_buffer_[1]};
        referee = new BSP_n::USART_c(referee_uart_param);
        referee_safe = new SafeTask_c("RE", 500, ReDisonline, ReOnline);
        referee->start_receive();
    }

    /**
     * @brief 裁判系统数据段解析 与 整包CRC16校验
     * @param RefereeData 帧头解析出来的CMDID对应的细分数据结构体指针
     * @param frame_header 裁判系统此次通信的帧头指针
     * @param data_length  帧头解析出来的CMDID对应的细分数据结构体长度
     */
    void referee_c::RefereeDataCRC16Deal(void *RefereeData, uint8_t *frame_header,
                                      uint8_t data_length)
    {
        uint8_t *RefereeDataU8 = (uint8_t *) RefereeData;
        if (Verify_CRC16_Check_Sum(frame_header, HEADER_LEN + CMDID_LEN +
                                                         data_length + CRC16_LEN) ==
            1)// 整包CRC16校验
        {
            // 校验通过，搬运到数据结构体内
            memcpy(RefereeData, &frame_header[HEADER_LEN + CMDID_LEN], data_length);
            // 操作数据结构体内error，data_length不考虑数据结构体内error占用，因此指针移动后指向error所在字节
            memset(RefereeDataU8 + data_length, 0, sizeof(uint8_t));
            referee_safe->Online();
        } else
            memset(RefereeDataU8 + data_length, 1, sizeof(uint8_t));
    }

    /**
     * @brief 图传控制/裁判系统共用数据解析
     * @note  图传控制一次转发
     */
    void referee_c::RefereeDataProcess()
    {
        uint8_t i;
        for (i = 0; i < referee_data_.DataLen; i++) {
            if (referee_data_.RefereeData[i] == 0xA5) {// 帧头
                if (Verify_CRC8_Check_Sum(&referee_data_.RefereeData[i], HEADER_LEN) ==
                    1) {// 帧头CRC8校验
                    referee_data_.RealLen = ((referee_data_.RefereeData[i + 1]) |
                                        (referee_data_.RefereeData[i + 2] << 8));
                    referee_data_.Cmd_ID =
                            ((referee_data_.RefereeData[i + HEADER_LEN]) |
                             (referee_data_.RefereeData[i + HEADER_LEN + 1] << 8));// 命令码ID
                    switch (referee_data_.Cmd_ID) {
                        case ID_PICTURE_TRANSMISSION://把图传链路放前面，优化解析速度
                            RefereeDataCRC16Deal(&this->referee_data_.PHOTO_ctrl,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_PICTURE_TRANSMISSION_LEN);
                            i = i + (DATA_PICTURE_TRANSMISSION_LEN + 9) - 1;
                            break;
                        case ID_STATE:
                            RefereeDataCRC16Deal(&this->referee_data_.Game_Status,
                                                 &referee_data_.RefereeData[i], DATA_STATUS_LEN);
                            i = i + (DATA_STATUS_LEN + 9) + 9 - 1;
                            break;
                        case ID_RESULT:
                            RefereeDataCRC16Deal(&this->referee_data_.Game_Result,
                                                 &referee_data_.RefereeData[i], DATA_RESULT_LEN);
                            i = i + (DATA_RESULT_LEN + 9) - 1;
                            break;
                        case ID_ROBOT_HP:
                            RefereeDataCRC16Deal(&this->referee_data_.Robot_HP,
                                                 &referee_data_.RefereeData[i], DATA_ROBOT_HP_LEN);
                            i = i + (DATA_ROBOT_HP_LEN + 9) - 1;
                            break;
                        case ID_EVENT_DATA:
                            RefereeDataCRC16Deal(&this->referee_data_.Event_Data,
                                                 &referee_data_.RefereeData[i], DATA_EVENT_DATA_LEN);
                            i = i + (DATA_EVENT_DATA_LEN + 9) - 1;
                            break;
                        case ID_SUPPLY_PROJECTILE_ACTION:
                            RefereeDataCRC16Deal(&this->referee_data_.Supply_Action,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_SUPPLY_PROJECTILE_ACTION_LEN);
                            i = i + (DATA_SUPPLY_PROJECTILE_ACTION_LEN + 9) - 1;
                            break;
                        case ID_REFEREE_WARNING:
                            RefereeDataCRC16Deal(&this->referee_data_.Referee_Warning,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_REFEREE_WARNING_LEN);
                            i = i + (DATA_REFEREE_WARNING_LEN + 9) - 1;
                            break;
                        case ID_DART_REMAINING_TIME:
                            RefereeDataCRC16Deal(&this->referee_data_.Dart_Remaining_Time,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_DART_REMAINING_TIME_LEN);
                            i = i + (DATA_DART_REMAINING_TIME_LEN + 9) - 1;
                            break;
                        case ID_ROBOT_STATE:
                            RefereeDataCRC16Deal(&this->referee_data_.Robot_Status,
                                                 &referee_data_.RefereeData[i], DATA_ROBOT_STATUS_LEN);
                            i = i + (DATA_ROBOT_STATUS_LEN + 9) - 1;
                            break;
                        case ID_POWER_HEAT_DATA:
                            RefereeDataCRC16Deal(&this->referee_data_.Power_Heat,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_POWER_HEAT_DATA_LEN);
                            i = i + (DATA_POWER_HEAT_DATA_LEN + 9) - 1;
                            break;
                        case ID_ROBOT_POS:
                            RefereeDataCRC16Deal(&this->referee_data_.Robot_Position,
                                                 &referee_data_.RefereeData[i], DATA_ROBOT_POS_LEN);
                            i = i + (DATA_ROBOT_POS_LEN + 9) - 1;
                            break;
                        case ID_BUFF:
                            RefereeDataCRC16Deal(&this->referee_data_.Buff, &referee_data_.RefereeData[i],
                                                 DATA_BUFF_LEN);
                            i = i + (DATA_BUFF_LEN + 9) - 1;
                            break;
                        case ID_AERIAL_ROBOT_ENERGY:
                            RefereeDataCRC16Deal(&this->referee_data_.Aerial_Energy,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_AERIAL_ROBOT_ENERGY_LEN);
                            i = i + (DATA_AERIAL_ROBOT_ENERGY_LEN + 9) - 1;
                            break;
                        case ID_ROBOT_HURT:
                            RefereeDataCRC16Deal(&this->referee_data_.Robot_Hurt,
                                                 &referee_data_.RefereeData[i], DATA_ROBOT_HURT_LEN);
                            i = i + (DATA_ROBOT_HURT_LEN + 9) - 1;
                            break;
                        case ID_SHOOT_DATA:
                            RefereeDataCRC16Deal(&this->referee_data_.Shoot_Data,
                                                 &referee_data_.RefereeData[i], DATA_SHOOT_DATA_LEN);
                            i = i + (DATA_SHOOT_DATA_LEN + 9) - 1;
                            break;
                        case ID_BULLET_REMAINING:
                            RefereeDataCRC16Deal(&this->referee_data_.Bullet_Num,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_BULLET_REMAINING_LEN);
                            i = i + (DATA_BULLET_REMAINING_LEN + 9) - 1;
                            break;
                        case ID_RFID_STATUS:
                            RefereeDataCRC16Deal(&this->referee_data_.RFID_Status,
                                                 &referee_data_.RefereeData[i], DATA_RFID_STATUS_LEN);
                            i = i + (DATA_RFID_STATUS_LEN + 9) - 1;
                            break;
                        case ID_DART_CLIENT_CMD:
                            RefereeDataCRC16Deal(&this->referee_data_.Dart_Client,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_DART_CLIENT_CMD_LEN);
                            i = i + (DATA_DART_CLIENT_CMD_LEN + 9) - 1;
                            break;
                        case ID_GROUND_ROBOT_POSITION:
                            RefereeDataCRC16Deal(&this->referee_data_.Robot_Position_Al,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_ROBOT_POSITION_LEN);
                            i = i + (DATA_ROBOT_POSITION_LEN + 9) - 1;
                            break;
                        case ID_RARD_MRAK_DATA:
                            RefereeDataCRC16Deal(&this->referee_data_.radar_mark,
                                                 &referee_data_.RefereeData[i], DATA_RADAR_MARK_LEN);
                            i = i + (DATA_RADAR_MARK_LEN + 9) - 1;
                            break;
                        case ID_SENTRY:
                            RefereeDataCRC16Deal(&this->referee_data_.sentry, &referee_data_.RefereeData[i],
                                                 DATA_SENTRY_INFO_LEN);
                            i = i + (DATA_SENTRY_INFO_LEN + 9) - 1;
                            break;
                        case ID_RADAR:
                            RefereeDataCRC16Deal(&this->referee_data_.radar_info,
                                                 &referee_data_.RefereeData[i], DATA_RADAR_INFO_LEN);
                            i = i + (DATA_RADAR_INFO_LEN + 9) - 1;
                            break;
                        // case 0x301:
                        case ID_DIY_CONTROLLER:
                            RefereeDataCRC16Deal(&this->referee_data_.DIY_control,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_DIY_CONTROLLER_LEN);
                            i = i + (DATA_DIY_CONTROLLER_LEN + 9) - 1;
                            break;
                        case ID_CLIENT_DOWMLOAD:// 修订
                            RefereeDataCRC16Deal(&this->referee_data_.ClientMapData,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_CLIENT_DOWMLOAD_LEN);
                            i = i + (DATA_CLIENT_DOWMLOAD_LEN + 9) - 1;
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
#endif

#ifdef USE_VT13

    /*图传接收端回调函数*/
    void vt13_data_callback(const uint8_t *pdata, uint16_t psize)
    {
        memcpy(referee_c::GetInstance()->referee_data_.RefereeData, pdata, psize);
        referee_c::GetInstance()->referee_data_.DataLen = psize;
        referee_c::GetInstance()->VtmDataProcess();
        // referee_c::GetInstance().vt13_data__DataProcess();
    }
    static void VtmOnline()
    {
        referee_c::GetInstance()->is_online_tc_ = 1;
    }
    void referee_c::VtmClear(void)
    {
        // 计算 ch_0 到 key 的字节数
        size_t size = reinterpret_cast<uint8_t*>(&referee_c::GetInstance()->vt13_data_.data) - reinterpret_cast<uint8_t*>(&referee_c::GetInstance()->vt13_data_.sof_1);
        // 使用 memset 清零
        memset(&GetInstance()->vt13_data_.sof_1, 0, size);;
        GetInstance()->vt13_data_.mode_sw = 1;
    }
    static void VtmDisonline(void)
    {
        referee_c::GetInstance()->VtmClear();
        referee_c::GetInstance()->is_online_tc_ = 0;
    }
    void referee_c::VtmInit()
    {
        // 视觉通信串口配置
        BSP_n::USART_c::Config vtm_uart_param = {
            .Handle = &VTM_USART,
            .RxBuffSize = REFEREE_RX_Buffer_Num,    // 接收区大小
            .RxType = BSP_n::USART_c::RxType_t::DMA_IDLE_DOUBLE,// 接收类型 DMA_IDLE_DOUBLE
            .TxType = BSP_n::USART_c::TxType_t::DMA,// 发送类型
            .pTxCallback = nullptr,// 发送回调,初始化都为nullptr
            .pRxCallback = &vt13_data_callback,// 接收回调函数指针
            .pRxBuffer = referee_c::GetInstance()->referee_rx_buffer_[0],
            .pSecRxBuffer = referee_c::GetInstance()->referee_rx_buffer_[1]};
        TC_Safe = new SafeTask_c("TC", 200, VtmDisonline, VtmOnline);
        vtm = new BSP_n::USART_c(vtm_uart_param);
        vtm->start_receive();
    }

    /**
     * @brief VT13图传遥控数据段解析
     * 
     */
    void referee_c::VtmDataProcess()
    {
        uint8_t i;
        for (i = 0; i < referee_data_.DataLen; i++) {
            if (referee_data_.RefereeData[i] == 0xA5) {                                     // 帧头
                if (Verify_CRC8_Check_Sum(&referee_data_.RefereeData[i], HEADER_LEN) == 1) {// 帧头CRC8校验
                    referee_data_.RealLen = ((referee_data_.RefereeData[i + 1]) |
                                        (referee_data_.RefereeData[i + 2] << 8));
                    referee_data_.Cmd_ID =
                            ((referee_data_.RefereeData[i + HEADER_LEN]) |
                             (referee_data_.RefereeData[i + HEADER_LEN + 1] << 8));// 命令码ID
                    switch (referee_data_.Cmd_ID) {
                        case ID_PICTURE_TRANSMISSION:
                            RefereeDataCRC16Deal(&this->referee_data_.PHOTO_ctrl,
                                                 &referee_data_.RefereeData[i],
                                                 DATA_PICTURE_TRANSMISSION_LEN);
                            i = i + (DATA_PICTURE_TRANSMISSION_LEN + 9) - 1;
                            TC_Safe->Online();
                            break;
                        default:
                            break;
                    }
                }
            } else if (i != (referee_data_.DataLen - 1) && referee_data_.RefereeData[i] == 0xA9 &&
                       referee_data_.RefereeData[i + 1] == 0x53) {
                if (Verify_CRC16_Check_Sum(&referee_data_.RefereeData[i], 21) == 1) {
                    memcpy(&this->vt13_data_, &referee_data_.RefereeData[i], 21);
                    vt13_data_.ch[0] = ((int16_t) vt13_data_.ch_0) - VT13_CH_VALUE_OFFSET;
                    vt13_data_.ch[1] = ((int16_t) vt13_data_.ch_1) - VT13_CH_VALUE_OFFSET;
                    vt13_data_.ch[2] = ((int16_t) vt13_data_.ch_2) - VT13_CH_VALUE_OFFSET;
                    vt13_data_.ch[3] = ((int16_t) vt13_data_.ch_3) - VT13_CH_VALUE_OFFSET;
                    vt13_data_.ch[4] = ((int16_t) vt13_data_.wheel) - VT13_CH_VALUE_OFFSET;
                    TC_Safe->Online();
                }
            }
        }
    }
#endif

    // /**************************** 裁判系统文档内容 UI用函数 *********************/
    //
    //
    // unsigned char UI_Seq = 0; //包序号
    // extern UART_HandleTypeDef huart1;
    // /****************************************串口驱动映射************************************/
    // void UI_SendByte(unsigned char ch)
    // {
	   //  HAL_UART_Transmit(&huart1,&ch,sizeof(unsigned char),1);
    // }
    // /********************************************删除操作*************************************
    // **参数：Del_Operate  对应头文件删除操作
    //         Del_Layer    要删除的层 取值0-9
    // *****************************************************************************************/
    // void UI_c::UI_Delete(uint8_t Del_Operate, uint8_t Del_Layer)
    // {
    //     unsigned char *framepoint;   //读写指针
    //     uint16_t frametail = 0xFFFF; //CRC16校验值
    //     int loop_control;            //For函数循环控制
    //
    //     UI_Packhead framehead;
    //     UI_Data_Operate datahead;
    //     UI_Data_Delete del;
    //
    //     framepoint = (unsigned char *)&framehead;
    //
    //     framehead.SOF = UI_SOF;
    //     framehead.Data_Length = 8;
    //     framehead.Seq = UI_Seq;
    //     framehead.CRC8 = Get_CRC8_Check_Sum(framepoint, 4, 0xFF);
    //     framehead.CMD_ID = UI_CMD_Robo_Exchange; //填充包头数据
    //
    //     datahead.Data_ID = UI_Data_ID_Del;
    //     datahead.Sender_ID = Robot_ID;
    //     datahead.Receiver_ID = Client_ID; //填充操作数据
    //
    //     del.Delete_Operate = Del_Operate;
    //     del.Layer = Del_Layer; //控制信息
    //
    //     frametail = Get_CRC16_Check_Sum(framepoint, sizeof(framehead), frametail);
    //     framepoint = (unsigned char *)&datahead;
    //     frametail = Get_CRC16_Check_Sum(framepoint, sizeof(datahead), frametail);
    //     framepoint = (unsigned char *)&del;
    //     frametail = Get_CRC16_Check_Sum(framepoint, sizeof(del), frametail); //CRC16校验值计算
    //
    //     framepoint = (unsigned char *)&framehead;
    //
    //     for (loop_control = 0; loop_control < sizeof(framehead); loop_control++)
    //     {
    //         UI_SendByte(*framepoint);
    //         framepoint++;
    //     }
    //
    //     framepoint = (unsigned char *)&datahead;
    //
    //     for (loop_control = 0; loop_control < sizeof(datahead); loop_control++)
    //     {
    //         UI_SendByte(*framepoint);
    //         framepoint++;
    //     }
    //
    //     framepoint = (unsigned char *)&del;
    //
    //     for (loop_control = 0; loop_control < sizeof(del); loop_control++)
    //     {
    //         UI_SendByte(*framepoint);
    //         framepoint++;
    //     } //发送所有帧
    //
    //     framepoint = (unsigned char *)&frametail;
    //
    //     for (loop_control = 0; loop_control < sizeof(frametail); loop_control++)
    //     {
    //         UI_SendByte(*framepoint);
    //         framepoint++; //发送CRC16校验值
    //     }
    //
    //     UI_Seq++; //包序号+1
    // }
    // /************************************************绘制直线*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Start_x、Start_x    开始坐标
    //         End_x、End_y   结束坐标
    // **********************************************************************************************************/
    // void UI_c::Line_Draw(
    //     Graph_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_Width,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     uint32_t End_x,
    //     uint32_t End_y)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->graphic_name[2 - i] = imagename[i];
    //
    //     image->operate_tpye = Graph_Operate;
    //     image->layer = Graph_Layer;
    //     image->color = Graph_Color;
    //     image->width = Graph_Width;
    //     image->start_x = Start_x;
    //     image->start_y = Start_y;
    //     image->end_x = End_x;
    //     image->end_y = End_y;
    // }
    //
    // /************************************************绘制矩形*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Start_x、Start_x    开始坐标
    //         End_x、End_y   结束坐标（对顶角坐标）
    // **********************************************************************************************************/
    // void UI_c::Rectangle_Draw(
    //     Graph_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_Width,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     uint32_t End_x,
    //     uint32_t End_y)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->graphic_name[2 - i] = imagename[i];
    //
    //     image->graphic_tpye = UI_Graph_Rectangle;
    //     image->operate_tpye = Graph_Operate;
    //     image->layer = Graph_Layer;
    //     image->color = Graph_Color;
    //     image->width = Graph_Width;
    //     image->start_x = Start_x;
    //     image->start_y = Start_y;
    //     image->end_x = End_x;
    //     image->end_y = End_y;
    // }
    //
    // /************************************************绘制整圆*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Start_x、Start_x    圆心坐标
    //         Graph_Radius  图形半径
    // **********************************************************************************************************/
    // void UI_c::Circle_Draw(
    //     Graph_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_Width,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     uint32_t Graph_Radius)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->graphic_name[2 - i] = imagename[i];
    //
    //     image->graphic_tpye = UI_Graph_Circle;
    //     image->operate_tpye = Graph_Operate;
    //     image->layer = Graph_Layer;
    //     image->color = Graph_Color;
    //     image->width = Graph_Width;
    //     image->start_x = Start_x;
    //     image->start_y = Start_y;
    //     image->radius = Graph_Radius;
    // }
    //
    // /************************************************绘制圆弧*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Graph_StartAngle,Graph_EndAngle    开始，终止角度
    //         Start_y,Start_y    圆心坐标
    //         x_Length,y_Length   x,y方向上轴长，参考椭圆
    // **********************************************************************************************************/
    // void UI_c::Arc_Draw(
    //     Graph_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_StartAngle,
    //     uint32_t Graph_EndAngle,
    //     uint32_t Graph_Width,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     uint32_t x_Length,
    //     uint32_t y_Length)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->graphic_name[2 - i] = imagename[i];
    //
    //     image->graphic_tpye = UI_Graph_Arc;
    //     image->operate_tpye = Graph_Operate;
    //     image->layer = Graph_Layer;
    //     image->color = Graph_Color;
    //     image->width = Graph_Width;
    //     image->start_x = Start_x;
    //     image->start_y = Start_y;
    //     image->start_angle = Graph_StartAngle;
    //     image->end_angle = Graph_EndAngle;
    //     image->end_x = x_Length;
    //     image->end_y = y_Length;
    // }
    //
    // /************************************************绘制浮点型数据*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Graph_Size     字号
    //         Graph_Digit    小数位数
    //         Start_x、Start_x    开始坐标
    //         Graph_Float   要显示的变量
    // **********************************************************************************************************/
    // void UI_c::Float_Draw(
    //     Float_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_Size,
    //     uint32_t Graph_Digit,
    //     uint32_t Graph_Width,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     float Graph_Float)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->graphic_name[2 - i] = imagename[i];
    //
    //     image->graphic_tpye = UI_Graph_Float;
    //     image->operate_tpye = Graph_Operate;
    //     image->layer = Graph_Layer;
    //     image->color = Graph_Color;
    //     image->width = Graph_Width;
    //     image->start_x = Start_x;
    //     image->start_y = Start_y;
    //     image->start_angle = Graph_Size;
    //     image->end_angle = Graph_Digit;
    //     image->graph_Float = Graph_Float;
    // }
    //
    // /************************************************绘制字符型数据*************************************************
    // **参数：*image Graph_Data类型变量指针，用于存放图形数据
    //         imagename[3]   图片名称，用于标识更改
    //         Graph_Operate   图片操作，见头文件
    //         Graph_Layer    图层0-9
    //         Graph_Color    图形颜色
    //         Graph_Width    图形线宽
    //         Graph_Size     字号
    //         Start_x、Start_y    开始坐标
    //         *Char_Data          待发送字符串开始地址
    // **********************************************************************************************************/
    // void UI_c::Char_Draw(
    //     String_Data *image,
    //     char imagename[3],
    //     uint32_t Graph_Operate,
    //     uint32_t Graph_Layer,
    //     uint32_t Graph_Color,
    //     uint32_t Graph_Width,
    //     uint32_t Graph_Size,
    //     uint32_t Start_x,
    //     uint32_t Start_y,
    //     const char *Char_Data)
    // {
    //     int i;
    //
    //     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
    //         image->Graph_Control.graphic_name[2 - i] = imagename[i];
    //
    //     image->Graph_Control.graphic_tpye = UI_Graph_Char;
    //     image->Graph_Control.operate_tpye = Graph_Operate;
    //     image->Graph_Control.layer = Graph_Layer;
    //     image->Graph_Control.color = Graph_Color;
    //     image->Graph_Control.width = Graph_Width;
    //     image->Graph_Control.start_x = Start_x;
    //     image->Graph_Control.start_y = Start_y;
    //     image->Graph_Control.start_angle = Graph_Size;
    //     image->Graph_Control.end_angle = strlen(Char_Data);
    //
    //     strcpy(image->show_Data,Char_Data);
    // }

}
