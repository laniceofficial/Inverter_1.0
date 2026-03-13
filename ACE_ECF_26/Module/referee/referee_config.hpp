#ifndef REFEREE_CONFIG_HPP
#define REFEREE_CONFIG_HPP

#ifdef USE_REFEREE
    #ifdef STM32H723xx
    #define REFEREE_USART huart2
    #elif STM32F405xx
    #define REFEREE_USART huart6
    #endif
#endif

#ifdef USE_VT13
    #ifdef STM32H723xx
    #define VTM_USART huart2
    #elif STM32F405xx
    #define VTM_USART huart6
    #endif
#endif

namespace Referee_n {

    /**************************** 裁判系统文档内容 裁判系统用数据 *********************/
    #define VT13_CH_VALUE_OFFSET ((uint16_t) 1024)// 遥控通道中间值
    #define REFEREE_RX_Buffer_Num 255
    #define NULL 0
    #define Referee_Data_len 128
        /* ----------------------- 帧头长度---------------------------- */
    #define HEADER_LEN 5
        /* ----------------------- 指令长度---------------------------- */
    #define CMDID_LEN 2
        /* ----------------------- CRC冗余码长度1---------------------------- */
    #define CRC16_LEN 2
        /* ----------------------- 数据段长度---------------------------- */
    #define DATA_STATUS_LEN 11   //! 比赛状态数据长度(官方有误)
    #define DATA_RESULT_LEN 1    // 比赛结果数据长度
    #define DATA_ROBOT_HP_LEN 32 //! 比赛机器人血量数据长度(官方有误)
    // #define DATA_DART_STATUS_LEN									3							//飞镖发射状态长度
    #define DATA_EVENT_DATA_LEN 4               // 场地事件数据长度
    #define DATA_SUPPLY_PROJECTILE_ACTION_LEN 4 //! 场地补给站动作标识数据长度(官方有误)
    #define DATA_REFEREE_WARNING_LEN 3          // 裁判警告数据长度
    #define DATA_DART_REMAINING_TIME_LEN 3      // 飞镖发射口倒计时
    #define DATA_ROBOT_STATUS_LEN 13            //! 机器人状态数据(官方有误)
    #define DATA_POWER_HEAT_DATA_LEN 16         //! 实时功率热量数据(官方有误)
    #define DATA_ROBOT_POS_LEN 12               // 机器人位置数据
    #define DATA_BUFF_LEN 1                     // 机器人增益数据
    #define DATA_AERIAL_ROBOT_ENERGY_LEN 2      //! 空中机器人能量状态数据,只有空中机器人主控发送(官方有误)
    #define DATA_ROBOT_HURT_LEN 1               // 伤害状态数据
    #define DATA_SHOOT_DATA_LEN 7               //! 实时射击数据(官方有误)
    #define DATA_BULLET_REMAINING_LEN 6         //! 子弹剩余发送数(官方有误)
    #define DATA_RFID_STATUS_LEN 4              // 机器人 RFID 状态
    #define DATA_DART_CLIENT_CMD_LEN 6          // 飞镖机器人客户端指令书
    #define DATA_ROBOT_POSITION_LEN 40
    #define DATA_RADAR_MARK_LEN 6
    #define DATA_SENTRY_INFO_LEN 4
    #define DATA_RADAR_INFO_LEN 1
    // #define DATA_STUDENT_INTERACTIVE_HEADER_DATA  6             //UI
    #define DATA_DIY_CONTROLLER_LEN 30           // 自定义控制器
    #define DATA_CLIENT_DOWMLOAD_LEN 15          // 小地图下发位置信息
    #define DATA_PICTURE_TRANSMISSION_LEN 12 // 图传遥控信息
    #define DATA_CLIENT_RECEIVE 10           // 小地图接收位置信息

        /* ----------------------- 命令码ID---------------------------- */
    #define ID_STATE 0x0001    // 比赛状态数据
    #define ID_RESULT 0x0002   // 比赛结果数据
    #define ID_ROBOT_HP 0x0003 // 比赛机器人机器人血量数据
    // #define ID_DART_STATUS 												0x0004				//飞镖发射状态
    #define ID_EVENT_DATA 0x0101                // 场地事件数据
    #define ID_SUPPLY_PROJECTILE_ACTION 0x0102  // 场地补给站动作标识数据
    #define ID_SUPPLY_PROJECTILE_BOOKING 0x0103 // 场地补给站预约子弹数据
    #define ID_REFEREE_WARNING 0x0104           // 裁判警告数据
    #define ID_DART_REMAINING_TIME 0x0105       // 飞镖发射口倒计时
    #define ID_ROBOT_STATE 0x0201               // 机器人状态数据
    #define ID_POWER_HEAT_DATA 0x0202           // 实时功率热量数据
    #define ID_ROBOT_POS 0x0203                 // 机器人位置数据
    #define ID_BUFF 0x0204                      // 机器人增益数据
    #define ID_AERIAL_ROBOT_ENERGY 0x0205       // 空中机器人能量状态数据
    #define ID_ROBOT_HURT 0x0206                // 伤害状态数据
    #define ID_SHOOT_DATA 0x0207                // 实时射击数据
    #define ID_BULLET_REMAINING 0x0208          // 子弹剩余发送数
    #define ID_RFID_STATUS 0x0209               // 机器人 RFID 状态
    #define ID_DART_CLIENT_CMD 0x020A           // 飞镖机器人客户端指令数据
    #define ID_GROUND_ROBOT_POSITION 0X020B     // 地面机器人位置数据
    #define ID_RARD_MRAK_DATA 0X020C            // 雷达标记进度数据
    #define ID_SENTRY 0X020D                    // 哨兵自主决策信息同步
    #define ID_RADAR 0x020E                     // 雷达自主决策信息同步
    // #define ID_STUDENT_INTERACTIVE_HEADER_DATA    0x0301       // 机器人交互信息
    #define ID_DIY_CONTROLLER 0x0302       // 自定义控制器
    #define ID_CLIENT_DOWMLOAD 0x0303      // 小地图下发位置信息
    #define ID_PICTURE_TRANSMISSION 0x0304 // 图传遥控信息
    #define ID_CLIENT_RECEIVE 0x0305       // 小地图接收位置信息

    /**************************** 裁判系统文档内容 裁判系统结构体 *********************/
#pragma pack(1) //按1字节对齐

    /*比赛状态数据*/ // 0x0001
    typedef __packed struct
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
        uint8_t error;
    } Game_Type_Data_t;

    /*比赛结果数据*/ // 0x0002
    typedef __packed struct
    {
        uint8_t winner;
        uint8_t error;
    } Game_Result_t;

    /*血量数据*/ // 0x0003
    typedef __packed struct
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t red_5_robot_HP;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t blue_5_robot_HP;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
        uint8_t error;
    } Robot_Hp_Data_t;

    /*场地事件数据*/ // 0x0101
    typedef __packed struct
    {
        //   uint32_t event_type;
        //   uint32_t BloodPoint_1 : 1;
        //   uint32_t BloodPoint_2 : 1;
        //   uint32_t BloodPoint_3 : 1;
        //   uint32_t ENERGY : 3;
        //   uint32_t Annular_HighLand : 1;
        //   uint32_t R3_or_B3_HighLand : 1;
        //   uint32_t R4_or_B4_HighLand : 1;
        uint32_t BloodPoint_us_front : 1;
        uint32_t BloodPoint_us_inside : 1;
        uint32_t BloodPoint_us : 1;
        uint32_t Buff_us : 1;
        uint32_t BuffSmall_us : 1;
        uint32_t BuffBig_us : 1;
        uint32_t Upland_Annular_us : 2;
        uint32_t Upland_Trapezium_us : 2;
        uint32_t Upland_Trapezium_us_ : 2;
        uint32_t Virtual_Sheild_us : 7;
        uint32_t Dart_hitPost_afterTime : 9;
        uint32_t Dart_AimType : 2;
        uint32_t BloodPoint_Mid : 2;
        uint8_t error;
    } Area_Data_t;

    /*补给站动作标识*/ // 0x0102
    typedef __packed struct
    {
        uint8_t supply_projectile_id;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
        uint8_t error;
    } Supply_Data_t;

    /*裁判警告信息*/ // 0x0104
    typedef __packed struct
    {
        uint8_t level;
        uint8_t foul_robot_id;
        uint8_t offend_conut;
        uint8_t error;
    } Referee_Warning_t;

    /*飞镖发射口数据*/ // 0x0105
    typedef __packed struct
    {
        uint8_t dart_remaining_time;
        uint16_t latest_aim : 2;
        uint16_t aim_count : 3;
        uint16_t aim_target : 2;
        uint16_t reserved : 9;
        uint8_t error;
    } Dart_Launch_Data_t;

    /*机器人状态数据*/ // 0x0201
    typedef __packed struct
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit; // 当前枪口热量上限
        uint16_t chassis_power_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
        uint8_t reserved : 5;
        uint8_t error;
    } Robot_Situation_Data_t;

    /*功率热量数据*/ // 0x0202
    typedef __packed struct
    {
        uint16_t chassis_volt;
        uint16_t chassis_current;
        float chassis_power;
        uint16_t chassis_power_buffer;          // 功率缓存
        uint16_t shooter_id1_17mm_cooling_heat; // 枪口热量
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
        uint8_t error;
    } Robot_Power_Heat_Data_t;

    /*机器人位置*/ // 0x0203
    typedef __packed struct
    {
        float x;
        float y;
        float angle;
        uint8_t error;
    } Robot_Position_Data_t;

    /*机器人增益数据*/ // 0x0204
    typedef __packed struct
    {
        uint8_t power_rune_buff;
        uint8_t error;
    } Area_Buff_Data_t;

    /*空中机器人能量状态*/ // 0x0205
    typedef __packed struct
    {
        uint8_t attack_time;
        uint8_t error;
    } UAV_Data_t;

    /*伤害状态*/ // 0x0206
    typedef __packed struct
    {
        uint8_t armor_id : 4;
        uint8_t hurt_type : 4;
        uint8_t error;
    } Robot_Hurt_Data_t;

    /*实时射击信息*/ // 0x0207
    typedef __packed struct
    {
        uint8_t bullet_type;
        uint8_t shooter_id;
        uint8_t bullet_freq;
        float bullet_speed; // 弹丸初速度
        uint8_t error;
    } Robot_Shoot_Data_t;

    /*子弹剩余发射数*/ // 0x0208
    typedef __packed struct
    {
        uint16_t bullet_remaining_num_17mm; // 17mm 子弹剩余发射数目
        uint16_t bullet_remaining_num_42mm; // 42mm 子弹剩余发射数目
        uint16_t coin_remaining_num;        // 剩余金币数量
        uint8_t error;
    } Robot_RaminingBullet_Data_t;

    /*RFID状态*/ // 0x0209
    typedef __packed struct
    {
        uint32_t BaseLand : 1;
        uint32_t HighLand_us : 1;
        uint32_t HighLand_enm : 1;
        uint32_t RorB3_us : 1;
        uint32_t RorB3_enm : 1;
        uint32_t RorB4_us : 1;
        uint32_t RorB4_enm : 1;
        uint32_t Buff : 1;
        uint32_t OverSlope_before_us : 1;
        uint32_t OverSlope_after_us : 1;
        uint32_t OverSlope_before_enm : 1;
        uint32_t OverSlope_after_enm : 1;
        uint32_t Outpost_us : 1;
        uint32_t BloodPooint_us : 1;
        uint32_t Patrol_area_us : 1;
        uint32_t Patrol_area_enm : 1;
        uint32_t Resource_area_us : 1;
        uint32_t Resource_area_enm : 1;
        uint32_t Exchange_area_us : 1;
        uint32_t CentralGain_Point : 1;
        uint32_t other : 12;
        uint8_t error;
    } RFID_Situation_Data_t;

    /*飞镖机器人客户端指令数据*/ // 0x020A
    typedef __packed struct
    {
        uint8_t dart_launch_opening_status;
        uint8_t dart_attack_target;
        uint16_t target_change_time;
        uint16_t operate_launch_cmd_time;
        uint8_t error;
    } Dart_Client_Cmd_t;

    /*机器人位置数据*/ //0x20B
    typedef __packed struct
    {
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float standard_5_x;
        float standard_5_y;
        uint8_t error;
    } ground_robot_position_t;

    /*雷达标记进度*/ //0x20C
    typedef __packed struct
    {
        uint8_t mark_hero_progress;
        uint8_t mark_engineer_progress;
        uint8_t mark_standard_3_progress;
        uint8_t mark_standard_4_progress;
        uint8_t mark_standard_5_progress;
        uint8_t mark_sentry_progress;
        uint8_t error;
    } radar_mark_data_t;

    /*哨兵自主决策信息同步*/ //0x20D
    typedef __packed struct
    {
        uint32_t sentry_bullet : 11;
        uint32_t sentry_bullet_count : 4;
        uint32_t sentry_blood_count : 4;
        uint32_t reserved : 13;
        uint8_t error;
    } sentry_info_t;

    /*雷达自主决策信息同步*/ //0x20E
    typedef __packed struct
    {
        uint8_t chance_double_vulnerability : 2;
        uint8_t IndoubleVulnerability:1;
        uint8_t other : 5;
        uint8_t error;
    } radar_info_t;

    /*自定义控制器数据*/ //0x302
    typedef __packed struct
    {
        uint8_t data[30];
        uint8_t error;
    } DIY_control_t;

    /*图传链路机器人向自定义控制器发送的数据*/ //0x309
    typedef __packed struct
    {
        uint8_t data[30];
        uint8_t error;
    } robot_custom_data_t;

    /*图传控制数据接收信息*/ //0x304
    typedef __packed struct
    {
        struct
        {
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t press_l;
            uint8_t press_r;
        } mouse;
        union
        {
            uint16_t key_code;
            struct
            {
                uint16_t W : 1;
                uint16_t S : 1;
                uint16_t A : 1;
                uint16_t D : 1;
                uint16_t SHIFT : 1;
                uint16_t CTRL : 1;
                uint16_t Q : 1;
                uint16_t E : 1;
                uint16_t R : 1;
                uint16_t F : 1;
                uint16_t G : 1;
                uint16_t M : 1; // uint16_t Z : 1;
                uint16_t N : 1; // uint16_t X : 1;
                uint16_t C : 1;
                uint16_t V : 1;
                uint16_t B : 1;
            } bit;
        } kb;
        uint16_t other;
        uint8_t error;
    } PHOTO_ctrl_t;

    /*客户端接受信息*/ //0x305
    // 雷达站发送的坐标信息可以被所有己方操作手在第一视角小地图看到。
    typedef __packed struct
    {
        uint16_t target_robot_ID;
        float target_position_x;
        float target_position_y;
        uint8_t error;
    } Client_Map_Command_Data_t;

    /*交互数据接收信息*/  // 0x306
    typedef __packed struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_ID;
        uint16_t receiver_ID;
        // uint16_t data[];
        uint8_t error;
    } student_interactive_header_data_t;

    /*学生机器人间通信*/
    // 未使用过，待测试
    // 可用于UI和车间通信
    typedef __packed struct
    {
        uint8_t data[1];
        uint8_t error;
    } robot_interactive_data_t;

    /*客户端下发信息*/
    typedef __packed struct
    {
        float target_position_x;
        float target_position_y;
        float target_position_z;
        uint8_t commd_keyboard;
        uint16_t target_robot_ID;
        uint8_t error;
    } Robot_Command_t;

    /*裁判系统数据*/
    typedef __packed struct
    {
        uint8_t RefereeData[256];
	    uint8_t RealData[45];
	    int16_t DataLen;
	    int16_t RealLen;
	    int16_t Cmd_ID;
	    uint8_t RECEIVE_FLAG;

        Game_Type_Data_t Game_Status;
        Game_Result_t Game_Result;
        Robot_Hp_Data_t Robot_HP;
        Area_Data_t Event_Data;
        Supply_Data_t Supply_Action;
        Referee_Warning_t Referee_Warning;
        Dart_Launch_Data_t Dart_Remaining_Time;
        Robot_Situation_Data_t Robot_Status;
        Robot_Power_Heat_Data_t Power_Heat;
        Robot_Position_Data_t Robot_Position;
        Area_Buff_Data_t Buff;
        UAV_Data_t Aerial_Energy;
        Robot_Hurt_Data_t Robot_Hurt;
        Robot_Shoot_Data_t Shoot_Data;
        Robot_RaminingBullet_Data_t Bullet_Num;
        RFID_Situation_Data_t RFID_Status;
        Dart_Client_Cmd_t Dart_Client;
        PHOTO_ctrl_t PHOTO_ctrl;
        student_interactive_header_data_t Interact_Header;
        robot_interactive_data_t Interact_Data;
        Robot_Command_t Client_Data;
        Client_Map_Command_Data_t ClientMapData;
        ground_robot_position_t Robot_Position_Al;
        radar_mark_data_t radar_mark;
        sentry_info_t sentry;
        radar_info_t radar_info;
        DIY_control_t DIY_control;
    } RefereeData_t;

    /* ----------------------- T13 接收数据结构体---------------------------- */
    typedef __packed struct
    {
        uint8_t sof_1;
        uint8_t sof_2;
        uint64_t ch_0 : 11;
        uint64_t ch_1 : 11;
        uint64_t ch_2 : 11;
        uint64_t ch_3 : 11;
        uint64_t mode_sw : 2;//接收端挡位切换开关位置：C:0, N:1, S:2
        uint64_t pause : 1;  //接收端暂停按键
        uint64_t fn_1 : 1;   //接收端自定义按键（左）
        uint64_t fn_2 : 1;   //接收端自定义按键（右）
        uint64_t wheel : 11; //接收端波轮位置
        uint64_t trigger : 1;//接收端扳机键

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t mouse_left : 2;
        uint8_t mouse_right : 2;
        uint8_t mouse_middle : 2;
        uint16_t key;
        uint16_t crc16;

        uint8_t data[256];
        int16_t dataLen;
        int16_t ch[5];
    } VT13data_t;
    /**************************** 裁判系统文档内容 UI用数据 *********************/
    // #define NULL 0
    // #define __FALSE 100
    // #define  Robot_ID UI_Data_RobotID_BStandard2
    // #define  Client_ID UI_Data_ClientID_RStandard2
    //
    //     /* ----------------------- 开始标志 ---------------------------- */
    // #define UI_SOF 0xA5
    //     /* ----------------------- CMD_ID数据 ---------------------------- */
    // #define UI_CMD_Robo_Exchange 0x0301
    //     /* ----------------------- 内容ID数据 ---------------------------- */
    // #define UI_Data_ID_Del 0x100
    // #define UI_Data_ID_Draw1 0x101
    // #define UI_Data_ID_Draw2 0x102
    // #define UI_Data_ID_Draw5 0x103
    // #define UI_Data_ID_Draw7 0x104
    // #define UI_Data_ID_DrawChar 0x110
    //     /* ----------------------- 红方机器人ID ---------------------------- */
    // #define UI_Data_RobotID_RHero 1
    // #define UI_Data_RobotID_REngineer 2
    // #define UI_Data_RobotID_RStandard1 3
    // #define UI_Data_RobotID_RStandard2 4
    // #define UI_Data_RobotID_RStandard3 5
    // #define UI_Data_RobotID_RAerial 6
    // #define UI_Data_RobotID_RSentry 7
    // #define UI_Data_RobotID_RRadar 9
    //     /* ----------------------- 蓝方机器人ID ---------------------------- */
    // #define UI_Data_RobotID_BHero 101
    // #define UI_Data_RobotID_BEngineer 102
    // #define UI_Data_RobotID_BStandard1 103
    // #define UI_Data_RobotID_BStandard2 104
    // #define UI_Data_RobotID_BStandard3 105
    // #define UI_Data_RobotID_BAerial 106
    // #define UI_Data_RobotID_BSentry 107
    // #define UI_Data_RobotID_BRadar 109
    //     /* ----------------------- 红方操作手ID ---------------------------- */
    // #define UI_Data_ClientID_RHero 0x0101
    // #define UI_Data_ClientID_REngineer 0x0102
    // #define UI_Data_ClientID_RStandard1 0x0103
    // #define UI_Data_ClientID_RStandard2 0x0104
    // #define UI_Data_ClientID_RStandard3 0x0105
    // #define UI_Data_ClientID_RAerial 0x0106
    //     /* ----------------------- 蓝方操作手ID ---------------------------- */
    // #define UI_Data_ClientID_BHero 0x0165
    // #define UI_Data_ClientID_BEngineer 0x0166
    // #define UI_Data_ClientID_BStandard1 0x0167
    // #define UI_Data_ClientID_BStandard2 0x0168
    // #define UI_Data_ClientID_BStandard3 0x0169
    // #define UI_Data_ClientID_BAerial 0x016A
    //     /* ----------------------- 删除操作 ---------------------------- */
    // #define UI_Data_Del_NoOperate 0
    // #define UI_Data_Del_Layer 1
    // #define UI_Data_Del_ALL 2
    //     /* ----------------------- 图形配置参数__图形操作 ---------------------------- */
    // #define UI_Graph_ADD 1
    // #define UI_Graph_Change 2
    // #define UI_Graph_Del 3
    //     /* ----------------------- 图形配置参数__图形类型 ---------------------------- */
    // #define UI_Graph_Line 0      //直线
    // #define UI_Graph_Rectangle 1 //矩形
    // #define UI_Graph_Circle 2    //整圆
    // #define UI_Graph_Ellipse 3   //椭圆
    // #define UI_Graph_Arc 4       //圆弧
    // #define UI_Graph_Float 5     //浮点型
    // #define UI_Graph_Int 6       //整形
    // #define UI_Graph_Char 7      //字符型
    //     /* ----------------------- 图形配置参数__图形颜色 ---------------------------- */
    // #define UI_Color_Main 0 //红蓝主色
    // #define UI_Color_Yellow 1
    // #define UI_Color_Green 2
    // #define UI_Color_Orange 3
    // #define UI_Color_Purplish_red 4 //紫红色
    // #define UI_Color_Pink 5
    // #define UI_Color_Cyan 6 //青色
    // #define UI_Color_Black 7
    // #define UI_Color_White 8
    // #define FRAMEHEAD_LEN 7
    // #define UI_DATA_OPERATE_LEN 6
    // #define GRAPH_DATA_LEN 15
    // #define STRING_DATA_LEN 45
    // #define ALL_GRAPH_LEN 30
    // #define ALL_STRING_LEN 60
    // #define ALL_SEVEN_GRAPH_LEN 120

    /**************************** 裁判系统文档内容 UI结构体 *********************/
    //
    // typedef struct {
    //     uint8_t SOF; //起始字节,固定0xA5
    //     uint16_t Data_Length; //帧数据长度
    //     uint8_t Seq; //包序号
    //     uint8_t CRC8; //CRC8校验值
    //     uint16_t CMD_ID; //命令ID
    // } UI_Packhead; //帧头
    //
    // typedef struct {
    //     uint16_t Data_ID; //内容ID
    //     uint16_t Sender_ID; //发送者ID
    //     uint16_t Receiver_ID; //接收者ID
    // } UI_Data_Operate; //操作定义帧
    //
    // typedef struct {
    //     uint8_t Delete_Operate; //删除操作
    //     uint8_t Layer; //删除图层
    // } UI_Data_Delete; //删除图层帧
    //
    // typedef struct {
    //     uint8_t graphic_name[3];
    //     uint32_t operate_tpye: 3;
    //     uint32_t graphic_tpye: 3;
    //     uint32_t layer: 4;
    //     uint32_t color: 4;
    //     uint32_t start_angle: 9;
    //     uint32_t end_angle: 9;
    //     uint32_t width: 10;
    //     uint32_t start_x: 11;
    //     uint32_t start_y: 11;
    //     int graph_Float; //浮点数据
    // } Float_Data;
    //
    // typedef struct {
    //     uint8_t graphic_name[3];
    //     uint32_t operate_tpye: 3;
    //     uint32_t graphic_tpye: 3;
    //     uint32_t layer: 4;
    //     uint32_t color: 4;
    //     uint32_t start_angle: 9;
    //     uint32_t end_angle: 9;
    //     uint32_t width: 10;
    //     uint32_t start_x: 11;
    //     uint32_t start_y: 11;
    //     uint32_t radius: 10;
    //     uint32_t end_x: 11;
    //     uint32_t end_y: 11; //图形数据
    // } Graph_Data; //
    //
    // typedef struct {
    //     Graph_Data Graph_Control;
    //     char show_Data[30];
    // } String_Data; //打印字符串数据

#pragma pack()

}

#endif
