
/*************************** Dongguan-University of Technology -ACE**************************
 * @file    cybergear.c
 * @author  zhengNannnn & huwei
 * @version V2.0
 * @date    2023/10/1   v1.0
 *          2024/12/1   v2.0
 * @brief
 ******************************************************************************
 * @verbatim
 *  米狗电机驱动库,支持自定义选择can邮箱,支持多电机同时使用
 *  使用方法：
 *      先创建一个米狗电机结构体
 *      初始化：
 *            创建对象，设置can网络和主机ID，电机ID （主机ID默认设置0）
 *            使用MI_motor_get_ID(MI_Motor);//获取电机id值
 *            使用changeID(MI_Motor,num);//更改ID
 *      使用：多种运行模式,具体流程参考用户手册
 *  demo：
 *      MI_motor = new cybergear_motor_c(&hcan1,0,127);
        while(1)
        {
          MI_motor->MI_motor_controlmode(0.5f,0.0f,0.0f,0,0); //运控模式
        }
 * @attention
 *      电机如果掉电重启需要重新enable，不然会导致电机无法使用，这一步在四种控制模式中解决了
 *      如果使用非运控模式，需要根据说明书按特定顺序写入参数，这里已经封装在对应的函数中，使用的适合只需要调用一个函数即可
 * @version
 * v1.0   基础版本
 * v1.1   添加接收处理函数
 * v2.0   C++升级版本(支持多个电机使用，对四个控制模式进行封装，方便调用)
 ************************** Dongguan-University of Technology -ACE***************************/
#include "Cybergear_motor.hpp"


using namespace cybergear_n;

cybergear_motor_c* cybergear_motor_c::MI_motor_instance_head = nullptr;
uint8_t cybergear_motor_c::MI_num_ = 0;

//函数声明
void MI_motor_recive_callback(BSP_CAN_Part_n::CANInstance_c* register_instance);

//共用体float转型uint8_t
union FloatAndBytes_t {
    float floatValue;
    uint8_t byteValue[4];
}FloatAndBytes;

bool get_can_id = false;
uint8_t MI_id = 0;
/**
  * @brief         浮点数转4字节
  * @param         浮点数
  * @return        4字节数组
  * @description   IEEE 754 协议
  */
uint8_t byte[4];
uint8_t* Float_to_Byte(float f)
{
	unsigned long longdata = 0;
	longdata = *(unsigned long*)&f;       
	byte[0] = (longdata & 0xFF000000) >> 24;
	byte[1] = (longdata & 0x00FF0000) >> 16;
	byte[2] = (longdata & 0x0000FF00) >> 8;
	byte[3] = (longdata & 0x000000FF);
	return byte;
}
/**
  * @brief         4字节转浮点数
  * @param         4字节数组
  * @return        浮点数
  * @description   IEEE 754 协议
  */
float Byte_to_Float(uint8_t* byte)
{
    unsigned long longdata = 0;

    // 将字节按位组装成一个无符号长整数
    longdata |= (unsigned long)byte[0] << 24;
    longdata |= (unsigned long)byte[1] << 16;
    longdata |= (unsigned long)byte[2] << 8;
    longdata |= (unsigned long)byte[3];

    // 将长整数重新解释为浮点数
    float result = *(float*)&longdata;

    return result;
}

/**
  * @brief  float转int，数据打包用
  * @param  x float数值
  * @param  x_min float数值的最小值
  * @param  x_max float数值的最大值
  * @param  bits  int的数据位数
  * @retval null
  */
int float_to_uint(float x, float x_min, float x_max, int bits) 
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief  小米电机回文16位数据转浮点
  * @param  16位回文 
  * @param  对应参数下限 
  * @param  对应参数上限 
  * @param  参数位数
  * @return 参数对应浮点数
  */
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}


//小米电机构造函数
cybergear_motor_c::cybergear_motor_c(CAN_HandleTypeDef *phcan,uint8_t master_id,uint8_t motor_id):motor_can_ins_(phcan,master_id,motor_id,CAN_ID_EXT,MI_motor_recive_callback)
{
    this->master_id = master_id;
    this->motor_id = motor_id;
    this->next_ = nullptr;
    //记录下数量
    this->MI_num_++;
    this->MI_motor_setMechPosition2Zero();

    if(cybergear_motor_c::MI_motor_instance_head == nullptr)
    {
        cybergear_motor_c::MI_motor_instance_head = this;
        return;
    }

    //链表尾插法
    cybergear_motor_c *temp = cybergear_motor_c::MI_motor_instance_head;
    while(temp->next_!= nullptr)
    {
        temp = temp->next_;
    }
    temp->next_ = this;
}


/**
  * @brief  小米电机使能
  * @param  hmotor 电机结构体
  * @param  id 电机id
  * @retval null
  */
void cybergear_motor_c::MI_motor_enable()
{
    this->txMsg.ExtId.mode = 3;
    this->txMsg.ExtId.motor_id = this->motor_id; //不再重复设置
    this->txMsg.ExtId.data = this->master_id;
    this->txMsg.ExtId.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }
    
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  获取设备ID （通信类型0），首先你要知道电机ID才能使用这个函数，emmm，还是得通过上位机设定id（默认ID为127）
  * @retval null 
  */
void cybergear_motor_c::MI_motor_get_ID()
{
    this->txMsg.ExtId.mode = 0;
    this->txMsg.ExtId.data = 0;
    this->txMsg.ExtId.motor_id = 0;
    this->txMsg.ExtId.res = 0;
    
    for(uint8_t i=0; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }

    
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
    while(get_can_id);//等待can中断接收到电机目前id
    this->motor_id = MI_id;
    
}

/**
  * @brief  运控模式电机控制指令（通信类型1）
  * @param  hmotor 电机结构体
  * @param  torque 力矩
  * @param  MechPosition 目标位置
  * @param  speed 转速
  * @param  kp 
  * @param  kd 
  * @retval null
  */
void cybergear_motor_c::MI_motor_controlmode(float torque, float MechPosition , float speed , float kp , float kd)
{
    //断电自启动
    static int count = 0;
    if(count == 0)
        MI_motor_enable();
    count++;
    if(count > MAX_COUNT)
        count = 0;
    
    this->current_mode = cybergear_n::control;
    this->txMsg.ExtId.mode = 1;
    this->txMsg.ExtId.data = float_to_uint(torque,T_MIN,T_MAX,16);
    this->txMsg.ExtId.res = 0;
 
    this->txMsg.Data[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    this->txMsg.Data[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    this->txMsg.Data[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    this->txMsg.Data[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    this->txMsg.Data[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    this->txMsg.Data[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    this->txMsg.Data[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    this->txMsg.Data[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);

    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  电机停止运行帧（通信类型4）
  * @param  hmotor 电机结构体
  * @retval null
  */
void cybergear_motor_c::MI_motor_stop()
{
    this->txMsg.ExtId.mode = 4;
    this->txMsg.ExtId.data = this->master_id;
    this->txMsg.ExtId.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }

    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param  hmotor 电机结构体
  * @retval null
  */
void cybergear_motor_c::MI_motor_setMechPosition2Zero()
{
    this->txMsg.ExtId.mode = 6;
    this->txMsg.ExtId.data = MASTER_ID;
    this->txMsg.ExtId.res = 0;
    this->txMsg.Data[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param  MI_motor 电机结构体
  * @param  Target_ID 想要改成的电机ID
  * @retval null
  */
void cybergear_motor_c::MI_motor_changeID(uint8_t Target_ID)
{
    this->txMsg.ExtId.mode = 7;	
    this->txMsg.ExtId.motor_id = this->motor_id;
    this->txMsg.ExtId.data = Target_ID << 8 | this->master_id;
    this->txMsg.ExtId.res = 0;
    
    this->motor_id = Target_ID;//更改本电机canID
 
    for(uint8_t i=0; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  单个参数读取（通信类型17）
  * @param  MI_motor 电机结构体指针
  * @param  parameter_index_e 读取的参数
  * @retval null
  * @note   目前反馈接收存在小bug
  */
void cybergear_motor_c::MI_motor_Read_One_Para( parameter_index_e index)
{
    this->txMsg.ExtId.mode = 17;
    this->txMsg.ExtId.data = MASTER_ID;
    this->txMsg.ExtId.res = 0;
    this->txMsg.Data[0]=index;
    memcpy(&this->txMsg.Data[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        this->txMsg.Data[i]=0;
    }
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @brief  单个参数写入（通信类型18） （掉电丢失）
  * @param  电机结构体指针
  * @param  写入的参数
  * @param  写入的值
  * @retval null
  * @note   电流 位置 速度模式第一次写入无反应？
  */
void cybergear_motor_c::MI_motor_Write_One_Para(parameter_index_e index , float Value)
{
    this->txMsg.ExtId.mode = 18;
    this->txMsg.ExtId.motor_id = this->motor_id;
    this->txMsg.ExtId.data = this->master_id;
    this->txMsg.ExtId.res = 0;
    for(uint8_t i=2; i<8; i++)
    {
        this->txMsg.Data[i] = 0;
    }

    memcpy(&this->txMsg.Data[0],&index,2);

    if (index == run_mode)
    {
        this->txMsg.Data[4]=(uint8_t)Value;
		this->txMsg.Data[5]=0x00;
		this->txMsg.Data[6]=0x00;
		this->txMsg.Data[7]=0x00;	
    }
    else
    {
		Float_to_Byte(Value);
		this->txMsg.Data[4]=byte[3];
		this->txMsg.Data[5]=byte[2];
		this->txMsg.Data[6]=byte[1];
		this->txMsg.Data[7]=byte[0];		
	}
    this->motor_can_ins_.Chance_Ext_Tx_ID(*((uint32_t*)&(this->txMsg.ExtId)));
    for(int i = 0;i < 8;i++)
    {
        this->motor_can_ins_.tx_buff[i] = this->txMsg.Data[i];
    }
    this->motor_can_ins_.ECF_Transmit(1);
}

/**
  * @function     : 设置电机控制模式
  * @param        : 电机控制模式
  * @return       : null
  */
void cybergear_motor_c::Set_Mode(Mode_rum_mode_t Mode)
{
	MI_motor_Write_One_Para(run_mode,Mode);
}

//速度模式控制指令
void cybergear_motor_c::MI_Speedmode_SetSpeed(float set_speed)
{
    //断电自启动
    static int count = 0;

    if(this->current_mode != cybergear_n::speed || count == 0)
    {
        Set_Mode(cybergear_n::speed);
        MI_motor_enable();
        MI_motor_Write_One_Para(limit_cur,23);
        this->current_mode = cybergear_n::speed;
    }
    count ++;
    if(count > MAX_COUNT) count = 0;
    MI_motor_Write_One_Para(spd_ref,set_speed);
}

//位置模式控制指令
void cybergear_motor_c::MI_Postionmode_SetPosition(float set_position)
{
    //断电自启动
    static int count = 0;

    if(this->current_mode != cybergear_n::postion || count == 0)
    {
        Set_Mode(cybergear_n::postion);
        MI_motor_enable();
        MI_motor_Write_One_Para(limit_spd,5);  //最大速度，可自己另外设置
        this->current_mode = cybergear_n::postion;
    }
    count ++;
    if(count > MAX_COUNT) count = 0;
    MI_motor_Write_One_Para(loc_ref,set_position);
}

//电流模式控制指令
void cybergear_motor_c::MI_Currentmode_SetCurrent(float set_current)
{
    //断电自启动
    static int count = 0;

    if(this->current_mode != cybergear_n::current || count == 0)
    {
        Set_Mode(cybergear_n::current);
        MI_motor_enable();
        this->current_mode = cybergear_n::current;
    }
    count ++;
    if(count > MAX_COUNT) count = 0;
    MI_motor_Write_One_Para(iq_ref,set_current);
}


rxExtId_t rx_extid; //接收的扩展帧
    uint8_t* Rx_Data = NULL;
/**
  * @brief  小米电机接收处理函数
  * @retval null
  * @note   判断的数据结构体内本电机id需要自己手动设置
  */
void MI_motor_recive_callback(BSP_CAN_Part_n::CANInstance_c* register_instance)
{
    cybergear_motor_c* p = cybergear_motor_c::MI_motor_instance_head;

    rx_extid = *(rxExtId_t*)&register_instance->rx_id_; //接收的扩展帧
    Rx_Data = register_instance->rx_buff;  //接收的数据帧

    for(;p!=nullptr;p = p->next_)
    {
        cybergear_motor_c* rx_MI_motor = p;

        if (rx_extid.motor_id != rx_MI_motor->master_id || rx_extid.motor_now_id != rx_MI_motor->motor_id)  //非本主机或非本电机
        {
            continue;
        }

        switch (rx_extid.mode)
        {
            case 0:{//获取设备的ID和64位MCU唯一标识符(通信类型0)
                if (rx_extid.motor_id == 0xFE){
                    get_can_id = true;//MI_motor_get_ID用得到
                    MI_id = rx_extid.motor_id;
                };     //后面的是64位MCU唯一标识符,暂不添加（没用）
                break;
            }
            case 2:{//电机反馈数据帧,用来向主机反馈电机运行状态(通信类型2)
                
                //拓展帧bit16~23信息
                rx_MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t)rx_extid.Mode_status;
                rx_MI_motor->motor_error_list.Encoder_not_calibrated_fault = rx_extid.Uncalibrated;
                rx_MI_motor->motor_error_list.HALL_encoding_fault = rx_extid.HALL_encoding_failt;
                rx_MI_motor->motor_error_list.Magnetic_encoding_fault = rx_extid.Magnetic_encoding_fault;
                rx_MI_motor->motor_error_list.Over_temperature_fault = rx_extid.Over_temperature;
                rx_MI_motor->motor_error_list.Over_current_fault = rx_extid.Over_current;
                rx_MI_motor->motor_error_list.Under_voltage_fault = rx_extid.Under_voltage_fault;
                
                rx_MI_motor->postion=uint16_to_float(Rx_Data[0]<<8|Rx_Data[1],MIN_P,MAX_P,16);

                rx_MI_motor->speed=uint16_to_float(Rx_Data[2]<<8|Rx_Data[3],MIN_S,MAX_S,16);			

                rx_MI_motor->torque=uint16_to_float(Rx_Data[4]<<8|Rx_Data[5],MIN_T,MAX_T,16);				

                rx_MI_motor->temperture=(Rx_Data[6]<<8|Rx_Data[7])*Temp_Gain;

                break;
            }
            case 17:{//发送读取单个参数的应答帧,根据index参数下标判断回传参数类型(通信类型17)
                parameter_index_e index = (parameter_index_e)(Rx_Data[1]<<8| Rx_Data[0]);//

                //共用体4个u8转float
                FloatAndBytes.byteValue[0] = Rx_Data[4];
                FloatAndBytes.byteValue[1] = Rx_Data[5];
                FloatAndBytes.byteValue[2] = Rx_Data[6];
                FloatAndBytes.byteValue[3] = Rx_Data[7];

                switch (index){
                    case run_mode:
                        rx_MI_motor->motor_parameter_list.Mode_rum_mode = (Mode_rum_mode_t) Rx_Data[4];//
                        break;
                    case iq_ref:
                        rx_MI_motor->motor_parameter_list.iq_ref = FloatAndBytes.floatValue;
                        break;
                    case spd_ref:
                        rx_MI_motor->motor_parameter_list.spd_ref = FloatAndBytes.floatValue;
                        break;
                    case imit_torque:
                        rx_MI_motor->motor_parameter_list.imit_torque = FloatAndBytes.floatValue;
                        break;
                    case cur_kp:
                        rx_MI_motor->motor_parameter_list.cur_kp = FloatAndBytes.floatValue;
                        break;
                    case cur_ki:
                        rx_MI_motor->motor_parameter_list.cur_ki = FloatAndBytes.floatValue;
                        break;
                    case cur_filt_gain:
                        rx_MI_motor->motor_parameter_list.cur_filt_gain = FloatAndBytes.floatValue;
                        break;
                    case loc_ref:
                        rx_MI_motor->motor_parameter_list.loc_ref = FloatAndBytes.floatValue;
                        break;
                    case limit_spd:
                        rx_MI_motor->motor_parameter_list.limit_spd = FloatAndBytes.floatValue;
                        break;
                    case limit_cur:
                        rx_MI_motor->motor_parameter_list.limit_cur = FloatAndBytes.floatValue;
                        break;
                    default :
                        break;
                }
            }
            case 21:{//故障反馈帧
                //Byte0
                rx_MI_motor->motor_error_list.Over_temperature_fault = Rx_Data[0] & 0x01;//取bit0
                rx_MI_motor->motor_error_list.Drive_core_fault = (Rx_Data[0]>>1) & 0x01;//取bit1
                rx_MI_motor->motor_error_list.Under_voltage_fault = (Rx_Data[0]>>2) & 0x01;//取bit2
                rx_MI_motor->motor_error_list.Over_voltage_fault =  (Rx_Data[0]>>3) & 0x01;//取bit3
                rx_MI_motor->motor_error_list.B_phase_current_sampling_overcurrent = (Rx_Data[0]>>4) & 0x01;//取bit4
                rx_MI_motor->motor_error_list.C_phase_current_sampling_overcurrent = (Rx_Data[0]>>5) & 0x01;//取bit5
                rx_MI_motor->motor_error_list.Magnetic_encoding_fault = (Rx_Data[0]>>7) ;//取bit7
                
                uint16_t Byte1_2 = ((uint16_t)Rx_Data[1] << 8) | Rx_Data[2];//取bit8~bit15
                if (Byte1_2 != 0)   rx_MI_motor->motor_error_list.Overload_fault = true;
                else                rx_MI_motor->motor_error_list.Overload_fault = false;
                
                rx_MI_motor->motor_error_list.A_phase_current_sampling_overcurrent = Rx_Data[3] & 0x01;//取bit16
            }
        }//switch(mode)

    }//for
    
}


