/*************************** Dongguan-University of Technology -ACE**************************
 * @file    safe_task.c
 * @author  zhengNannnn
 * @version V1.0
 * @date    2023/11/5
 * @brief
 ******************************************************************************
 * @verbatim
 *  安全任务函数，支持用户自定义名称、失联检测时间、失联回调函数
 *  使用方法：
 *      申请一个安全任务，丢入名字及自定义失联回调函数
 *      类似与喂狗机制，在收到消息时刷新在线状态
 *      当达到失联阈值时执行自定义任务
 *  demo：
 *       //创建安全任务
 *       osThreadDef(SAFE_TASK, Safe_Task, osPriorityHigh, 0, 128);
 *		 Chassis_TASKHandle = osThreadCreate(osThread(Chassis_task), NULL);
 *       
 *       void online(void){enable米狗电机}
 *       void disconnect(void){unable米狗电机}
 *       //申请安全任务
 *       Safe_task_c demo_safe_task("demo",10,disconnect,online);
 *       demo_safe_task.online();
 * @attention
 *       好用爱用
 * @version           time
 * v1.0   基础版本
 * v2.0   C++升级版本  2024-8-27
 ************************** Dongguan-University of Technology -ACE***************************/
#include "safe_task.hpp"
#include "bsp_dwt.hpp"

extern "C"{
    #include "safe_task.h"
    #include <string.h>
    #include <stdint.h>
    #include "FreeRTOS.h"
    #include "task.h"
    #include "main.h"
}
SafeTask_c* Head = nullptr;

/*任务间采用链表*/
// static bool dwt_init = false;
SafeTask_c::SafeTask_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback)
{
    SafeTask_c* last_task_ptr = Head;
    //写入相关参数
    strcpy(this->name, name);
    this->disconnection_threshold = Discon_ms;
    this->disconnection_ms = 0;
    this->disconnection_falg = false;
    this->first_disconnect = true;
    this->online_callback = OnlineCallback;
    this->disconnet_callback = DisconnetCallBack;
    this->last_online_time_ms = 0;
    this->next_task = nullptr;
    //链表尾插法
    if(Head == nullptr)
    {
        Head = this;
        return;
    }        
    while(last_task_ptr->next_task != nullptr)     last_task_ptr = last_task_ptr->next_task;
    last_task_ptr->next_task = this;
}
/**
 * @brief 刷新在线状态
 * @note  若有注册在线回调，则会运行
 */
void SafeTask_c::Online(void)
{
    BSP_n::DWT_c *dwt = BSP_n::DWT_c::Get_DwtInstance();
    if (this->online_callback != nullptr)    this->online_callback();
    this->last_online_time_ms = dwt->GetTimeline_ms();
    this->disconnection_ms = 0;
    this->disconnection_falg = false;
    this->first_disconnect = true;
}
/**
 * @brief 失联时间计算
 */
void SafeTask_c::CalcDisconnectionTime(void)
{
    BSP_n::DWT_c *dwt = BSP_n::DWT_c::Get_DwtInstance();
    this->disconnection_ms = dwt->GetTimeline_ms()-this->last_online_time_ms;
    if (this->disconnection_ms < 0) this->disconnection_ms = 0;
    if (this->disconnection_ms > this->disconnection_threshold) this->disconnection_falg = true;
}
/**
 * @brief 若有掉线失联，执行失联函数
 * @note  失联函数仅执行一次
 */
void SafeTask_c::DoingDisconnetCallBack(void)
{
    if (this->disconnection_falg)   
    {
        if (this->first_disconnect) this->disconnet_callback();
        this->first_disconnect = false;
    }
}

/**
 * @brief 丢到freertos任务
 */
void Safe_Task(void const *argument)
{
	while(1)
	{
        //链表轮询
        SafeTask_c* temp_task_ptr = Head;
        while(temp_task_ptr != nullptr)
        {
            temp_task_ptr->CalcDisconnectionTime();
            temp_task_ptr->DoingDisconnetCallBack();
            temp_task_ptr = temp_task_ptr->next_task;
        }
		vTaskDelay(1);
	}
}

