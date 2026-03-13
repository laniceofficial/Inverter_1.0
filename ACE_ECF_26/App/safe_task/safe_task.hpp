#ifndef __TASK_SAFE_HPP  //如果未定义
#define __TASK_SAFE_HPP  //那么定义
#include <stdlib.h>
#include <stdint.h>
#include <functional>


using Callback = std::function<void(void)>;

class SafeTask_c
{
    public:
    char   name[50];                 //任务名称
    uint64_t  disconnection_ms;      //失联计数器
    uint64_t  disconnection_threshold;  //失联阈值
    bool   disconnection_falg;       //失联标志位
    bool   first_disconnect;         //首次失联标志位，防止多次进入失联回调函数
    // Safe_task_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack);
    SafeTask_c(const char* name, uint64_t Discon_ms, Callback DisconnetCallBack, Callback OnlineCallback);
    void Online(void);
    void CalcDisconnectionTime(void);
    void DoingDisconnetCallBack(void);
    SafeTask_c *next_task = nullptr;
    private:
    uint64_t last_online_time_ms = 0;
    Callback disconnet_callback;    //用户自定义失联回调函数
    Callback online_callback;

    // void (*DisconnetCallBack)(void);    //用户自定义失联回调函数
    // void (*OnlineCallBack)(void); 
};


#endif