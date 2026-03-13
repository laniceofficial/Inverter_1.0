/*************************** Dongguan-University of Technology -ACE**************************
 * @file    music_task.cpp
 * @author  L_Zero
 ******************************************************************************
 * @verbatim
 *  Cubemx上注册Music_Task任务(extern)，优先级低，栈大小128(没算过
 *  使用Play()或RePlay()类方法切换播放的音乐，Play_Note()可直接放音符(高优先级，会打断当前音乐
 *  使用is_playing()获取播放情况
 *  配置音乐在music_lib.hpp中
 * @attention
 *  需要Music_Init()后才能正常使用
 * @version -- @date
 *  v1.0 -- 2025/11/20
 *      基础的提醒、警告、错误、严重错误音，使能失能音，和一个示例音乐
 *  v2.0 -- 2025/11/23
 *      music_lib.hpp下添加预编译“曲谱规范模式”功能(MUSIC_STANDARD_MODE)
 *      开启该宏定义将要求MUSIC_TABLE[]元素索引必须与Music_PlayState_e的定义一一对应，
 *      但可以减少多曲谱下Music_Task()循环检索带来的性能消耗
 *  v3.0 -- 2025/11/23
 *      拓展Play、RePlay功能:
 *        现在可以在调用该函数时指定曲目的bpm、循环播放次数、播放结束等待时间,
 *        而不是只能按照写好的曲谱来
 *      曲谱使用命名空间，可以在其他文件写谱
 ************************** Dongguan-University of Technology -ACE***************************/

#include "music_task.hpp"

extern "C"{
#include <string.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
}

/* region ================================ 构造与初始化 ================================ */
Music_c* instance_ptr = nullptr; // 待注册单例指针

// 构造单例，并获取指针
Music_c* Music_c::Get_Instance()
{
    static Music_c instance;
    return &instance;
}

// 初始化，注册单例
void Music_Init(Music_c::Init_Config_t config)
{
    if(instance_ptr == nullptr)
    {
        instance_ptr = Music_c::Get_Instance();
        if(config.allow_dif < 1.0f)
        {
            instance_ptr->buzzer_ = new Buzzer_n::Buzzer_c(config.htim, config.channel, config.apb_mhz);
        }
        else
        {
            instance_ptr->buzzer_ = new Buzzer_n::Buzzer_c(config.htim, config.channel, config.apb_mhz, config.allow_dif);
        }
    }
}
// endregion



/* region ================================ 功能函数 ================================ */

// 外部调用，直接放音符
void Music_c::Play_Note(float note)
{
    current_state_ = Idle;
    next_state_ = Idle;
    buzzer_->Play(note);
}

// 外部调用，在next_state不等于当前状态时候会重播曲子
void Music_c::Play(Music_PlayState_e next_state)
{
    next_state_ = next_state;
    is_user_define_ = false;
    is_playing_ = true;
}
void Music_c::Play(Music_PlayState_e next_state, uint16_t bpm, uint8_t times, uint8_t delay_beat)
{
    next_state_ = next_state;
    bpm_ = bpm;
    times_ = times;
    delay_beat_ = delay_beat;
    is_user_define_ = true;
    is_playing_ = true;
}

// 外部调用，强制在下次Music_Task执行时重播
void Music_c::RePlay(Music_PlayState_e next_state)
{
    current_state_ = Idle;
    next_state_ = next_state;
    is_user_define_ = false;
    is_playing_ = true;
}
void Music_c::RePlay(Music_PlayState_e next_state, uint16_t bpm, uint8_t times, uint8_t delay_beat)
{
    current_state_ = Idle;
    next_state_ = next_state;
    bpm_ = bpm;
    times_ = times;
    delay_beat_ = delay_beat;
    is_user_define_ = true;
    is_playing_ = true;
}

/***
 * @brief 播放歌曲，循环调用
 * @param table 谱子指针，见music_lib.hpp
 * @param is_replay 是否重播，true将会重新执行谱子内容
 * */
void Music_c::Play_Song(const Music_Table_t *table, bool is_replay)
{
    static uint16_t index = 0;      // 曲谱索引
    static uint16_t cnt_beat = 0;    // 当前音剩余的拍数

    if(is_replay) // 重新演奏
    {
        buzzer_->Off();
        index = 0;
        cnt_beat = 0;
        if(!is_user_define_)
        {
            bpm_ = table->bpm;
            times_ = table->times;
            delay_beat_ = table->delay_beat;
        }
        is_playing_ = true;
        return;
    }

    if(!cnt_beat) // 没拍数了
    {
        if(table->song[index] > -0.1f) // 找下一个音
        {
            buzzer_->Play(table->song[index++]);
            cnt_beat = (uint8_t)(table->song[index++] + 0.1f); // 预防浮点精确问题，都加个0.1f
            cnt_beat--;
        }
        else // 一曲终了
        {
            if(times_ > 1) // 循环播放
            {
                times_--;
                index = 0;
                buzzer_->Play(table->song[index++]);
                cnt_beat = (uint8_t)(table->song[index++] + 0.1f);
                cnt_beat--;
            }
            else // 演奏结束
            {
                buzzer_->Off();
                if(!delay_beat_)
                {
                    is_playing_ = false;
                    if(table->song[index] < -10.1f) // 曲谱要求回Idle
                    {
                        Play(Idle);
                        instance_ptr->StateChange();
                    }
                }
                else
                {
                    delay_beat_--;
                }
            }
        }
    }
    else
    {
        cnt_beat--;
    }
}
// endregion



/* region ================================ 状态机 ================================ */

// 切状态
inline void Music_c::StateChange()
{
    if(next_state_ != current_state_)
    {
        StateExit();
        current_state_ = next_state_;
        StateStart();
    }
}

// 入状态
inline void Music_c::StateStart()
{
    if(current_state_ == Idle)
    {
        buzzer_->Off();
        is_playing_ = false;
    }
    else
    {
#ifdef MUSIC_STANDARD_MODE // 曲谱规范模式下取消检索减少性能消耗
        instance_ptr->Play_Song(&MUSIC_TABLE[current_state_], true);
#else
        for(uint8_t i = 0; MUSIC_TABLE[i].play_state != Idle; i++)
        {
            if(MUSIC_TABLE[i].play_state == instance_ptr->current_state_)
            {
                instance_ptr->Play_Song(&MUSIC_TABLE[i], true);
            }
        }
#endif
    }
}

// 出状态
inline void Music_c::StateExit()
{
    // no work to do
}
// endregion



/* region ================================ 执行任务 ================================ */

// 主任务
void Music_Task(void const *argument)
{
    while (1)
    {
        if (instance_ptr == nullptr) // 未初始化就挂机
        {
            // no work to do
            vTaskDelay(1000);
        }
        else
        {
            instance_ptr->StateChange();
            if(!instance_ptr->current_state_ == Idle)
            {
#ifdef MUSIC_STANDARD_MODE // 曲谱规范模式下取消检索减少性能消耗
                instance_ptr->Play_Song(&MUSIC_TABLE[instance_ptr->current_state_], false);
#else
                for(uint8_t i = 0; MUSIC_TABLE[i].play_state != Idle; i++)
                {
                    if(MUSIC_TABLE[i].play_state == instance_ptr->current_state_)
                    {
                        instance_ptr->Play_Song(&MUSIC_TABLE[i], false);
                    }
                }
#endif
            }
            else
            {
                instance_ptr->is_playing_ = false;
            }
            vTaskDelay(60 * 1000 / instance_ptr->bpm_);
        }
    }
}
// endregion
