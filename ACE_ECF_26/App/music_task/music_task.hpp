#ifndef __MUSIC_HPP
#define __MUSIC_HPP

#include "buzzer.hpp"
#include "music_task.h"
#include "music_lib.hpp"

class Music_c
{
public:
    /* type define */
    typedef struct {
        TIM_HandleTypeDef *htim = &htim17;
        uint32_t channel = TIM_CHANNEL_1;
        uint16_t apb_mhz = 170;  // 一般H7是240mhz
        float allow_dif = 1.0f;  // 允许设定音高与实际音高的频率差，尽量大于1.0f，越大音越不准但是执行速度越快
    } Init_Config_t;

    /* function */
    void Play_Note(float note);
    void Play(Music_PlayState_e next_state);
    void Play(Music_PlayState_e next_state, uint16_t bpm, uint8_t times, uint8_t delay_beat);
    void RePlay(Music_PlayState_e next_state);
    void RePlay(Music_PlayState_e next_state, uint16_t bpm, uint8_t times, uint8_t delay_beat);
    bool is_playing(void){return is_playing_;}

    static Music_c *Get_Instance(void);

    /* friend */
    friend void Music_Task(void const *argument);
    friend void Music_Init(Music_c::Init_Config_t config);

private:
    Music_c() = default;

    ~Music_c() = default;

    /* use instance */
    Buzzer_n::Buzzer_c *buzzer_ = nullptr;

    /* data */
    Music_PlayState_e current_state_ = Idle;
    Music_PlayState_e next_state_ = Idle;
    uint16_t bpm_ = 60;
    uint8_t times_ = 1;      // 当前曲子剩余的循环播放次数
    uint8_t delay_beat_ = 0; // 曲子完全播放完后等待

    /* flag */
    bool is_playing_ = false;
    bool is_user_define_ = false;

    /* function */
    void Play_Song(const Music_Table_t *song, bool is_replay);

    void StateChange(void);
    void StateStart(void);
    void StateExit(void);

};
void Music_Init(Music_c::Init_Config_t config);

#endif // !__MUSIC_HPP
