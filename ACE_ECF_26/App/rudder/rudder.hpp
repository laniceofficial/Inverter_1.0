#ifndef RUDDER_HPP
#define RUDDER_HPP
#include "servo.hpp"
#include "Alg_PID.hpp"
#include "BMI088driver.hpp"
// 从镖头垂直与纸面向外，左上1，右上2，右下3，左下4
// 设俯仰舵偏δp（正值为抬头）、偏航舵偏δy（正值为向右偏航）、滚转舵偏δr（正值为右滚转）。
//-90~90舵偏
#define MIM_ANGLE 10.f
#define MIDDLE_ANGLE 135.f
#define MAX_ANGLE 260.f
class Rudder_c
{
public:
    Rudder_c() {}
    Rudder_c(const INS_t *ins)
    {
        Init(ins);
    }
    void CalDelta(float delta_yaw, float delta_pitch, float delta_roll);
    void SetDelta();
    void Init(const INS_t *ins);
    void Calibrate();
    void Calculate(float yaw_cmd, float pitch_cmd, float roll_cmd);
    void RollStableOnly();
    void PitchCal(float pitch_cmd);
    void YawCal(float yaw_cmd);
    void RollCal(float roll_cmd);
    alg_n::PID_c PID_rate_y;
    alg_n::PID_c PID_pos_y;
    alg_n::PID_c PID_rate_p;
    alg_n::PID_c PID_pos_p;
    alg_n::PID_c PID_rate_r;
    alg_n::PID_c PID_pos_r;

private:
    void YawControllerInit(const float *rate, const float *pos);
    void PitchControllerInit(const float *rate, const float *pos);
    void RollControllerInit(const float *rate, const float *pos);

    float pcmd = 0;
    float rcmd = 0;
    float ycmd = 0;
    float delta1 = 0;
    float delta2 = 0;
    float delta3 = 0;
    float delta4 = 0;
    float angle1 = 0;
    float angle2 = 0;
    float angle3 = 0;
    float angle4 = 0;
    Servo_n::Servo_c *wing[4] = {nullptr};
};

#endif // !RUDDER_HPP
