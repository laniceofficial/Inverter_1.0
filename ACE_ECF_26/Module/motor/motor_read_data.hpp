/************************** Dongguan-University of Technology -ACE**************************
* @file  motor_read_data.hpp
* @brief 读取电机参数（或者其他有关电机参数）
* @author 叶智明
* @version 1.0
* @date 2025-10-12
* @note 写代码的时候关键的变量尽量都不要用指针，不然jlink观测太多会卡死，平时直接读取一个instance时，其中数据量过多
        因此特别创建文件来读取电机数据和配置，也可以进行数据的计算
        此文件就包含了摩擦轮调试数据
        (后面可以进化到read_data.hpp吼)
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/

#ifndef __MOTOR_READ_DATA_HPP
#define __MOTOR_READ_DATA_HPP
#include <cstdint>
#include "dji_driver.hpp"
#include "dm_driver.hpp"
#include "RC.hpp"

namespace Motor_n {
    /**
     * @brief 电机数据读取
     */
    struct MotorData_t {
        // 电机原始反馈值结构体
        struct MotorRawFeedback_t
        {
            float feedback_speed;
            float last_feedback_speed; //上次速度
            int16_t last_ecd;          //dji专用
            int16_t feedback_ecd;      //dji专用
            float position;
            float force_feedback;   //电流反馈和力矩反馈通用
            float temperature[2];
            uint8_t error;          //错误码
        } motor_raw_data;

        // 处理后的电机数据结构体
        struct MotorProcessedFeedback_t
        {
            float speed;          // 转速
            float speed_difference;// 转速差
            float torque;         // 力矩
            float relative_angle; // 单圈相对角度
            float absolute_angle; // 单圈绝对角度
            float total_ecd;      // 总编码值
            float total_angle;    // 总角度
            float total_round;    // 总圈数
            // 工程平移关节用
            float linear_speed;        // 线速度
            float linear_displacement; // 线位移
            // 加速度？感觉暂时不用了
        } motor_processed_data;

        void ReadDjiData(DjiMotor_n::DjiDriver_c *dji_motor) {
            this->motor_raw_data.feedback_speed = dji_motor->motor_data_.motor_raw_data.feedback_speed;
            this->motor_raw_data.last_feedback_speed = dji_motor->motor_data_.motor_raw_data.last_feedback_speed;
            this->motor_raw_data.last_ecd = dji_motor->motor_data_.motor_raw_data.last_ecd;
            this->motor_raw_data.feedback_ecd = dji_motor->motor_data_.motor_raw_data.feedback_ecd;
            this->motor_raw_data.force_feedback = dji_motor->motor_data_.motor_raw_data.force_feedback;
            this->motor_raw_data.temperature[0] = dji_motor->motor_data_.motor_raw_data.temperature[0];

            this->motor_processed_data.absolute_angle = dji_motor->motor_data_.motor_processed_data.absolute_angle;
            this->motor_processed_data.relative_angle = dji_motor->motor_data_.motor_processed_data.relative_angle;
            this->motor_processed_data.speed = dji_motor->motor_data_.motor_processed_data.speed;
            this->motor_processed_data.torque = dji_motor->motor_data_.motor_processed_data.torque;
            this->motor_processed_data.total_ecd = dji_motor->motor_data_.motor_processed_data.total_ecd;
            this->motor_processed_data.total_angle = dji_motor->motor_data_.motor_processed_data.total_angle;
            this->motor_processed_data.total_round = dji_motor->motor_data_.motor_processed_data.total_round;
            this->motor_processed_data.linear_speed = dji_motor->motor_data_.motor_processed_data.linear_speed;
        }

        void ReadDmData(DmMotor_n::DmDriver_c *dm_motor) {
            this->motor_raw_data.feedback_speed = dm_motor->motor_data_.motor_raw_data.feedback_speed;
            this->motor_raw_data.position = dm_motor->motor_data_.motor_raw_data.position;
            this->motor_raw_data.force_feedback = dm_motor->motor_data_.motor_raw_data.force_feedback;
            this->motor_raw_data.temperature[0] = dm_motor->motor_data_.motor_raw_data.temperature[0];
            this->motor_raw_data.temperature[1] = dm_motor->motor_data_.motor_raw_data.temperature[1];
            this->motor_raw_data.error = dm_motor->motor_data_.motor_raw_data.error;

            this->motor_processed_data.speed = dm_motor->motor_data_.motor_processed_data.speed;
            this->motor_processed_data.torque = dm_motor->motor_data_.motor_processed_data.torque;
            this->motor_processed_data.absolute_angle = dm_motor->motor_data_.motor_processed_data.absolute_angle;
            this->motor_processed_data.relative_angle = dm_motor->motor_data_.motor_processed_data.relative_angle;
            this->motor_processed_data.total_angle = dm_motor->motor_data_.motor_processed_data.total_angle;
            this->motor_processed_data.total_round = dm_motor->motor_data_.motor_processed_data.total_round;
        }
    };

    /**
     * @brief 电机配置读取（部分配置在初始化里面的一般不会变的就没有获取，可以自行补充）
     */
    struct MotorConfig_t {
        MotorBaseDef_n::Motor_Type_euc motor_type;
        char motor_name[20];
        uint32_t motor_id;
        MotorBaseDef_n::Motor_Driver_Way_eb motor_driver_way;
        bool motor_init_flag = false;              // 初始化标志位
        bool motor_online_flag = false;            // 在线标志位
        float motor_max_output, motor_min_output; // 输出限幅
        float dt_, working_time;                   // 反馈时间间隔和持续运行时间
        float pid_output;
        MotorBaseDef_n::Motor_Controller_t *motor_controller;

        void ReadDjiConfig(DjiMotor_n::DjiDriver_c *dji_motor) {
            this->motor_type = dji_motor->motor_type_;
            strcpy(this->motor_name, dji_motor->motor_name_);
            this->motor_id = dji_motor->motor_id_;
            this->motor_driver_way = dji_motor->motor_driver_way_;
            this->motor_init_flag = dji_motor->motor_init_flag_;
            this->motor_online_flag = dji_motor->motor_online_flag_;
            this->motor_max_output = dji_motor->motor_max_output_;
            this->motor_min_output = dji_motor->motor_min_output_;
            this->dt_ = dji_motor->dt_;
            this->working_time = dji_motor->working_time_;
            this->pid_output = dji_motor->motor_controller_.pid_output;
        }

        void ReadDmConfig(DmMotor_n::DmDriver_c *dm_motor) {
            this->motor_type = dm_motor->motor_type_;
            strcpy(this->motor_name, dm_motor->motor_name_);
            this->motor_id = dm_motor->motor_id_;
            this->motor_driver_way = dm_motor->motor_driver_way_;
            this->motor_init_flag = dm_motor->motor_init_flag_;
            this->motor_online_flag = dm_motor->motor_online_flag_;
            this->motor_max_output = dm_motor->motor_max_output_;
            this->motor_min_output = dm_motor->motor_min_output_;
            this->dt_ = dm_motor->dt_;
            this->working_time = dm_motor->working_time_;
            this->pid_output = dm_motor->motor_controller_.pid_output;
        }
    };

/*************************** 环形缓冲区 *******************************************************/
    #define SHOOT_AVERAGE_CNT 80
    struct UBF_SpeedBuffer_t{
        float buf[SHOOT_AVERAGE_CNT]; // 存放数据
        uint16_t head=0;                // 下一个写入位置 (0..SHOOT_AVERAGE_CNT-1)
        uint16_t cnt=0;                 // 当前有效元素个数 (<= SHOOT_AVERAGE_CNT)
    };

    static inline void UbfInit(UBF_SpeedBuffer_t *ubf)
    {
        if (!ubf) return;
        ubf->head = 0;
        ubf->cnt = 0;
    }

    /**
     * @brief 将一个 float 数据压入环形缓冲（覆盖最旧项）。
     * @param ubf  缓冲指针
     * @param data 指向 float 的 void*
     */
    inline void UbfPush(UBF_SpeedBuffer_t *ubf, void *data)
    {
        if (!ubf || !data) return;
        ubf->buf[ubf->head] = *((float *)data);
        ubf->head++;
        if (ubf->head >= SHOOT_AVERAGE_CNT) ubf->head = 0;
        if (ubf->cnt < SHOOT_AVERAGE_CNT) ubf->cnt++;
    }

    /**
     * @brief 将缓冲数据弹出到数组，顺序为 新 -> 旧（dst[offset] = newest）。
     * @param ubf      缓冲指针
     * @param dst      目的数组（应能容纳 offset + max_cnt）
     * @param offset   写入 dst 的起始偏移
     * @param max_cnt  最多拷出的数量（通常传 SHOOT_AVERAGE_CNT）
     * @return 实际拷出的元素数（0..max_cnt）
     */
    inline uint16_t UbfPopIntoArrayNew2Old(UBF_SpeedBuffer_t *ubf, float *dst, uint16_t offset, uint16_t max_cnt)
    {
        if (!ubf || !dst || max_cnt == 0) return 0;

        uint16_t avail = ubf->cnt;
        if (avail == 0) return 0;

        /* 实际要拷出的数量不超过 avail 与 max_cnt */
        uint16_t to_copy = (avail < max_cnt) ? avail : max_cnt;

        /* newest 的索引是 head-1（若 head==0 则是 SHOOT_AVERAGE_CNT-1）*/
        int32_t idx = (int32_t)ubf->head - 1;
        if (idx < 0) idx += (int32_t)SHOOT_AVERAGE_CNT;

        for (uint16_t i = 0; i < to_copy; ++i) {
            /* dst[offset + i] = newest, then older... */
            dst[offset + i] = ubf->buf[(uint16_t)idx];
            /* 向前移到上一条（更旧）数据 */
            idx--;
            if (idx < 0) idx += (int32_t)SHOOT_AVERAGE_CNT;
        }
        return to_copy;
    }

    inline uint16_t UbfPopIntoArrayOld2New(UBF_SpeedBuffer_t *ubf, float *dst, uint16_t offset, uint16_t max_cnt)
    {
        if (!ubf || !dst || max_cnt == 0) return 0;

        uint16_t avail = ubf->cnt;
        if (avail == 0) return 0;

        /* 实际要拷出的数量不超过 avail 与 max_cnt */
        uint16_t to_copy = (avail < max_cnt) ? avail : max_cnt;

        /* oldest 的索引 = head - cnt（若 <0 则环回）*/
        int32_t idx = (int32_t)ubf->head - (int32_t)ubf->cnt;
        if (idx < 0) idx += (int32_t)SHOOT_AVERAGE_CNT;

        for (uint16_t i = 0; i < to_copy; ++i) {
            /* dst[offset + i] = 最旧数据开始，依次往新 */
            dst[offset + i] = ubf->buf[(uint16_t)idx];
            /* 向前移到下一条（更新的数据） */
            idx++;
            if (idx >= SHOOT_AVERAGE_CNT) idx = 0;
        }
        return to_copy;
    }

/*************************** 环形缓冲区 *******************************************************/

    /**
     * @brief 仿华农摩擦轮调试数据
     */
    struct ShootTestData_t {
        float speed;            // 实时速度
        uint32_t bullet_cnt;    // 统计发射的子弹数
        float max_speed;        // 最大速度
        uint32_t max_index;     // 最大速度对应的索引
        float min_speed;        // 最小速度
        uint32_t min_index;     // 最小速度对应的索引
        float average_speed;    // 平均速度
        float variance;         // 方差
        float difference;       // 极差
        UBF_SpeedBuffer_t Shoot_SpeedUbf; // 环形缓冲区

        void init() {
            UbfInit(&Shoot_SpeedUbf);
        }

        void ReadShootTestData(void)
        {
            static float last_bullet_speed;
            uint8_t sb[4];
            uint32_t real_num;
            REFEREE_t *REdata =  ECF_RC::ECF_RC_instance.getREdata();
            // if ( bullet_speed != last_bullet_speed ) {
                memcpy(&speed, &ECF_RC::ECF_RC_instance.REFFEREE.Shoot_Data.bullet_speed, sizeof(float));
            // }
            if(speed != last_bullet_speed)
            {
                bullet_cnt++;
                UbfPush(&Shoot_SpeedUbf, (void*)&speed);
                static float Shoot_speed_Aver[80] = {0};
                real_num = UbfPopIntoArrayOld2New(&Shoot_SpeedUbf, Shoot_speed_Aver, 0, SHOOT_AVERAGE_CNT);   //获取最新的SHOOT_AVERAGE_CNT颗弹丸
                arm_max_f32(Shoot_speed_Aver, real_num, &max_speed, &max_index);
                arm_min_f32(Shoot_speed_Aver, real_num, &min_speed, &min_index);
                arm_mean_f32(Shoot_speed_Aver, real_num, &average_speed);
                arm_var_f32(Shoot_speed_Aver, real_num, &variance);
                difference = max_speed - min_speed;
                last_bullet_speed = speed;
            }
        }
    };
}

#endif
