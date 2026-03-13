#ifndef BSP_BUZZER_HPP
#define BSP_BUZZER_HPP
#include "tim.h"

namespace Buzzer_n
{
#define N_ST .0f
#define N_C0 16.35f
#define N_CS0 17.32f
#define N_D0 18.35f
#define N_DS0 19.45f
#define N_E0 20.60f
#define N_F0 21.83f
#define N_FS0 23.12f
#define N_G0 24.50f
#define N_GS0 25.96f
#define N_A0 27.50f
#define N_AS0 29.14f
#define N_B0 30.87f
#define N_C1 32.70f
#define N_CS1 34.65f
#define N_D1 36.71f
#define N_DS1 38.89f
#define N_E1 41.20f
#define N_F1 43.65f
#define N_FS1 46.25f
#define N_G1 49.00f
#define N_GS1 51.91f
#define N_A1 55.00f
#define N_AS1 58.27f
#define N_B1 61.74f
#define N_C2 65.41f
#define N_CS2 69.30f
#define N_D2 73.42f
#define N_DS2 77.78f
#define N_E2 82.41f
#define N_F2 87.31f
#define N_FS2 92.50f
#define N_G2 98.00f
#define N_GS2 103.83f
#define N_A2 110.00f
#define N_AS2 116.54f
#define N_B2 123.47f
#define N_C3 130.81f
#define N_CS3 138.59f
#define N_D3 146.83f
#define N_DS3 155.56f
#define N_E3 164.81f
#define N_F3 174.61f
#define N_FS3 185.00f
#define N_G3 196.00f
#define N_GS3 207.65f
#define N_A3 220.00f
#define N_AS3 233.08f
#define N_B3 246.94f
#define N_C4 261.63f
#define N_CS4 277.18f
#define N_D4 293.66f
#define N_DS4 311.13f
#define N_E4 329.63f
#define N_F4 349.23f
#define N_FS4 369.99f
#define N_G4 392.00f
#define N_GS4 415.30f
#define N_A4 440.00f
#define N_AS4 466.16f
#define N_B4 493.88f
#define N_C5 523.25f
#define N_CS5 554.37f
#define N_D5 587.33f
#define N_DS5 622.25f
#define N_E5 659.26f
#define N_F5 698.46f
#define N_FS5 739.99f
#define N_G5 783.99f
#define N_GS5 830.61f
#define N_A5 880.00f
#define N_AS5 932.33f
#define N_B5 987.77f
#define N_C6 1046.50f
#define N_CS6 1108.73f
#define N_D6 1174.66f
#define N_DS6 1244.51f
#define N_E6 1318.51f
#define N_F6 1396.91f
#define N_FS6 1479.98f
#define N_G6 1567.98f
#define N_GS6 1661.22f
#define N_A6 1760.00f
#define N_AS6 1864.66f
#define N_B6 1975.53f
#define N_C7 2093.00f
#define N_CS7 2217.46f
#define N_D7 2349.32f
#define N_DS7 2489.02f
#define N_E7 2637.02f
#define N_F7 2793.83f
#define N_FS7 2959.96f
#define N_G7 3135.96f
#define N_GS7 3322.44f
#define N_A7 3520.00f
#define N_AS7 3729.31f
#define N_B7 3951.07f
#define N_C8 4186.01f
#define N_CS8 4434.92f
#define N_D8 4698.64f
#define N_DS8 4978.03f
#define N_E8 5274.04f
#define N_F8 5587.65f
#define N_FS8 5919.91f
#define N_G8 6271.93f
#define N_GS8 6644.88f
#define N_A8 7040.00f
#define N_AS8 7458.62f
#define N_B8 7902.13f

#define N_L1 (int)N_C4
#define N_L2 (int)N_D4
#define N_L3 (int)N_E4
#define N_L4 (int)N_F4
#define N_L5 (int)N_G4
#define N_L6 (int)N_A4
#define N_L7 (int)N_B4
#define N_M1 (int)N_C5
#define N_M2 (int)N_D5
#define N_M3 (int)N_E5
#define N_M4 (int)N_F5
#define N_M5 (int)N_G5
#define N_M6 (int)N_A5
#define N_M7 (int)N_B5
#define N_H1 (int)N_C6
#define N_H2 (int)N_D6
#define N_H3 (int)N_E6
#define N_H4 (int)N_F6
#define N_H5 (int)N_G6
#define N_H6 (int)N_A6
#define N_H7 (int)N_B6

    class Buzzer_c
    {
    public:
        Buzzer_c(TIM_HandleTypeDef *htim,
                 uint32_t channel, uint16_t apb_mhz);
        Buzzer_c(TIM_HandleTypeDef *htim,
                 uint32_t channel, uint16_t apb_mhz, float allow_dif);
        ~Buzzer_c() = default;
        void On(void);
        void Off(void);
        void Test(uint16_t psc, uint16_t arr);
        void Play(float note);

    private:
        TIM_HandleTypeDef *use_tim_ = nullptr;
        uint32_t channel_ = 0;
        uint32_t apb_hz_ = 0;
        uint16_t arr_ = 0;
        float allow_dif_ = 1.0f; // 允许实际音高与给定音高的频率差
    };
}

#endif // !BSP_BUZZER_HPP