#ifndef TASK_H
#define TASK_H

#ifdef __cplusplus
extern "C" {
#endif

// void task_init();
// void task_loop();

#define user_abs(x) ((x) > (0) ? (x) : (-(x)))
#define V_KP 0.01f
#define V_KI 0.0003f
void task_init();
void task_loop();
void sinTab_genarate();
void PID_Seyduty();
void duty_update();

#ifdef __cplusplus
}
#endif

#endif // !task 
