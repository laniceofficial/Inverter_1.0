#ifndef TASK_H
#define TASK_H

#define user_abs(x) ((x) > (0) ? (x) : (-(x)))

void task_init();
void task_loop();
void sinTab_genarate();
void PID_Seyduty();
void duty_update();

#endif // !task 