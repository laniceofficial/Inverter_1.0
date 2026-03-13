#include "Motor_General_def.hpp"

void Motor_General_Def_n::Motor_Controller_c::RefValChange(float ref_val)
{
    this->pid_ref = ref_val;
}

float Motor_General_Def_n::Motor_Controller_c::GetRefVal(void)
{
    return this->pid_ref;
}