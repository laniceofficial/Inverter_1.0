#ifndef __FILTER_H
#define __FILTER_H


#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include "arm_math.h"
#include "main.h"
//#include "struct_typedef.h"

#ifdef __cplusplus
}

#endif

/**
 * @brief 滤波算法命名空间
*/
namespace alg_n
{
    /****************************一阶低通滤波******************************************** */

    //一阶低通滤波参数

    
    // 一阶低通滤波类
    class FirstOrderFilter_c
    {
        typedef  struct
        {
            float input;		 //输入数据
            float last_input; //上次数据
            float out;		 //滤波输出的数据
            float num;		 //滤波参数
        } FirstOrderFilter_t;
        public:
        float Calc(float input);
        void Init(float num);
        void Clear(void);
        FirstOrderFilter_c(float num);
        private:
        FirstOrderFilter_t measure;
    };


    /**********************滑动均值滤波*************************************************** */
    


    //滑动均值滤波类
    class SlidingMeanFilter_c
    {
        //滑动均值滤波参数（浮点）
        typedef struct
        {
            float input;        //当前取样值
            int32_t count_num; //取样次数
            float output;       //滤波输出
            float sum;          //累计总和
            float FIFO[250];    //队列
            int32_t sum_flag;  //已经够250个标志
        } SlidingMeanFilter_t;
        public:
        SlidingMeanFilter_c();
        void Init();                      //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）
        float Calc(float Input, int num); //均值滑窗滤波
        
        private:
        SlidingMeanFilter_t measure;
    };

    /**********************递推平均滤波*************************************************** */
    //递推平均滤波参数


    class RecursiveAveFilter_c
    {
        typedef struct filter
        {
            int32_t count_num;
            float fifo[300];
            float sum;
            float filter_out;
        } RecursiveAveFilter_t;
    public:
        RecursiveAveFilter_c();
        float Calc(float input, int num);
        RecursiveAveFilter_t measure;
    };

}

#endif // !__FILTER_H
