

typedef struct cofe
{
    /* data */
    float A1;
    float A2;
    float B0;
    float B1;
    float B2;
    float gain;

} DIS_2ORDER_TF_COEF_DEF;
typedef struct data
{
    float w0;
    float w1;
    float w2;

    float output;
} DIS_2ORDER_TF_DATA_DEF;