#ifndef ADRC_USER_H
#define ADRC_USER_H


typedef struct
{
    float set;
    float rel;
    float out;

    //参数区，这11个就是需要调整的参数
    /****************TD**********/
    float r;        //快速跟踪因子
    float h;        //滤波因子,系统调用步长
    /**************ESO**********/
    float b;        //系统系数
    float delta;    //delta为fal（e，alpha，delta）函数的线性区间宽度
    float belta01;  //扩张状态观测器反馈增益1
    float belta02;  //扩张状态观测器反馈增益2
    float belta03;  //扩张状态观测器反馈增益3
    /**************NLSEF*******/
    float alpha1;//
    float alpha2;//
    float belta1;//跟踪输入信号增益,类似kp
    float belta2;//跟踪微分信号增益,类似ki
}ADRC_t;

void ADRC_init(ADRC_t *ADRC_type , float *a);



#endif

