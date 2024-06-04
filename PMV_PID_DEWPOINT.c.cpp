// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

using   namespace   std;
#pragma warning(disable:4996)
#include <windows.h>

#define INT_TYPE 0
#if INT_TYPE

struct PID {
    unsigned int SetPoint; // 设定目标 Desired Value
    unsigned int Proportion; // 比例常数 Proportional Const
    unsigned int Integral; // 积分常数 Integral Const
    unsigned int Derivative; // 微分常数 Derivative Const
    unsigned int LastError; // Error[-1]
    unsigned int PrevError; // Error[-2]
    unsigned int SumError; // Sums of Errors
    unsigned int out;
};
struct PID spid; // PID Control Structure
unsigned int rout; // PID Response (Output) 响应输出
unsigned int rin; // PID Feedback (Input)//反馈输入
void PIDInit(struct PID* pp)
{
    memset(pp, 0, sizeof(struct PID)); //用参数0初始化pp
}


void PID_setpoint(uint8_t ch, uint32_t setvalue)
{
    spid.SetPoint = setvalue;
}
unsigned int incPIDCalc(struct PID* pp, unsigned int NextPoint) //PID计算
{
    unsigned int dError, Error;
    int iError, iincpid;
    Error = pp->SetPoint - NextPoint; // 偏差
    pp->SumError += Error; // 积分
    dError = pp->LastError - pp->PrevError; // 当前微分
    pp->PrevError = pp->LastError;
    pp->LastError = Error;


    iincpid = (pp->Proportion * (Error - pp->LastError)
        +pp->Integral * Error 
        + pp->Derivative * (Error - 2 * pp->LastError + pp->PrevError)); 
    pp->PrevError = pp->LastError; //存储误差，便于下次计算
    pp->LastError = Error;
    return iincpid;
   
#if 0
    return (pp->Proportion * Error-//比例
        + pp->Integral * pp->SumError  //积分项
        + pp->Derivative * dError); //   微分项
#endif
}
unsigned int PIDCalc(struct PID* pp, unsigned int NextPoint) //PID计算
{
    unsigned int dError, Error;
    int iError, iincpid;


    Error = pp->SetPoint - NextPoint; // 偏差
    pp->SumError += Error; // 积分
    dError = pp->LastError - pp->PrevError; // 当前微分
    pp->PrevError = pp->LastError;
    pp->LastError = Error;

    return (pp->Proportion * Error -//比例
        +pp->Integral * pp->SumError  //积分项
        + pp->Derivative * dError); //   微分项

}

uint32_t PID_Control(uint8_t ch, uint32_t currentvalue)
{
    uint16_t Duty = 0;
    PID* PIDx = &spid;
    PIDx->SumError += (incPIDCalc(PIDx, currentvalue));  //计算增量并累加 

    std::cout << "currentvalue=" << currentvalue << "targetvalue=" << PIDx->SetPoint << "sum_error=" << PIDx->SumError << "out=" << PIDx->out << std::endl;
    //printf("ch=%d, currentvalue=%f, targetvalue=%f, sum_error=%f\r\n", ch, currentvalue, PIDx->setpoint, PIDx->sum_error);
    return PIDx->SumError;
    //Duty=PIDx->sum_error*100/4096;   //pwm1 代表将要输出PWM的占空比
    //if(Duty > 100)Duty = 100;
    //Duty = 100-Duty;

    //bsp_SetTempDuty(ch,Duty);
}

#else

#define PID_CH_NUM 2

#define PID_Ti      0.09  //积分时间常数
#define PID_Td     0.0028  //微分时间常数
#define PID_T       0.02  //采样周期
#define PID_Kp      0.32 //比例常数
#define PID_Ki      (PID_Kp * PID_T / PID_Ti)        // Kp Ki Kd 三个主要参数
#define PID_Kd      (PID_Kp * PID_Td / PID_T)



typedef struct
{
    float setpoint;//设定目标
    float sum_error;//误差累计
    float proportion;//比例常数
    float integral;//积分常数
    float derivative;//微分常数
    int last_error;//e[-1]
    int prev_error;//e[-2]
}PIDtypedef;

//float PID_Control(uint8_t ch, uint16_t currentvalue);
//void PID_setpoint(uint8_t ch, uint16_t setvalue);
//void PID_set(uint8_t ch, uint32_t pp, uint32_t ii, uint32_t dd);



PIDtypedef PID[PID_CH_NUM];	 //PID结构体

void incPIDinit(void)
{
    uint8_t i = 0;
    for (i = 0; i < PID_CH_NUM; i++) {
        //PID1参数初始化
        PID[i].sum_error = 0;
        PID[i].last_error = 0;
        PID[i].prev_error = 0;
        PID[i].proportion = 0;
        PID[i].integral = 0;
        PID[i].derivative = 0;
        PID[i].setpoint = 0;
    }
}

int incPIDcalc(PIDtypedef* PIDx, float nextpoint)
{
    int iError, iincpid;
    iError = PIDx->setpoint - nextpoint;  //当前误差
    /*iincpid=					       //增量计算
    PIDx->proportion*iError	        //e[k]项
    -PIDx->integral*PIDx->last_error	  //e[k-1]
    +PIDx->derivative*PIDx->prev_error;//e[k-2]
    */
    iincpid =							  //增量计算
        PIDx->proportion * (iError - PIDx->last_error)
        + PIDx->integral * iError
        + PIDx->derivative * (iError - 2 * PIDx->last_error + PIDx->prev_error);

    PIDx->prev_error = PIDx->last_error; //存储误差，便于下次计算
    PIDx->last_error = iError;
    return(iincpid);
}

uint32_t PID_Control(uint8_t ch, float currentvalue)
{
    uint16_t Duty = 0;
    PIDtypedef* PIDx = &PID[ch];
    PIDx->sum_error += (incPIDcalc(PIDx, currentvalue));  //计算增量并累加 

    std::cout << "currentvalue=" << currentvalue << "targetvalue=" << PIDx->setpoint << "sum_error=" << PIDx->sum_error << std::endl;
    //printf("ch=%d, currentvalue=%f, targetvalue=%f, sum_error=%f\r\n", ch, currentvalue, PIDx->setpoint, PIDx->sum_error);
    return PIDx->sum_error;
    //Duty=PIDx->sum_error*100/4096;   //pwm1 代表将要输出PWM的占空比
    //if(Duty > 100)Duty = 100;
    //Duty = 100-Duty;
 
    //bsp_SetTempDuty(ch,Duty);
}

void PID_setpoint(uint8_t ch, float setvalue)
{
    PID[ch % PID_CH_NUM].setpoint = setvalue;
}

void PID_set(uint8_t ch, float pp, float ii, float dd)
{
    PID[ch].proportion = pp;
    PID[ch].integral = ii;
    PID[ch].derivative = dd;
}

#endif
double calculate_dew_point(float temperature, float humidity)
{
    double temp_, humid_;
    double gamma, dewpoint;
    int16_t int_dewpoint;

    temp_ = (double)temperature;
    humid_ = (double)humidity;

    //printf("Calculating dew point for temperature %d.%d Celcius and humidity %d.%d RH\n",temperature / 10, temperature % 10, humidity / 10, humidity % 10);

    gamma = (17.271 * temp_) / (237.7 + temp_) + log(humid_ / 100.0);
    dewpoint = (237.7 * gamma) / (17.271 - gamma);
    //printf ("Dew Point %.2lf B:C\n", dewpoint);

    int_dewpoint = (int16_t)(dewpoint * 100);
    //printf ("Int dew point %d.%d B:C\n", int_dewpoint / 100,abs (int_dewpoint % 100));

    return dewpoint;
}

float calculateTcl(float taa, float tdb, float icl, float f_cl, float mw, float p1, float p2, float p3, float p4, float t_cla, float hc, float p5) {
    float xn = t_cla / 100;
    float xf;
    float eps = 0.01;
    int n = 0;

    do {
        xf = xn;
        xn = xf - (p5 + p4 * hc - p2 * pow(xf, 4)) / (100 + p3 * hc);
        n++;
    } while (fabs(xn - xf) > eps && n < 100);

    return 100 * xn - 273;
}
double calculatePMVGPT(double tdb, double tr, double vr, double rh, double met, double clo, double wme) {
    double pa = rh * 10 * exp(16.6536 - 4030.183 / (tdb + 235));
    double icl = 0.155 * clo;  // 衣着的热阻
    double m = met * 58.15;  // 代谢率
    double w = wme * 58.15;  // 外部功率
    double mw = m - w;  // 人体内部产热
    double fcl;
    if (icl < 0.078) {
        fcl = 1.0 + 1.29 * icl;
    }
    else {
        fcl = 1.05 + 0.645 * icl;
    }
    double hcf = 12.1 * sqrt(vr);
    double taa = tdb + 273;
    double tra = tr + 273;
    double tcla = taa + (35.5 - tdb) / (3.5 * 100000);
    double p1 = icl * fcl;
    double p2 = p1 * 3.96;
    double p3 = p1 * 100;
    double p4 = p1 * taa;
    double p5 = 308.7 - 0.028 * mw + p2 * pow(tra / 100, 4);
    double xn = tcla / 100;
    double xf = tcla / 50;
    double eps = 0.00015;

    int n = 0;
    double hcn = hcf;
    double hc = hcf;
    double xn_prev, xf_prev, tcla_prev;
    while (1) {
        xn_prev = xn;
        xf_prev = xf;
        tcla_prev = tcla;
        double xf_r = (xf * xf * xf + 50) / 100;
        double hc_2 = hcf * (3.0 + 0.1 * fabs(xf - 58.15 - 1.2 * mw));
        hc = (hc_2 > hcn) ? hc_2 : hcn;
        double taa_r = tcla + p5;
        double xn_2 = (p3 * hc) / (taa_r - 308.7);
        xn = (xn_2 + xn) / 2;
        xf = xn * (tcla / 100);
        if (fabs(xn - xn_prev) <= eps || n >= 200) {
            break;
        }
        n++;
    }

    double pmv = 0.303 * exp(-0.036 * m) + 0.028 - mw - hcf * (xn - 58.15 - 1.2 * mw);

    return pmv;
}




//#include <stdio.h>
//#include <math.h>

/// <summary>
/// 
/// </summary>
/// <param name="tdb">干球温度</param>
/// <param name="tr">平均辐射温度</param>
/// <param name="vr">风速</param>
/// <param name="rh">相对湿度</param>
/// <param name="met">代谢率</param>
/// <param name="clo">服装热阻</param>
/// <param name="wme">对外做工 通常为0</param>
/// <returns></returns>
float callPMV(float tdb, float tr, float vr, float rh, float met, float clo, float wme)
{
    float pa = rh * 10 * exp(16.6536 - 4030.183 / (tdb + 235));

    float icl = 0.155 * clo;  // thermal insulation of the clothing in M2K / W
    float m = met * 58.15;  // metabolic rate in W / M2
    float w = wme * 58.15; // external work in W / M2
    float mw = m - w; // internal heat production in the human body
    float f_cl = 0, hcf = 0, hc = 0, taa = 0, tra = 0, t_cla = 0, p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, xn, hcn = 0, xf = 0, eps = 0, n = 0;
    float tcl = 0, hl1 = 0, hl2, hl3, hl4, hl5, hl6, ts, _pmv;
    float PPD = 0;
    // calculation of the clothing area factor
    if (icl <= 0.078)
        f_cl = 1 + (1.29 * icl);  // ratio of surface clothed body over nude body
    else
        f_cl = 1.05 + (0.645 * icl);

    // heat transfer coefficient by forced convection
    hcf = 12.1 * sqrt(vr);
    hc = hcf;  // initialize variable
    taa = tdb + 273;
    tra = tr + 273;
    t_cla = taa + (35.5 - tdb) / (3.5 * icl + 0.1);

    p1 = icl * f_cl;
    p2 = p1 * 3.96;
    p3 = p1 * 100;
    p4 = p1 * taa;
    p5 = (308.7 - 0.028 * mw) + (p2 * pow((tra / 100.0), 4));
    xn = t_cla / 100;
    xf = t_cla / 50;
    eps = 0.00015;

    n = 0;
    while (abs(xn - xf) > eps)
    {
        xf = (xf + xn) / 2;
        hcn = 2.38 * pow(abs(100.0 * xf - taa), 0.25);
        if (hcf > hcn)
            hc = hcf;
        else
            hc = hcn;
        xn = (p5 + p4 * hc - p2 * pow(xf, 4)) / (100 + p3 * hc);
        n += 1;
        if (n > 150)
            printf("Max iterations exceeded");
    };


    tcl = 100 * xn - 273;

    // heat loss diff.through skin
    hl1 = 3.05 * 0.001 * (5733 - (6.99 * mw) - pa);
    /// heat loss by sweating
    if (mw > 58.15)
        hl2 = 0.42 * (mw - 58.15);
    else
        hl2 = 0;
    // latent respiration heat loss
    hl3 = 1.7 * 0.00001 * m * (5867 - pa);
    // dry respiration heat loss
    hl4 = 0.0014 * m * (34 - tdb);
    // heat loss by radiation
    hl5 = 3.96 * f_cl * (pow(xn, 4) - pow((tra / 100.0), 4));
    // heat loss by convection
    hl6 = f_cl * hc * (tcl - tdb);

    ts = 0.303 * exp(-0.036 * m) + 0.028;
    _pmv = ts * (mw - hl1 - hl2 - hl3 - hl4 - hl5 - hl6);
    PPD = 100 - 95 * exp(-0.3353 * pow(_pmv, 4) - 0.2179 * pow(_pmv, 2));
    printf("PMV值为: %.2f,PPD = %f\n", _pmv,PPD);


    return _pmv;
}
   



// 辅助计算函数
double calculatePa(double rh, double ta) {
    return rh * 10 * exp(16.6536 - 4030.183 / (ta + 235));
}

// 主计算函数
double calculatePMV2(double M, double W, double Icl, double fcl, double ta, double tr, double vel, double rh) {
    double hc, tcl, pa, pmv, ppd;
    double icl = Icl * 0.155; // 转换clo值到m²K/W

    // 计算水蒸气压 (Pa)
    pa = calculatePa(rh, ta);

    // 初始计算Tcl
    tcl = ta + (35.5 - ta) / (3.5 * icl + 0.1);

    // 迭代计算Tcl和hc
    for (int i = 0; i < 150; i++) {
        double tcl_old = tcl;
        hc = 12.1 * sqrt(vel);
        tcl = (35.7 - 0.028 * (M - W) - icl * (3.96 * pow(fcl, 0.4) * (pow((tcl + 273.15), 4) - pow((tr + 273.15), 4)) + hc * fcl * (tcl - ta))) / (icl * (3.96 * pow(fcl, 0.4) * (pow((tcl + 273.15), 4) - pow((tr + 273.15), 4)) + hc * fcl));
        if (fabs(tcl - tcl_old) < 0.001) {
            break;
        }
    }

    // 计算PMV
    pmv = (0.303 * exp(-0.036 * M) + 0.028) * ((M - W) - 3.96 * fcl * (pow((tcl + 273.15), 4) - pow((tr + 273.15), 4)) - fcl * hc * (tcl - ta) - 0.0014 * M * (34 - ta) - 3.05 * (5.73 - 0.007 * (M - W) - pa));

    return pmv;
}
void CalcPMV2()
{
    // 定义常数
    float M = 70.0;       // 代谢率 (W/m²)
    float W = 0.0;       // 机械功 (W/m²)
    float Icl = 0.155;    // 衣服隔热值 (clo)
    float fcl = 1.05;    // 衣服面积因子
    float ta = 25.0;     // 空气温度 (°C)
    float tr = 25.0;      // 平均辐射温度 (°C)
    float vel = 0.1;     // 空气速度 (m/s)
    float rh = 50.0;      // 相对湿度 (%)
   float pmv =  calculatePMV2(M, W, Icl, fcl, ta, tr, vel, rh);
      printf("PMV值为: %.2f\n", pmv);
}
/*
// 计算PMV值的函数
double calculatePMV(double ta, double tr, double vel, double rh, double met, double clo) {
    // 计算水蒸气分压
    double pa = rh / 100 * 10 * exp(16.6536 - 4030.183 / (ta + 235));
    // 计算服装表面热阻
    double icl = 0.155 * clo;
    // 计算代谢率
    double m = met * 58.15;
    // 计算蒸发热
    double w = m - 3.05;
    // 计算有效代谢率
    double mw = m - icl * (3.96 * exp(0.067 * w) - 6.1);
    // 计算人体表面积
    double fcl = 1.0 + 0.15 * clo;
    // 计算对流热损失系数
    double hcf = 12.1 * sqrt(vel);
    // 计算人体表面温度
    double tcl = ta + 273 - 0.42 * (vel / hcf * (1.0 + 1.0 / 7.0) * (1.0 - icl) - 1.0) * (ta - tr);
    // 计算蒸发潜热
    double hl1 = 3.05 * 0.001 * (5733 - 6.99 * mw - pa);
    // 计算辐射热损失
    double hl2 = 0.42 * (mw - 58.15);
    // 计算汗蒸发热损失
    double hl3 = 1.7 * 0.00001 * m * (5867 - pa);
    // 计算呼吸损失
    double hl4 = 0.0014 * m * (34 - ta);
    // 计算最后的辐射热损失
    double hl5 = 3.96 * fcl * (exp(0.067 * mw) - 1);
    // 计算对流换热系数
    double ts = 0.303 * exp(-0.036 * m) + 0.028;
    // 计算PMV值
    double pmv = ts * (mw - hl1 - hl2 - hl3 - hl4 - hl5);

    return pmv;
}

int CalcPMV() {
    // 定义输入参数
    double ta = 25;  // 室内空气温度（摄氏度）
    double tr = 24;  // 平均辐射温度（摄氏度）
    double vel = 0.1;  // 空气流速（m/s）
    double rh = 50;  // 相对湿度（%）
    double met = 1.2;  // 代谢率（metabolic rate）
    double clo = 0.6;  // 服装热阻（clothing insulation）

    // 调用计算PMV值的函数
    double pmv = calculatePMV(ta, tr, vel, rh, met, clo);
    // 输出结果
    printf("PMV值为: %.2f\n", pmv);

    return 0;
}
*/


int main()
{

    int i = 1;
    uint32_t curValue = 0.0f;
    uint32_t outValue = 0;

   								//PID controller
    volatile int32_t control = 0;					//control variable
    uint8_t dc = 0;									//duty cycle

    callPMV(25, 25, 0.2, 35, 1.4, 0.61, 0);
    calculatePMVGPT(25, 25, 0.2, 35, 1.4, 0.61,0);
    while (0)
    {
#if INT_TYPE
        PIDInit(&spid); // Initialize Structure
        spid.Proportion = 10; // Set PID Coefficients
        spid.Integral = 8;
        spid.Derivative = 6;
        spid.SetPoint = 3000; // Set PID Setpoint
        PID_setpoint(i, 3000);//SetValue
        //spid.out = PIDCalc(&spid,curValue);
        curValue = PID_Control(i, curValue);//currentValue
        if (curValue > 400) curValue = 400;
        curValue += 26.0f;
#else

        for (i = 0; i <1; i++)
        {
            PID_set(i, PID_Kp, PID_Ki, PID_Kd);
            PID_setpoint(i, 3000);//SetValue

            curValue = PID_Control(i, curValue);//currentValue
         
           curValue += 26.0f +i;
            _sleep(1000);
        }
        calculate_dew_point(24.5, 48);
#endif
    }

    std::cout << "Hello World!\n";
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
