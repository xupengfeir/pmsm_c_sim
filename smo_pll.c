#include "smo_pll.h"
#include "measure.h"
#include "ACMSim.h"

struct SmoPll smopll;

void SmoPLL_init(){
    smopll.ual = 0; // US_C(0);
    smopll.ube = 0; // US_C(1);
    smopll.eal = 0;
    smopll.ebe = 0;
    smopll.val = 0;
    smopll.vbe = 0;

    smopll.ial = 0; // IS_C(0); 实测电流
    smopll.ibe = 0; // IS_C(1);
    smopll.ial_es = 0;
    smopll.ibe_es = 0;
    smopll.ial_es_next = smopll.ial_es;
    smopll.ibe_es_next = smopll.ibe_es;

    smopll.A = exp(-ACSM.R/(ACSM.Ls * ACSM.Ts));    // 这两个值需要实时更新，因为凸极电机Ls是变化的
    smopll.B = (1 - smopll.A) / ACSM.R;

    smopll.k = 200;     // 系数k
    smopll.we = 0;                      // 初始角速度 0
    smopll.theta_e = TWO_PI_OVER_3;     // 和计算Ls时电角度一致

    /* PI参数待完善 占坑*/
    smopll.Pipll.Kp = 0;     
    smopll.Pipll.Ki = 0;
    smopll.Pipll.Ti = 0;
    smopll.Pipll.i_limit = 0;
    smopll.Pipll.i_state = 0;

}
/* 饱和函数 */
static double sat_smo(double iError){
    if(iError > SATURATION_LIMIT)
        return 1;
    else if(iError < -SATURATION_LIMIT)
        return -1;
    else if(fabs(iError) <= SATURATION_LIMIT)
        return (iError / SATURATION_LIMIT);
}
/* 2阶低通滤波器 */
static double Lowfilter2(double input){
    /*
        2阶低通滤波器实现
    */
    return 0;
}
/* PI 控制器 */
static double PI(struct PI_pll *r, double err){
    #define I_STATE r->i_state
    #define I_LIMIT r->i_limit
    double output;
    I_STATE += err * r->Ki;
    if(I_STATE > I_LIMIT)
        I_STATE = I_LIMIT;
    else if(I_STATE < -I_LIMIT)
        I_STATE = -I_LIMIT;

    output = I_STATE + err * r->Kp;
    if(output > I_LIMIT)
        output = I_LIMIT;
    else if(output < -I_LIMIT)
        output = -I_LIMIT;
    return output;
    #undef I_STATE
    #undef I_LIMIT
}
/* 积分器 */
static double Integral_pll(double input){
    /*
        积分器实现（考虑积分步长）
    */
    return 0;
}
/* SMO + PLL 参考袁雷书实现 */
void Smo_Observation(){
    /* SMO */
    smopll.ial = IS_C(0);   // 测量数值
    smopll.ibe = IS_C(1);
    smopll.ual = US_C(0);
    smopll.ube = US_C(1);

    smopll.val = Lowfilter2(sat_smo(-smopll.ial + smopll.ial_es) * smopll.k);   /* 低通滤波器待完善 */
    smopll.ial_es_next = (smopll.ual - smopll.val) * smopll.B + smopll.A * smopll.ial_es;
    smopll.ial_es = smopll.ial_es_next;

    smopll.vbe = Lowfilter2(sat_smo(-smopll.ibe + smopll.ibe_es) * smopll.k);   /* 低通滤波器待完善 */
    smopll.ibe_es_next = (smopll.ube - smopll.vbe) * smopll.B + smopll.A * smopll.ibe_es;
    smopll.ibe_es = smopll.ibe_es_next;

    smopll.eal = smopll.val;
    smopll.ebe = smopll.vbe;

    /* PLL */
    double eError = (-smopll.eal * cos(smopll.theta_e) - smopll.ebe * sin(smopll.theta_e));
    smopll.we = PI(&smopll.Pipll, eError);
    smopll.theta_e = Integral_pll(smopll.we);   /* 积分器待完善 */
}