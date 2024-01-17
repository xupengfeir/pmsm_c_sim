#include "ACMSim.h"
#include "controller.h"
#include "measure.h"
#include "smo_pll.h"

struct ControllerForPmsm CTRL;

void controller_init(){
    CTRL.timebase = 0.0;

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    CTRL.R = ACSM.R;
    CTRL.rFlux = ACSM.rFlux;
    CTRL.Ld = ACSM.Ld;
    CTRL.Lq = ACSM.Lq;

    CTRL.Tload = 0.0;
    CTRL.rpm_cmd = 0.0;

    CTRL.Js = ACSM.Js;
    CTRL.Js_inv = 1.0/ CTRL.Js;

    CTRL.omg_fb = 0.0;
    CTRL.ial_fb = 0.0;
    CTRL.ibe_fb = 0.0;
    CTRL.psi_mu_al_fb = 0.0;
    CTRL.psi_mu_be_fb = 0.0;

    CTRL.rotor_flux_cmd = 0.0;  // id = 0 控制

    CTRL.omg_ctrl_err = 0.0;
    CTRL.speed_ctrl_err = 0.0;

    CTRL.iDs = 0.0;
    CTRL.iQs = 0.0;

    CTRL.theta_e = 0.0;
    CTRL.cosT = 1.0;
    CTRL.sinT = 0.0;

    CTRL.omega_syn = 0.0;

    CTRL.uDs_cmd = 0.0;
    CTRL.uQs_cmd = 0.0;
    CTRL.iDs_cmd = 0.0;
    CTRL.iQs_cmd = 0.0;

    // pi控制器 参数整定
    CTRL.pi_speed.Kp = 0.5;
    CTRL.pi_speed.Ti = 5;
    CTRL.pi_speed.Ki = (CTRL.pi_speed.Kp*4.77) / CTRL.pi_speed.Ti * (TS*VC_LOOP_CEILING*DOWN_FREQ_EXE_INVERSE); // 4.77 = 60/(npp*2*pi)
    CTRL.pi_speed.i_state = 0.0;
    CTRL.pi_speed.i_limit = 8;
    printf("Kp_omg=%g, Ki_omg=%g\n", CTRL.pi_speed.Kp, CTRL.pi_speed.Ki);   //打印速度环Kp和Ki值

    CTRL.pi_iDs.Kp = 15;
    CTRL.pi_iDs.Ti = 0.08;
    CTRL.pi_iDs.Ki = CTRL.pi_iDs.Kp/CTRL.pi_iDs.Ti*TS;
    CTRL.pi_iDs.i_state = 0.0;
    CTRL.pi_iDs.i_limit = 350;

    CTRL.pi_iQs.Kp = 15;
    CTRL.pi_iQs.Ti = 0.08;
    CTRL.pi_iQs.Ki = CTRL.pi_iQs.Kp/CTRL.pi_iQs.Ti*TS;
    CTRL.pi_iQs.i_state = 0.0;
    CTRL.pi_iQs.i_limit = 650;
    printf("Kp_curr=%g, Ki_curr=%g\n", CTRL.pi_iDs.Kp, CTRL.pi_iDs.Ki);
}

/* PI控制器 */
static double PI(struct PI_Reg *r, double err){
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

void control(double speed_cmd){
    /* 将 测量量(观测器或者编码器测量而来) 反馈给控制器 */
    CTRL.omg_fb = smopll.we;
    CTRL.ial_fb = IS_C(0);
    CTRL.ibe_fb = IS_C(1);
    CTRL.theta_e = smopll.theta_e;

    /* id = 0的控制，磁链=电流*电感 */
    #if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
        CTRL.rotor_flux_cmd = 0;
    #endif
    /* 设置 d轴电流 = d轴磁链/d轴电感 */
    CTRL.iDs_cmd = CTRL.rotor_flux_cmd / CTRL.Ld;

    /* 设置q轴电流， 速度环 */
    static int vc_count = 0;    // 速度环控制频率可以降低一点，频率小于电流环。对于复杂的观测器，它不希望速度环频繁地改变
    if(vc_count++ == VC_LOOP_CEILING*DOWN_FREQ_EXE_INVERSE){    // 20个电流环周期执行一个速度环周期
        vc_count = 0;
        CTRL.omg_ctrl_err  = speed_cmd * RPM_2_RAD_PER_SEC - CTRL.omg_fb;   //给定值 - 真实值 = err，此时输出 +PI
        CTRL.iQs_cmd = -PI(&CTRL.pi_speed, -CTRL.omg_ctrl_err);
        CTRL.speed_ctrl_err = -CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM;
    }
    CTRL.cosT = cos(CTRL.theta_e);
    CTRL.sinT = sin(CTRL.theta_e);
    /* 转为旋转坐标系 */
    CTRL.iDs = AB2D(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    CTRL.iQs = AB2Q(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    /* 电流环 得到dq轴电压分量 */
    CTRL.uDs_cmd = -PI(&CTRL.pi_iDs, -CTRL.iDs_cmd + CTRL.iDs);
    CTRL.uQs_cmd = -PI(&CTRL.pi_iQs, -CTRL.iQs_cmd + CTRL.iQs);
    /* 转为静止坐标系 作为SVPWM的输入*/
    CTRL.ual = DQ2A(CTRL.uDs_cmd, CTRL.uQs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = DQ2B(CTRL.uDs_cmd, CTRL.uQs_cmd, CTRL.cosT, CTRL.sinT);    

}

