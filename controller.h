#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#define VC_LOOP_CEILING 40

struct PI_Reg{
    double Kp;      // P
    double Ti;      // 积分时间
    double Ki;      //I
    double i_state; // 积分项累加值
    double i_limit; // 输出限制
};

struct ControllerForPmsm{

    double timebase;

    double ual;
    double ube;

    double R;
    double rFlux;   //转子磁链
    double Ld;
    double Lq;

    double Tload;
    double rpm_cmd;

    double Js;
    double Js_inv;

    double omg_fb;      // 反馈转速  一般为真实的测量值
    double ial_fb;      // 反馈alpha轴电流， 一般为真实测量电流经过clark变换得到 clark：abc -> α-β
    double ibe_fb;

    double psi_mu_al_fb; //测量的磁链
    double psi_mu_be_fb;

    double rotor_flux_cmd;  //转子磁链

    double omg_ctrl_err;    //
    double speed_ctrl_err;

    double iDs;     // 定子d轴电流
    double iQs;     // 定子q轴电流

    double theta_e;
    double cosT;
    double sinT;

    double omega_syn;   // 同步角速度

    double uDs_cmd;     // d轴电压
    double uQs_cmd;
    double iDs_cmd;
    double iQs_cmd;

    struct PI_Reg pi_speed;     //速度环
    struct PI_Reg pi_iDs;       //d轴电流环
    struct PI_Reg pi_iQs;       //q轴电流环
};
extern struct ControllerForPmsm CTRL;

void controller_init();
void control(double speed_cmd);

#endif
