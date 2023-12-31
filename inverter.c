#include "ACMSim.h"
#include "controller.h"

void inverter(){
    ACSM.ual = CTRL.ual;
    ACSM.ube = CTRL.ube;
    ACSM.ud = AB2D(ACSM.ual, ACSM.ube, cos(ACSM.theta_e), sin(ACSM.theta_e));
    ACSM.uq = AB2Q(ACSM.ual, ACSM.ube, cos(ACSM.theta_e), sin(ACSM.theta_e));
}