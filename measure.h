#ifndef __MEASURE_H
#define __MEASURE_H


#define US(X) sm.us[X]  
#define IS(X) sm.is[X]
#define US_C(X) sm.us_curr[X]
#define IS_C(X) sm.is_curr[X]
#define US_P(X) sm.us_prev[X]
#define IS_P(X) sm.is_prev[X]


struct SmMeasure{
    double us[2];
    double is[2];
    double us_curr[2];
    double is_curr[2];
    double us_prev[2];
    double is_prev[2];

    double Js;
    double Js_inv;

    double R;
    double rFlux;
    double Ld;
    double Lq;

    double npp;
    double omg;
    double theta_r;
    double theta_e;
};
extern struct SmMeasure sm;

void SmMeasure_init();
void measurement();

#endif
