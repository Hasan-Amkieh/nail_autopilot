#include <math.h>       /* pow(3), log(3) */

double temp_k(double t) {
    return t + 273.15;
}

static double es_pa(double t) {
    double p;
    double es_hpa;
    double eso = 6.1078;
    double c0 =  0.99999683;
    double c1 = -0.90826951e-2;     /* -0.90826951*10**-2; */
    double c2 =  0.78736169e-4;     /*  0.78736169*10**-4; */
    double c3 = -0.61117958e-6;     /* -0.61117958*10**-6; */
    double c4 =  0.43884187e-8;     /*  0.43884187*10**-8; */
    double c5 = -0.29883885e-10;    /* -0.29883885*10**-10; */
    double c6 =  0.21874425e-12;    /*  0.21874425*10**-12; */
    double c7 = -0.17892321e-14;    /* -0.17892321*10**-14; */
    double c8 =  0.11112018e-16;    /*  0.11112018*10**-16; */
    double c9 = -0.30994571e-19;    /* -0.30994571*10**-19; */

    p = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * (c5 + t * (c6 + t * (c7 + t * (c8 + t * (c9)))))))));

    es_hpa = eso/(pow(p,8));
    return (es_hpa * 100);
}

static double partialpressurewatervapour_pa(double t, double rh) {
    return(es_pa(t) * rh);
}

static double partialpressuredryair_pa(double p, double t, double rh) {
    return(p - partialpressurewatervapour_pa(t, rh));
}

double densityhumidair(double p, double t, double rh) {
    double SpecificGasConstantDryAir = 287.0531;
    double SpecificGasConstantWaterVapour = 461.4964;

    return(partialpressuredryair_pa(p, t, rh) / (SpecificGasConstantDryAir * temp_k(t)) +
    partialpressurewatervapour_pa(t, rh) / (SpecificGasConstantWaterVapour * temp_k(t)));
}

