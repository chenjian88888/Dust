#ifndef __plan_init__
#define __plan_init__
// Lattice_dynamic and EM_dynamic
struct InitialConditions
{
  double d0;   //初始的横向偏移值 [m]
  double dd0;  //初始的横向速度 [m/s]
  double ddd0; //初始的横向加速度 [m/s^2]

  double s0;   //初始的纵向值[m]
  double ds0;  //初始的纵向速度[m/s]
  double dds0; //初始的纵向加速度[m/ss]

  double init_relative_time; //规划起始点的时间

  double x_init;
  double y_init;
  double z_init;

  double v_init;
  double a_init;

  double theta_init;
  double kappa_init;
  double dkappa_init;
};

#endif