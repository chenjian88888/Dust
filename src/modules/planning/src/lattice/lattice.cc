#include "lattice.h"

namespace dust{
namespace lattice_ns{

lattice::lattice(){

    // subscriber
    referenceLine_subscriber_ = n_.subscribe("/referenceLine_smoothed", 10, &lattice::referenceLineCallback, this);

    gps_sub_ = n_.subscribe("/gps", 10, &lattice::gpsCallback, this);

    // publisher
}


void lattice::referenceLineCallback(const nav_msgs::Path &path_point)
{
  ROS_ERROR("reference line received size:%ld",path_point.poses.size());
  reference_path.first.clear();
  reference_path.second.clear();
  accumulated_s.clear();
  reference_points.clear();
  // 计算参考点的kappa、theta，只需要计算一次
  if (path_point.poses.size() > 0 && path_point_flag_.size() < 0)
  {
      path_point_flag_.push_back(path_point.poses[0].pose.position.x);
      std::vector<double> headings;
      std::vector<double> kappas;
      std::vector<double> dkappas;
      std::vector<std::pair<double, double>> xy_points;

      // auto beforeTime = std::chrono::steady_clock::now(); //计时开始
      for (size_t i = 0; i < path_point.poses.size(); ++i)
      {
          reference_path.first.push_back(path_point.poses[i].pose.position.x);
          reference_path.second.push_back(path_point.poses[i].pose.position.y);
          xy_points.emplace_back(path_point.poses[i].pose.position.x, path_point.poses[i].pose.position.y);
      }

      if (!PathMatcher::ComputePathProfile(xy_points, &headings, &accumulated_s, &kappas, &dkappas))
      {
          ROS_WARN("rerferenceline generate failed!");
      }
      else
      {
        for (size_t i = 0; i < xy_points.size(); ++i)
        {
            //创建ReferencePoint类
            ReferencePoint reference_point(kappas[i], dkappas[i], xy_points[i].first, xy_points[i].second, headings[i], accumulated_s[i]);
            reference_points.emplace_back(reference_point);
        }
      }
    //参考线三次样条方程
    //输入(x1,y1),(x2,y2)......(xn,yn),返回 x-s坐标系与y-s坐标系
    //csp = new CubicSpline2D(reference_path.first, reference_path.second, accumulated_s); //只需要执行一次
    //初始化
    // kappa_init = kappas.front();
    // dkappa_init = dkappas.front();
    // init_lon_state = 0; //复位
    // cango = true;       //复位
  }
}

void lattice::gpsCallback(const msg_gen::gps &pGps){
	gps_ = pGps;
    gps_flag_(1,0);
}

void lattice::plan() {
    ROS_INFO("lattice plan start");
    //初始参数的输入
    InitialConditions lattice_ic = {
        d0,   // 初始的横向偏移值 [m]
        dd0,  // 初始的横向速度 [m/s]
        ddd0, // 初始的横向加速度 [m/s^2]

        init_lon_state,     // 初始的纵向值[m]
        ds0,                // 初始的纵向速度[m/s]
        dds0,               // 初始的纵向加速度[m/ss]
        init_relative_time, // 规划起始点的时间

        x_init, // 用来寻找匹配点
        y_init,
        z_init,
        v_init,
        a_init,

        theta_init,
        kappa_init,
        dkappa_init,
    };
    plan_start_point();
    // 创建起点参数
    TrajectoryPoint planning_init_point(lattice_ic);
    // 轨迹生成
    // PlanningTarget planning_target(Config_.default_cruise_speed, accumulated_s); // 目标
    // best_path = LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
    //                                  FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
}

void lattice::plan_start_point() {
    if (this->is_first_run){
        // 第一次运行
        this->is_first_run = false;
        x_init = gps_.posX;// 定位
        y_init = gps_.posY;
        theta_init = gps_.oriZ;
        kappa_init = 0;
        ds0 = 0;
        dds0 = 0;
        init_relative_time = + 0.1;
    } else{
        // 非第一次运行
        double x_cur = gps_.posX;// 定位
        double y_cur = gps_.posY;
        double theta_cur = gps_.oriZ;
        double kappa_cur = 0;
        double dt = 0.1;
    }
}

} // namespace lattice
} // namespace dust