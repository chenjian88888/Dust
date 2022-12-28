#include "lattice.h"

namespace dust{
namespace lattice_ns{

// global param
std::pair<std::vector<double>, std::vector<double>> reference_path; //参考路径点位置（x,y）
std::vector<double> accumulated_s;                        //纵向距离
std::vector<ReferencePoint> reference_points;             //参考路径点参数

lattice::lattice(){

    // subscriber
    referenceLine_subscriber_ = n_.subscribe("/referenceLine_smoothed", 10, &lattice::referenceLineCallback, this); //订阅

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

void lattice::plan() {
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
    //创建起点参数
    TrajectoryPoint planning_init_point(lattice_ic);
    //轨迹生成
    // PlanningTarget planning_target(Config_.default_cruise_speed, accumulated_s); // 目标
    // best_path = LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
    //                                  FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
}

} // namespace lattice
} // namespace dust