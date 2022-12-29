#include "lattice.h"

namespace dust{
namespace lattice_ns{

lattice::lattice(){
    // ros parameter settings
    ros::param::get("use_lateral_optimization", this->FLAGS_lateral_optimization);

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
    gps_flag_ = {1};
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
    ros::WallTime current_time_ros = ros::WallTime::now();
    double current_time = current_time_ros.toSec();
    plan_start_point(current_time);
    // 创建起点参数
    TrajectoryPoint planning_init_point(lattice_ic);
    // 轨迹生成
    // PlanningTarget planning_target(Config_.default_cruise_speed, accumulated_s); // 目标
    PlanningTarget planning_target(10, accumulated_s); // 目标
    lon_decision_horizon = accumulated_s[accumulated_s.size() - 1];
    best_path_ = LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
                                     FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon);
}

void lattice::plan_start_point(double &current_time) {
    if (this->is_first_run){
        // 第一次运行
        this->is_first_run = false;
        x_init = gps_.posX;// 定位
        y_init = gps_.posY;
        z_init = 0;
        ds0 = 0;
        dd0 = 0;
        theta_init = gps_.oriZ;
        dds0 = 0;
        ddd0 = 0;
        kappa_init = 0;
        dkappa_init = 0;

        init_relative_time = current_time + 0.1;
    } else{
        // 非第一次运行
        double x_cur = gps_.posX;// 定位
        double y_cur = gps_.posY;
        double theta_cur = gps_.oriZ;
        double kappa_cur = 0;
        double vx_cur = gps_.velX;
        double vy_cur = gps_.velY;
        double ax_cur = gps_.accelX;
        double ay_cur = gps_.accelY;
        double dt = 0.1;
        int index = 0;
        for (int i = 0; i < - 1;++i){
            if ( best_path_[i].relative_time <= current_time &&  best_path_[i + 1].relative_time > current_time){
                index = i;
                break;
            }
        }
        // 上一周期规划的本周期车辆应该在的位置
        double pre_x_desire = best_path_[index].x;
        double pre_y_desire = best_path_[index].y;
        double pre_x_theta = best_path_[index].theta;
        // 计算横纵向误差
        Eigen::Matrix<double, 2, 1> tor;
        tor << cos(pre_x_theta), sin(pre_x_theta);
        Eigen::Matrix<double, 2, 1> nor;
        nor << -sin(pre_x_theta), cos(pre_x_theta);
        // 误差向量
        Eigen::Matrix<double, 2, 1> d_err;
        d_err << x_cur - pre_x_desire, y_cur - pre_y_desire;
        double lon_err = abs(tor.transpose() * d_err);
        double lat_err = abs(nor.transpose() * d_err);
        // 如果纵向误差大于2.5或者横向误差大于0.5认为控制没跟上
        if (lon_err > 2.5 || lat_err > 0.5){
            ROS_INFO("out of control, use dynamic to deduce");
            x_init = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
            y_init = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
            z_init = 0;
            ds0 = vx_cur + ax_cur * dt;
            dd0 = vy_cur + ay_cur * dt;
            theta_init = atan2(dd0, ds0);
            dds0 = ax_cur;
            ddd0 = ay_cur;
            kappa_init = kappa_cur;
            dkappa_init = 0;
            init_relative_time = current_time + 0.1;
        }
        else {
          ROS_INFO("control is good");
          int index_good = 0;
          for (int i = 0; i < -1; ++i){
            if (best_path_[i].relative_time <= current_time+0.1 && best_path_[i + 1].relative_time > current_time+0.1)
            {
              index_good = i;
              break;
            }
          }
          x_init = best_path_[index_good].x;
          y_init = best_path_[index_good].y;
          z_init = 0;
          theta_init = best_path_[index_good].theta;
          kappa_init = best_path_[index_good].kappa;
          dkappa_init = best_path_[index_good].dkappa;
          // 上一帧轨迹的速度和加速度是指轨迹的切向速度，切向加速度
          ds0 = best_path_[index_good].v * cos(theta_init);
          dd0 = best_path_[index_good].v * sin(theta_init);
          // 计算轨迹在current_time + 100ms这个店的切向法向量
          tor << cos(theta_init), sin(theta_init);
          nor << -sin(theta_init), cos(theta_init);
          Eigen::Matrix<double, 2, 1> a_tor = best_path_[index_good].a * tor;
          Eigen::Matrix<double, 2, 1> a_nor = best_path_[index_good].v * best_path_[index_good].v * kappa_init * nor;
          dds0 = a_tor[0] + a_nor[0];
          ddd0 = a_tor[1] + a_nor[1];
          init_relative_time = best_path_[index_good].relative_time;
        }
    }
}

DiscretizedTrajectory lattice::LatticePlan(
    const TrajectoryPoint &planning_init_point,
    const PlanningTarget &planning_target,
    const std::vector<const Obstacle *> &obstacles,
    const std::vector<double> &accumulated_s,
    const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
    const double &init_relative_time, const double lon_decision_horizon)
{
  //ROS_WARN("start_lattice");
  DiscretizedTrajectory Optim_trajectory;

  // 1. compute the matched point of the init planning point on the reference line.经过投影后的匹配点
  ReferencePoint matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.path_point().x,
                                                          planning_init_point.path_point().y);

  // 2. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  // ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  // //与Apollo不同，我们的前探距离不加上init_s[0]，因为我们的仿真的参考线不长
  // auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_points, init_s[0],
  //                                                            init_s[0] + 20, //前瞻多少m lon_decision_horizon
  //                                                            0.0, Config_.FLAGS_trajectory_time_length, init_d);

  // auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points));
  // auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(obstacles, ptr_reference_line);

  // // 3.生成纵向和横向轨迹
  // Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  // std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  // std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  // trajectory1d_generator.GenerateTrajectoryBundles(planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  // //打印轨迹数
  // // std::cout << "lon_trajectory1d_bundle:" << lon_trajectory1d_bundle.size() << "\n";
  // // std::cout << "lat_trajectory1d_bundle:" << lat_trajectory1d_bundle.size() << "\n";

  // // 4.计算每条轨迹的代价,并得出优先级队列
  // TrajectoryEvaluator trajectory_evaluator(planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
  //                                          init_s, ptr_path_time_graph, reference_points);

  // // 5.轨迹拼接和最后的筛选
  // while (trajectory_evaluator.has_more_trajectory_pairs())
  // {
  //   double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
  //   auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
  //   // combine two 1d trajectories to one 2d trajectory
  //   auto combined_trajectory = trajectorycombiner.Combine(accumulated_s, *trajectory_pair.first, *trajectory_pair.second,
  //                                                         reference_points, init_relative_time);

  //   // 采样时候才调用，二次规划不用
  //   if (lateral_optimization == false)
  //   {
  //     // check longitudinal and lateral acceleration
  //     // considering trajectory curvatures
  //     auto result = constraintchecker_.ValidTrajectory(combined_trajectory);
  //     if (result != ConstraintChecker::Result::VALID)
  //     {
  //       switch (result)
  //       {
  //       case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
  //         break;
  //       case ConstraintChecker::Result::VALID:
  //       default:
  //         // Intentional empty
  //         break;
  //       }
  //       continue;
  //     }
  //     // Get instance of collision checker and constraint checker
  //     CollisionChecker collision_checker(obstacles, init_s[0], init_d[0], reference_points, ptr_path_time_graph);
  //     //碰撞检测
  //     if (collision_checker.InCollision(combined_trajectory))
  //     {
  //       continue;
  //     }
  //   }
  //   Optim_trajectory = std::move(combined_trajectory);
  //   break;
  // }
  // //ROS_WARN("trj_num :%d",Optim_trajectory.size());
  // return Optim_trajectory;
}

// void lattice::ComputeInitFrenetState(const ReferencePoint &matched_point, const TrajectoryPoint &cartesian_state,
//                             std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d)
// {
//   CartesianFrenetConverter::cartesian_to_frenet(
//       matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
//       matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
//       cartesian_state.path_point().x, cartesian_state.path_point().y,
//       cartesian_state.v, cartesian_state.a,
//       cartesian_state.path_point().theta,
//       cartesian_state.path_point().kappa, ptr_s, ptr_d);
// }

} // namespace lattice
} // namespace dust