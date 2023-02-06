#include "lattice.h"

std::vector<Obstacle> AllObstacle;                        //感知一帧识别到的所有障碍物
namespace dust{
namespace lattice_ns{

std::vector<PathPoint> ToDiscretizedReferenceLine(
      const std::vector<ReferencePoint> &ref_points)
  {
    double s = 0.0;
    std::vector<PathPoint> path_points;
    for (const auto &ref_point : ref_points)
    {
      PathPoint path_point;
      path_point.set_x(ref_point.x_);
      path_point.set_y(ref_point.y_);
      path_point.set_theta(ref_point.heading());
      path_point.set_kappa(ref_point.kappa());
      path_point.set_dkappa(ref_point.dkappa());

      if (!path_points.empty())
      {
        double dx = path_point.x - path_points.back().x;
        double dy = path_point.y - path_points.back().y;
        s += std::sqrt(dx * dx + dy * dy);
      }
      path_point.set_s(s);
      path_points.push_back(std::move(path_point));
    }
    return path_points;
}

lattice::lattice(){
    ROS_INFO("lattice plan start");
    // ros parameter settings
    ros::param::get("use_lateral_optimization", this->FLAGS_lateral_optimization);

    // subscriber
    referenceLine_subscriber_ = n_.subscribe("/referenceLine_smoothed", 10, &lattice::referenceLineCallback, this);

    gps_sub_ = n_.subscribe("/gps", 10, &lattice::gpsCallback, this);

    

    // publisher
    trajectory_pub_ = n_.advertise<msg_gen::trajectory>("/trajectory_waypoints", 10);        //发布局部轨迹
    // trajectory_pub_ = n_.advertise<nav_msgs::Path>("/trajectory_waypoints", 10);        //发布局部轨迹

    
    
}


void lattice::referenceLineCallback(const nav_msgs::Path &path_point)
{
    
    
    // 计算参考点的kappa、theta，只需要计算一次
    if (path_point.poses.size() > 0 && path_point_flag_.size() < 1)
    {
      reference_path.first.clear();
      reference_path.second.clear();
      accumulated_s.clear();
      reference_points.clear();

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

bool lattice::is_update_dynamic(nav_msgs::Path &trj_point_array, int size)
{
  bool is_update;
  is_update = false;
  //车的当前位置
  double pp_x = gps_.posX;
  double pp_y = gps_.posY;
  double xx = trj_point_array.poses[size].pose.position.x;
  double yy = trj_point_array.poses[size].pose.position.y;
  double distance = sqrt(pow(pp_x - xx, 2) + pow(pp_y - yy, 2));
  if (distance < 1) //接近了
  {
    is_update = true;
  }
  return is_update;
}

void lattice::plan() {
    
    std::vector<const Obstacle *> obstacles; // Apollo是这种类型
    for (size_t i = 0; i < AllObstacle.size(); i++){
      obstacles.emplace_back(&AllObstacle[i]);
    }

    double current_time = (double)(ros::WallTime::now().toSec());
    // ROS_INFO("current_time %f", current_time);
    plan_start_point(current_time);
    //初始参数的输入
    lattice_ic_ = {
        d0,   // 初始的横向偏移值 [m]
        dd0,  // 初始的横向速度 [m/s]
        ddd0, // 初始的横向加速度 [m/s^2]

        s0,                 // 初始的纵向值[m]
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
    std::cout << "x_init: " << x_init << " y_init: " << y_init << std::endl;
    std::cout << "gps_x:  " << gps_.posX << " gps_y:  " << gps_.posY << std::endl;
    // ROS_INFO("current_time2 %f", current_time);
    // 创建起点参数
    std::cout << "lattice_ic_x = " << lattice_ic_.x_init << " lattice_ic_y = " << lattice_ic_.y_init << std::endl;
    TrajectoryPoint planning_init_point(lattice_ic_);
    // 轨迹生成
    // PlanningTarget planning_target(Config_.default_cruise_speed, accumulated_s); // 目标
    PlanningTarget planning_target(30, accumulated_s); // 目标 km/h
    lon_decision_horizon = accumulated_s[accumulated_s.size() - 1];
    best_path_ = LatticePlan(planning_init_point, planning_target, obstacles, accumulated_s, reference_points,
                                     FLAGS_lateral_optimization, init_relative_time, lon_decision_horizon, absolute_time);
    // 发布轨迹
    msg_gen::trajectory trajectory_d;
    trajectory_d.pointsize = best_path_.size();
    for (int i = 0; i < best_path_.size();++i){
      msg_gen::TrajectoryPoint TrajectoryPoint_d;
      TrajectoryPoint_d.x = best_path_[i].x;
      TrajectoryPoint_d.y = best_path_[i].y;
      // std::cout << "x_" << i << " = " << best_path_[i].x << " y_" << i << " = " << best_path_[i].y<< std::endl;
      TrajectoryPoint_d.z = best_path_[i].z;
      TrajectoryPoint_d.theta = best_path_[i].theta;
      TrajectoryPoint_d.kappa = best_path_[i].kappa;
      TrajectoryPoint_d.dkappa = best_path_[i].dkappa;
      TrajectoryPoint_d.v = best_path_[i].v;
      std::cout << "trajectory v" << i <<  " = " << TrajectoryPoint_d.v << std::endl;
      TrajectoryPoint_d.a = best_path_[i].a;
      TrajectoryPoint_d.relative_time = best_path_[i].relative_time;
      TrajectoryPoint_d.absolute_time = best_path_[i].absolute_time;
      TrajectoryPoint_d.d = best_path_[i].d;
      TrajectoryPoint_d.d_d = best_path_[i].d_d;
      TrajectoryPoint_d.d_dd = best_path_[i].d_dd;
      TrajectoryPoint_d.s = best_path_[i].s;
      TrajectoryPoint_d.s_d = best_path_[i].s_d;
      TrajectoryPoint_d.s_dd = best_path_[i].s_dd;
      TrajectoryPoint_d.s_ddd = best_path_[i].s_ddd;
      TrajectoryPoint_d.d_ddd = best_path_[i].d_ddd;
      trajectory_d.trajectorypoint.emplace_back(TrajectoryPoint_d);
    }
    ROS_INFO("trajectory_d.size %d", trajectory_d.pointsize);

    traj_points_.poses.clear();
    traj_points_.header.frame_id = "world";
    traj_points_.header.stamp = ros::Time::now();
    for (int i = 0; i < best_path_.size(); i++)
    {
      geometry_msgs::PoseStamped pose_stamp;
      pose_stamp.header.frame_id = "world";
      pose_stamp.header.stamp = ros::Time::now();
      pose_stamp.pose.position.x = best_path_[i].x;
      pose_stamp.pose.position.y = best_path_[i].y;
      pose_stamp.pose.position.z = 0;
      traj_points_.poses.push_back(pose_stamp);
    }

    trajectory_pub_.publish(trajectory_d);

    int update_pos = 10;
    if (is_update_dynamic(traj_points_, update_pos+8)){
      s0 = best_path_[update_pos].s;
      ds0 = best_path_[update_pos].s_d;
      dds0 = best_path_[update_pos].s_dd;
      d0 = best_path_[update_pos].d;
      dd0 = best_path_[update_pos].d_d;
      ddd0 = best_path_[update_pos].d_dd;

      init_relative_time = 0; // best_path_[update_pos].relative_time，动态障碍物的起始时间跟这个保持一致
      x_init = best_path_[update_pos].x;
      y_init = best_path_[update_pos].y;
      z_init = 0;
      v_init = best_path_[update_pos].v;
      a_init = best_path_[update_pos].a;

      theta_init = best_path_[update_pos].theta;
      kappa_init = best_path_[update_pos].kappa;
      dkappa_init = best_path_[update_pos].dkappa;
    }

}

void lattice::plan_start_point(double &current_time) {
    if (this->is_first_run){
        // 第一次运行
        std::cout << "第1次运行" << std::endl;
        this->is_first_run = false;
        x_init = gps_.posX;// 定位
        y_init = gps_.posY;
        z_init = 0;
        v_init = std::sqrt(gps_.velX*gps_.velX + gps_.velY*gps_.velY);
        a_init = std::sqrt(gps_.accelX*gps_.accelX + gps_.accelY*gps_.accelY);
        theta_init = gps_.oriZ;
        kappa_init = 0;
        dkappa_init = 0;
        s0 = 0;
        ds0 = 0;
        dd0 = 0;
        d0 = 0;
        dds0 = 0;
        ddd0 = 0;
        init_relative_time = 0.0;
        absolute_time = current_time + 0.1;// start point absolute time
    }
    else if (best_path_.size() < 0)
    {
        // 非第一次运行
        std::cout << "非第一次运行且有历史轨迹" << std::endl;
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
        // std::cout << "best_path_.size: " << best_path_.size() << std::endl;
        // ROS_INFO("current_time1 %f", current_time);
        ROS_INFO("best_path_[0].absolute_time %f", best_path_[0].absolute_time);
        ROS_INFO("best_path_[1].absolute_time %f", best_path_[1].absolute_time);
        ROS_INFO("best_path_[best_path_.size()-1].absolute_time %f", best_path_[best_path_.size()-1].absolute_time);
        for (int i = 0; i < best_path_.size()- 1;++i){
            // 感觉不应该会进这个循环？
            // std::cout << "best_path_.size1: " << best_path_.size() << std::endl;
            if ( best_path_[i].absolute_time <= current_time &&  best_path_[i + 1].absolute_time > current_time){
                index = i;
                std::cout << "match point index: " << i << std::endl;
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
        printf("pre_x_desire = %f 米\n", pre_x_desire);// 局部路径
        printf("pre_y_desire = %f 米\n", pre_y_desire);
        printf("lon_err = %f 米\n", lon_err);
        printf("lat_err = %f 米\n", lat_err);
        // 如果纵向误差大于2.5或者横向误差大于0.5认为控制没跟上
        if (lon_err > 2.5 || lat_err > 0.5){
            ROS_INFO("out of control, use dynamic to deduce");
            x_init = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
            y_init = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
            z_init = 0;
            v_init = std::sqrt(std::pow(vy_cur + ay_cur * dt, 2)+std::pow(vx_cur + ax_cur * dt, 2));
            a_init = std::sqrt(ax_cur*ax_cur + ay_cur*ay_cur);
            theta_init = atan2(vy_cur + ay_cur * dt, vx_cur + ax_cur * dt);
            kappa_init = kappa_cur;
            dkappa_init = 0;
            s0 = 0;
            ds0 = 0;
            dd0 = 0;
            d0 = 0;
            dds0 = 0;
            ddd0 = 0;
            
            absolute_time = current_time + 0.1;
        }
        else {
          ROS_INFO("good control");
          int index_good = 0;
          for (int i = 0; i < best_path_.size() - 1; ++i){
            if (best_path_[i].absolute_time <= current_time+0.1 && best_path_[i + 1].absolute_time > current_time+0.1)
            {
              index_good = i;
              std::cout << "plan start point index: " << i << std::endl;
              break;
            }
          }
          x_init = best_path_[index_good].x;
          y_init = best_path_[index_good].y;
          z_init = 0;
          v_init = best_path_[index_good].v;
          a_init = best_path_[index_good].a;
          theta_init = best_path_[index_good].theta;
          kappa_init = best_path_[index_good].kappa;
          dkappa_init = best_path_[index_good].dkappa;
          // 上一帧轨迹的速度和加速度是指轨迹的切向速度，切向加速度
          // 计算轨迹在current_time + 100ms这个点的切向法向量
          // tor << cos(theta_init), sin(theta_init);
          // nor << -sin(theta_init), cos(theta_init);
          // Eigen::Matrix<double, 2, 1> a_tor = best_path_[index_good].a * tor;
          // Eigen::Matrix<double, 2, 1> a_nor = best_path_[index_good].v * best_path_[index_good].v * kappa_init * nor;
          // dds0 = a_tor[0] + a_nor[0];
          // ddd0 = a_tor[1] + a_nor[1];
          s0 = best_path_[index_good].s;
          ds0 = best_path_[index_good].s_d;
          dd0 = best_path_[index_good].d_d;
          d0 = best_path_[index_good].d;
          dds0 = best_path_[index_good].s_dd;
          ddd0 = best_path_[index_good].d_dd;
          absolute_time = best_path_[index_good].absolute_time;
        }
    }
}

DiscretizedTrajectory lattice::LatticePlan(
    const TrajectoryPoint &planning_init_point,
    const PlanningTarget &planning_target,
    const std::vector<const Obstacle *> &obstacles,
    const std::vector<double> &accumulated_s,
    const std::vector<ReferencePoint> &reference_points, const bool &lateral_optimization,
    const double &init_relative_time, const double lon_decision_horizon, const double &absolute_time)
{
  //ROS_WARN("start_lattice");
  DiscretizedTrajectory Optim_trajectory;
  // 1. compute the matched point of the init planning point on the reference line.经过投影后的匹配点
  ReferencePoint matched_point = PathMatcher::MatchToPath(reference_points, planning_init_point.path_point().x,
                                                          planning_init_point.path_point().y);

  // 2. according to the matched point, compute the init state in Frenet frame.
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);
  std::cout << "init_s[0] = " << init_s[0] << " init_d[0] = " << init_d[0] << std::endl;
  // 与Apollo不同，我们的前探距离不加上init_s[0]，因为我们的仿真的参考线不长。设置障碍物，存储SL ST图
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(obstacles, reference_points, init_s[0],
                                                             init_s[0] + 20, //前瞻多少m lon_decision_horizon
                                                             0.0, Config_.FLAGS_trajectory_time_length, init_d);

  auto ptr_reference_line = std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(reference_points));
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(obstacles, ptr_reference_line);// 将障碍物按照id 存储为map容器(key, value)

  // 3.生成纵向和横向轨迹
  Trajectory1dGenerator trajectory1d_generator(init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  std::cout << "number_lon_traj = " << lon_trajectory1d_bundle.size();
  std::cout << "  number_lat_traj = " << lat_trajectory1d_bundle.size() << "\n";

  // 4.计算每条轨迹的代价,并得出优先级队列
  TrajectoryEvaluator trajectory_evaluator(planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
                                           init_s, ptr_path_time_graph, reference_points);

  // 5.轨迹拼接和最后的筛选
  while (trajectory_evaluator.has_more_trajectory_pairs())
  {
    double trajectory_pair_cost = trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = trajectorycombiner_.Combine(accumulated_s, *trajectory_pair.first, *trajectory_pair.second,
                                                          reference_points, init_relative_time, absolute_time);

    // 采样时候才调用，二次规划不用
    if (lateral_optimization == false)
    {
      // check longitudinal and lateral acceleration
      // considering trajectory curvatures
      auto result = constraintchecker_.ValidTrajectory(combined_trajectory);
      if (result != ConstraintChecker::Result::VALID)
      {
        switch (result)
        {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
        }
        continue;
      }
      // Get instance of collision checker and constraint checker
      CollisionChecker collision_checker(obstacles, init_s[0], init_d[0], reference_points, ptr_path_time_graph);
      //碰撞检测
      if (collision_checker.InCollision(combined_trajectory))
      {
        ROS_INFO("collision");
        continue;
      }
    }
    Optim_trajectory = std::move(combined_trajectory);
    std::cout << "Total_Trajectory_Cost = " << trajectory_pair_cost << "\n";
    break;
  }
  ROS_WARN("trj_num :%d",Optim_trajectory.size());
  return Optim_trajectory;
}

void lattice::pubTrajectory(DiscretizedTrajectory best_path){
  
}
void lattice::ComputeInitFrenetState(const ReferencePoint &matched_point, const TrajectoryPoint &cartesian_state,
                            std::array<double, 3> *ptr_s, std::array<double, 3> *ptr_d)
{
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.accumulated_s_, matched_point.x_, matched_point.y_,
      matched_point.heading_, matched_point.kappa_, matched_point.dkappa_,
      cartesian_state.path_point().x, cartesian_state.path_point().y,
      cartesian_state.v, cartesian_state.a,
      cartesian_state.path_point().theta,
      cartesian_state.path_point().kappa, ptr_s, ptr_d);
}

} // namespace lattice
} // namespace dust