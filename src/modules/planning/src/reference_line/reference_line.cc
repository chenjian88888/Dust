#include "reference_line.h"
#include <iostream>
#include <vector>

namespace dust{
namespace reference_line{

using namespace dust::lattice_ns;

referenceLine::referenceLine(){
    // ros parameter settings
    ros::param::get("which_smoothers", this->which_smoothers);
    ros::param::get("apply_curvature_constraint", this->apply_curvature_constraint);
    // subscriber
    routing_sub_ = n_.subscribe("/routing", 10, &referenceLine::routingCallback, this);

    // publisher
    referenceLine_pub_ = n_.advertise<nav_msgs::Path>("/referenceLine_smoothed", 10); //发布参考线，给局部规划用
}

void referenceLine::routingCallback(const geometry_msgs::PoseArray &routing){
    routing_waypoints_ = Eigen::MatrixXd::Zero(routing.poses.size(), 3);
    // 确保一开始只订阅一次
    if (routing_waypoint_flag_.size() < 1)
    {
        routing_waypoint_flag_.push_back(routing.poses[0].position.x);
        for (int i = 0; i < routing.poses.size(); ++i)
		{
            routing_waypoints_(i, 0) = routing.poses[i].position.x;
            routing_waypoints_(i, 1) = routing.poses[i].position.y;
            routing_waypoints_(i, 2) = routing.poses[i].position.z;
		}
        // 参考线的平滑计算只计算一次
        referenceLine_split(routing_waypoints_);
    }
}

void  referenceLine::run() {
    ROS_INFO("planning start");
    
    
    lattice latti; // 构造lattice类，创造/referenceLine_smoothed话题空间

    ros::Rate loop_rate(10);
    int n = 0;
    while (ros::ok())
    {

        
        /* code */
        // std::cout << "referenceline_.size: " << referenceline_.poses.size() << std::endl;
        if (referenceline_.poses.size() > 0)
        {
            referenceLine_pub_.publish(referenceline_);
        }
        // 确保gps和reference_points收到数据之后再进行
        if (latti.reference_points.size() > 0 && latti.gps_flag_.size() > 0)
        {
          std::cout << "n = " << n << std::endl;
        n++;
            latti.plan();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

void referenceLine::referenceLine_split(Eigen::MatrixXd &hdmap_way_points) {
    average_interpolation(hdmap_way_points, path_point_after_interpolation_, 0.2, 0.6);
    Smooth(path_point_after_interpolation_);
}


void referenceLine::average_interpolation(Eigen::MatrixXd& input, Eigen::MatrixXd& output, double interval_dis,
                                              double distance)// 0.2 0.6
{
  // 1.定义一个容器，类型为Point3d_s,即（x,y,z）
  std::vector<Point3d_s> vec_3d;
  std::vector<Point3d_s> n_vec;
  Point3d_s p;
  // 2.遍历
  // std::cout << " input.rows()" << input.rows() << std::endl;
  for (int i = 0; i < input.rows() - 1; i++)
  {
    double dis = (input.row(i + 1) - input.row(i)).norm();  //求两点的距离，前一行和这一行坐标的距离
    // std::cout << "dis " << dis << std::endl;
    //两点距离太长的话就进行插点
    if (dis >= distance)
    {
      //计算(x,y)两点的距离
      double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) +
                             (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
      //计算角度
      double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
      double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
      //两点之间要插值的插值点的数量
      int num = dis / interval_dis;  //分割了一下
      // std::cout << "num " << num << std::endl;
      //插入点
      for (int j = 0; j < num; j++)
      {
        // i=0,j=0的时候其实是插入起点
        p.x = input(i, 0) + j * interval_dis * cos_a;
        p.y = input(i, 1) + j * interval_dis * sin_a;
        p.z = input(i, 2);
        vec_3d.push_back(p);
      }
    }
    // 3.有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
    else if (dis < distance)
    {
      p.x = input(i, 0);
      p.y = input(i, 1);
      p.z = input(i, 2);
      vec_3d.push_back(p);
    }
  }
  // 4.漏了终点，需要加上
  p.x = input(input.rows() - 1, 0);
  p.y = input(input.rows() - 1, 1);
  p.z = input(input.rows() - 1, 2);
  vec_3d.push_back(p);

  //传给输出矩阵output
  output = Eigen::MatrixXd::Zero(vec_3d.size(), 3);
  int j = 0;
  for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++)
  {
    output(j, 0) = (*it).x;
    output(j, 1) = (*it).y;
    output(j, 2) = (*it).z;
    j++;
  }
}
    
void referenceLine::Smooth(Eigen::MatrixXd &path_point_after_interpolation_)
{
  referenceline_.poses.clear();
  referenceline_.header.frame_id = "world";
  referenceline_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose_stamp;
  pose_stamp.header.frame_id = "world";
  pose_stamp.header.stamp = ros::Time::now();

  //////////////优化参考路径点///////////////
  bool status = false;                                //是否平滑成功标志
  std::vector<std::pair<double, double>> raw_point2d; //参考路径点
  std::vector<double> box_bounds;                     //边界条件
  //获取原始参考路径点和边界值
  for (int i = 0; i < path_point_after_interpolation_.rows(); i++)
  {
    raw_point2d.emplace_back(path_point_after_interpolation_(i, 0), path_point_after_interpolation_(i, 1));
    box_bounds.emplace_back(0.25);
  }
  // fix front and back points to avoid end states deviate from the center of road
  box_bounds.front() = 0.0;
  box_bounds.back() = 0.0;

  //标准化路径点坐标，以第一个点为基准
  NormalizePoints(&raw_point2d);

  // box contraints on pos are used in cos theta smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> bounds = box_bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto &bound : bounds)
  {
    bound *= box_ratio;
  }

  std::vector<std::pair<double, double>> smoothed_point2d;
  std::vector<double> opt_x;
  std::vector<double> opt_y;
  //使用CosThetaSmoother
  if (which_smoothers == true)
  {
    ROS_WARN("CosThetaSmoother");
    status = CosThetaSmooth(raw_point2d, bounds, &opt_x, &opt_y);
  }
  //使用FemPosSmooth
  else if (which_smoothers == false)// 默认是false
  {
    ROS_WARN("FemPosDeviationSmoother");
    status = FemPosSmooth(raw_point2d, bounds, &opt_x, &opt_y);
  }
  if (status == false) //求解失败
  {
    ROS_WARN("reference line smoothing failed!");
  }
  else //求解成功
  {
    for (size_t i = 0; i < opt_x.size(); i++){
      smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
    }
    DeNormalizePoints(&smoothed_point2d);

    //发布
    for (int i = 0; i < smoothed_point2d.size(); i++){
      pose_stamp.pose.position.x = smoothed_point2d[i].first;
      pose_stamp.pose.position.y = smoothed_point2d[i].second;
      pose_stamp.pose.position.z = 0;
      referenceline_.poses.push_back(pose_stamp);
    }
  }
}

void referenceLine::NormalizePoints(std::vector<std::pair<double, double>> *xy_points)
{
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point)
                {
    auto curr_x = point.first;
    auto curr_y = point.second;
    std::pair<double, double> xy(curr_x - zero_x_, curr_y - zero_y_);
    point = std::move(xy); });
}

void referenceLine::DeNormalizePoints(std::vector<std::pair<double, double>> *xy_points)
{
  std::for_each(xy_points->begin(), xy_points->end(), [this](std::pair<double, double> &point)
                {
    auto curr_x = point.first;
    auto curr_y = point.second;
    std::pair<double, double> xy(curr_x + zero_x_, curr_y + zero_y_);
    point = std::move(xy); });
}

bool referenceLine::CosThetaSmooth(const std::vector<std::pair<double, double>>& raw_point2d,
                             const std::vector<double>& bounds, std::vector<double>* opt_x, std::vector<double>* opt_y)
{
  const double weight_cos_included_angle = 10000.0;
  const double weight_anchor_points = 1.0;
  const double weight_length = 1.0;
  const size_t print_level = 0;
  const size_t max_num_of_iterations = 500;
  const size_t acceptable_num_of_iterations = 15;
  const double tol = 1e-8;
  const double acceptable_tol = 1e-1;
  const bool use_automatic_differentiation = true;

  CosThetaIpoptInterface* smoother = new CosThetaIpoptInterface(raw_point2d, bounds);

  smoother->set_weight_cos_included_angle(weight_cos_included_angle);
  smoother->set_weight_anchor_points(weight_anchor_points);
  smoother->set_weight_length(weight_length);
  smoother->set_automatic_differentiation_flag(use_automatic_differentiation);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", static_cast<int>(print_level));
  app->Options()->SetIntegerValue("max_iter", static_cast<int>(max_num_of_iterations));
  app->Options()->SetIntegerValue("acceptable_iter", static_cast<int>(acceptable_num_of_iterations));
  app->Options()->SetNumericValue("tol", tol);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded)
  {
    std::cout << "*** Error during initialization!\n";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level)
  {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    // std::cout << "*** The problem solved in " << iter_count << " iterations!\n";
  }
  else
  {
    std::cout << "Solver fails with return code: " << static_cast<int>(status) << std::endl;
    return false;
  }
  smoother->get_optimization_results(opt_x, opt_y);
  return true;
}


bool referenceLine::FemPosSmooth(const std::vector<std::pair<double, double>>& raw_point2d,
                                    const std::vector<double>& bounds, std::vector<double>* opt_x,
                                    std::vector<double>* opt_y)
{
  if (apply_curvature_constraint){
    if (use_sqp){
      return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
    }
    else{
      return NlpWithIpopt(raw_point2d, bounds, opt_x, opt_y);
    }
  }
  else{
    return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
  return true;
}

bool referenceLine::QpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                                         const std::vector<double>& bounds, std::vector<double>* opt_x,
                                         std::vector<double>* opt_y)
{
  if (opt_x == nullptr || opt_y == nullptr)
  {
    std::cout << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationOsqpInterface solver;

  double weight_fem_pos_deviation = 1.0e10;
  double weight_ref_deviation = 1.0;
  double weight_path_length = 1.0;

  // osqp settings
  int max_iter = 500;
  // time_limit set to be 0.0 meaning no time limit
  double time_limit = 0.0;
  bool verbose = false;
  bool scaled_termination = true;
  bool warm_start = true;

  solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
  solver.set_weight_path_length(weight_path_length);
  solver.set_weight_ref_deviation(weight_ref_deviation);

  solver.set_max_iter(max_iter);
  solver.set_time_limit(time_limit);
  solver.set_verbose(verbose);
  solver.set_scaled_termination(scaled_termination);
  solver.set_warm_start(warm_start);

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve())
  {
    return false;
  }

  *opt_x = solver.opt_x();
  *opt_y = solver.opt_y();
  return true;
}

bool referenceLine::SqpWithOsqp(const std::vector<std::pair<double, double>>& raw_point2d,
                                          const std::vector<double>& bounds, std::vector<double>* opt_x,
                                          std::vector<double>* opt_y)
{
  if (opt_x == nullptr || opt_y == nullptr)
  {
    std::cout << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationSqpOsqpInterface solver;

  double weight_fem_pos_deviation = 1.0e10;
  double weight_ref_deviation = 1.0;
  double weight_path_length = 1.0;
  double weight_curvature_constraint_slack_var = 1.0e2;
  double curvature_constraint = 0.2;
  double sqp_ftol = 1e-4;
  double sqp_ctol = 1e-3;
  int sqp_pen_max_iter = 10;
  int sqp_sub_max_iter = 100;

  // osqp settings
  int max_iter = 500;
  // time_limit set to be 0.0 meaning no time limit
  double time_limit = 0.0;
  bool verbose = false;
  bool scaled_termination = true;
  bool warm_start = true;

  solver.set_weight_fem_pos_deviation(weight_fem_pos_deviation);
  solver.set_weight_path_length(weight_path_length);
  solver.set_weight_ref_deviation(weight_ref_deviation);
  solver.set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);

  solver.set_curvature_constraint(curvature_constraint);

  solver.set_sqp_sub_max_iter(sqp_sub_max_iter);
  solver.set_sqp_ftol(sqp_ftol);
  solver.set_sqp_pen_max_iter(sqp_pen_max_iter);
  solver.set_sqp_ctol(sqp_ctol);

  solver.set_max_iter(max_iter);
  solver.set_time_limit(time_limit);
  solver.set_verbose(verbose);
  solver.set_scaled_termination(scaled_termination);
  solver.set_warm_start(warm_start);

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve())
  {
    return false;
  }

  std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

  // TODO(Jinyun): unify output data container
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i)
  {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }
  return true;
}

bool referenceLine::NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d,
                                           const std::vector<double>& bounds, std::vector<double>* opt_x,
                                           std::vector<double>* opt_y)
{
  if (opt_x == nullptr || opt_y == nullptr)
  {
    std::cout << "opt_x or opt_y is nullptr";
    return false;
  }

  double weight_fem_pos_deviation = 1.0e10;
  double weight_ref_deviation = 1.0;
  double weight_path_length = 1.0;
  double weight_curvature_constraint_slack_var = 1.0e2;
  double curvature_constraint = 0.2;
  // ipopt settings
  int print_level = 0;
  int max_num_of_iterations = 500;
  int acceptable_num_of_iterations = 15;
  double tol = 1e-8;
  double acceptable_tol = 1e-1;

  FemPosDeviationIpoptInterface* smoother = new FemPosDeviationIpoptInterface(raw_point2d, bounds);

  smoother->set_weight_fem_pos_deviation(weight_fem_pos_deviation);
  smoother->set_weight_path_length(weight_path_length);
  smoother->set_weight_ref_deviation(weight_ref_deviation);
  smoother->set_weight_curvature_constraint_slack_var(weight_curvature_constraint_slack_var);
  smoother->set_curvature_constraint(curvature_constraint);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level", static_cast<int>(print_level));
  app->Options()->SetIntegerValue("max_iter", static_cast<int>(max_num_of_iterations));
  app->Options()->SetIntegerValue("acceptable_iter", static_cast<int>(acceptable_num_of_iterations));
  app->Options()->SetNumericValue("tol", tol);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded)
  {
    std::cout << "*** Error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded || status == Ipopt::Solved_To_Acceptable_Level)
  {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    std::cout << "*** The problem solved in " << iter_count << " iterations!";
  }
  else
  {
    std::cout << "Solver fails with return code: " << static_cast<int>(status);
    return false;
  }
  smoother->get_optimization_results(opt_x, opt_y);
  return true;
}



} // namespace reference_line
} // namespace dust
