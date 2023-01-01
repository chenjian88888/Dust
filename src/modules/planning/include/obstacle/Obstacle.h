#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <uuid_msgs/UniqueID.h>
#include "Obstacle_avoid.h"
#include "math_utils.h"
#include "ObjectDecision.h"
#include "boundarys.h"
#include "msg_gen/obstacle.h"

using namespace Eigen;

//障碍物的类，无非就是id、长宽高(可不用，换成4个顶点位置)、中心位置以及朝向
class Obstacle
{
public:
  Obstacle() = default;
  Obstacle(bool sub_obstacle);
  ~Obstacle() = default;

  void Ego_position(const nav_msgs::Odometry &msg);
  void Ego_theta(const geometry_msgs::PoseArray &theta);
  /**
   * @brief Set the Obstacles object 感知回调得到的障碍物的信息
   * 
   * @param msgs 
   */
  void setObstacles(const msg_gen::obstacle::ConstPtr &msgs);

  void SetSLBoundary(SL_Boundary &sl_boundary);
  void SetSTBoundary(ST_Boundary &st_boundary);
  const SL_Boundary &PerceptionSLBoundary() const;
  const ST_Boundary &path_st_boundary() const;
  bool IsStatic() const;
  bool IsVirtual() const;

  bool HasTrajectory() const;
  const ObjectDecisionType &LongitudinalDecision() const;
  const ObjectDecisionType &LateralDecision() const;
  bool HasLateralDecision() const;
  bool HasLongitudinalDecision() const;
  Box2d PerceptionBoundingBox() const;
  Box2d GetBoundingBox(const TrajectoryPoint &point) const;
  const common::math::Polygon2d &PerceptionPolygon() const;

  TrajectoryPoint GetPointAtTime(const double relative_time) const;

  void Show_ob_prediction(geometry_msgs::Pose &obstacle_trajectory_, const TrajectoryPoint trajectory_point_);
  void visualization(const std::vector<Obstacle> Obstacles_);

  const Prediction::Ob_Trajectory &Trajectory() const
  {
    return trajectory_;
  }

  void SetTrajectory(const Prediction::Ob_Trajectory trajectory)
  {
    trajectory_ = trajectory;
  }

private:
  SL_Boundary sl_boundary_;
  ST_Boundary path_st_boundary_;
  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;
  Prediction::Ob_Trajectory trajectory_; //动态障碍物的预测轨迹
  common::math::Polygon2d perception_polygon_;
  Box2d perception_bounding_box_;
 
  double eff_length;
  double eff_width;
  double eff_dis;
  std::vector<std::pair<double, double>> pinnacles;

  // handle
  ros::NodeHandle nObstacle_;
  // Publisher和Subscriber
  ros::Subscriber obstacle_sub_;
  ros::Publisher Obstacle_Prediction;

  Obstacle_avoid oba;

public:
  // 障碍物的id
  std::string obstacle_id;
  //障碍物的类型
  /*
    uint8 UNKNOWN=0
    uint8 CAR=1
    uint8 TRUCK=2
    uint8 BUS=3
    uint8 BICYCLE=4
    uint8 MOTORBIKE=5
    uint8 PEDESTRIAN=6
    uint8 ANIMAL=7
  */
  int obstacle_type;
  //障碍物的形状
  /*
    uint8 BOUNDING_BOX=0
    uint8 CYLINDER=1
    uint8 POLYGON=2
  */
  int obstacle_shape;
  //障碍物4个顶点位置, 两种表达方式
  geometry_msgs::PoseArray pinnacle;
  std::vector<Vec2d> polygon_points;

  //中心点位置
  geometry_msgs::Pose centerpoint;
  //障碍物的速度
  double obstacle_velocity;
  //障碍物朝向
  double obstacle_theta;

  //扣一圈的半径大小，适用于frenet下的圆形和方形
  double obstacle_radius;

  //障碍物的长宽高
  double obstacle_length;
  double obstacle_width;
  double obstacle_height;

  //障碍物标签
  ObjectDecisionType decision_;

  //时间戳
  ros::Time timestamp_;
};

extern std::vector<Obstacle> AllObstacle;

#endif // OBSTACLE_H