#ifndef ODOM_PREDICTOR_ODOM_PREDICTOR_H_
#define ODOM_PREDICTOR_ODOM_PREDICTOR_H_

#include <geometry_msgs/TransformStamped.h>
// #include <kindr/minimal/quat-transformation.h>
// #include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

// typedef kindr::minimal::QuatTransformation Transformation;
// typedef kindr::minimal::RotationQuaternion Rotation;
// typedef Transformation::Vector3 Vector3;
typedef Eigen::Vector3f V3;
typedef Eigen::Quaterniond Q;


class OdomPredictor {
 public:
  OdomPredictor();

  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  void imuBiasCallback(const sensor_msgs::ImuConstPtr& msg);

 
  void integrateIMUData(const sensor_msgs::Imu& msg);

  void publishOdometry();

  void publishTF();

  bool have_odom_;
  bool have_bias_;

  // ros::NodeHandle nh_;
  // ros::NodeHandle nh_private_;
  ros::NodeHandle nh_private_;

  ros::Subscriber imu_sub_;
  ros::Subscriber imu_bias_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher transform_pub_;

  tf::TransformBroadcaster br_;

  int max_imu_queue_length_;

  std::list<sensor_msgs::Imu> imu_queue_;

  int seq_;
  std::string frame_id_;
  std::string child_frame_id_;

  ros::Time estimate_timestamp_;
  // Transformation transform_;
  V3 linear_velocity_;
  V3 angular_velocity_;
  V3 Trans;
  Q Quat;

  V3 imu_linear_acceleration_bias_;
  V3 imu_angular_velocity_bias_;

  boost::array<double, 36ul> pose_covariance_;
  boost::array<double, 36ul> twist_covariance_;
};

#endif  // ODOM_PREDICTOR_ODOM_PREDICTOR_H_
