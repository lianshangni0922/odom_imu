#include "odom_predictor/odom_predictor.h"

OdomPredictor::OdomPredictor()
{
// ros::NodeHandle nh_;s
// ros::NodeHandle nh_("~");
  have_bias_= false;
  have_bias_= false;
  seq_ = 0;
  max_imu_queue_length_ = 100;

  constexpr size_t kROSQueueLength = 100;
  imu_sub_ =nh_private_.subscribe("imu", kROSQueueLength, &OdomPredictor::imuCallback, this);
  imu_bias_sub_ = nh_private_.subscribe("imu_bias", kROSQueueLength,&OdomPredictor::imuBiasCallback, this);
  odometry_sub_ = nh_private_.subscribe("odometry", kROSQueueLength,&OdomPredictor::odometryCallback, this);
  odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("predicted_odometry",kROSQueueLength);
  transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>("predicted_transform", kROSQueueLength);
}

void OdomPredictor::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  if (!have_bias_) {
    return;
  }

  // clear old IMU measurements
  while (!imu_queue_.empty() &&
         imu_queue_.front().header.stamp < msg->header.stamp) {
    imu_queue_.pop_front();
  }

  // extract useful information from message
  // tf::poseMsgToKindr(msg->pose.pose, &transform_);
  // float x,y,z;

  Trans[0] = msg->pose.pose.position.x;
  Trans[1] = msg->pose.pose.position.y;
  Trans[2] = msg->pose.pose.position.z;
  // Trans[0] = msg->pose.pose.position.x;
  // Trans[1] = msg->pose.pose.position.y;
  // Trans[2] = msg->pose.pose.position.z;
  Quat.x() = msg->pose.pose.orientation.x;
  Quat.y() = msg->pose.pose.orientation.y;
  Quat.z() = msg->pose.pose.orientation.z;
  Quat.w() = msg->pose.pose.orientation.w;



  pose_covariance_ = msg->pose.covariance;
  // tf::vectorMsgToKindr(msg->twist.twist.linear, &linear_velocity_);
  linear_velocity_[0] = msg->twist.twist.linear.x;
  linear_velocity_[1] = msg->twist.twist.linear.y;
  linear_velocity_[2] = msg->twist.twist.linear.z;

  // tf::vectorMsgToKindr(msg->twist.twist.angular, &angular_velocity_);

  angular_velocity_[0]= msg->twist.twist.angular.x;
  angular_velocity_[1]= msg->twist.twist.angular.y;
  angular_velocity_[2]= msg->twist.twist.angular.z;

  twist_covariance_ = msg->twist.covariance;
  frame_id_ = msg->header.frame_id;
  child_frame_id_ = msg->child_frame_id;

  // reintegrate IMU messages
  estimate_timestamp_ = msg->header.stamp;

  try {
    for (const sensor_msgs::Imu& imu_msg : imu_queue_) {
      integrateIMUData(imu_msg);
    }
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    have_bias_ = false;
    have_odom_ = false;
    imu_queue_.clear();
    return;
  }

  have_odom_ = true;
}

void OdomPredictor::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  if (msg->header.stamp < imu_queue_.back().header.stamp) {
    ROS_ERROR_STREAM("Latest IMU message occured at time: "
                     << msg->header.stamp
                     << ". This is before the previously received IMU "
                        "message that ouccured at: "
                     << imu_queue_.back().header.stamp
                     << ". The current imu queue will be reset.");
    imu_queue_.clear();
  }
  if (imu_queue_.size() > max_imu_queue_length_) {
    ROS_WARN_STREAM_THROTTLE(
        10, "There has been over "
                << max_imu_queue_length_
                << " IMU messages since the last odometry update. The oldest "
                   "measurement will be forgotten. This message is printed "
                   "once every 10 seconds");
    imu_queue_.pop_front();
  }

  imu_queue_.push_back(*msg);

  if (!have_bias_ || !have_odom_) {
    return;
  }

  try {
    integrateIMUData(*msg);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM(
        "IMU INTEGRATION FAILED, RESETING EVERYTHING: " << e.what());
    have_bias_ = false;
    have_odom_ = false;
    imu_queue_.clear();
    return;
  }

  publishOdometry();
  publishTF();
  ++seq_;
}

void OdomPredictor::imuBiasCallback(const sensor_msgs::ImuConstPtr& msg) {
  // tf::vectorMsgToKindr(msg->linear_acceleration,
  //                      &imu_linear_acceleration_bias_);
  imu_linear_acceleration_bias_[0] = msg -> linear_acceleration.x;
  imu_linear_acceleration_bias_[1] = msg -> linear_acceleration.y;
  imu_linear_acceleration_bias_[2] = msg -> linear_acceleration.z;
  // tf::vectorMsgToKindr(msg->angular_velocity, &imu_angular_velocity_bias_);
  imu_angular_velocity_bias_[0] = msg -> angular_velocity.x;
  imu_angular_velocity_bias_[1] = msg -> angular_velocity.y;
  imu_angular_velocity_bias_[2] = msg -> angular_velocity.z;
  have_bias_ = true;
}

void OdomPredictor::integrateIMUData(const sensor_msgs::Imu& msg) {
  const double delta_time = (msg.header.stamp - estimate_timestamp_).toSec();

  const V3 g(0.0, 0.0, -9.81);

  V3 imu_linear_acceleration, imu_angular_velocity;
  // tf::vectorMsgToKindr(msg.linear_acceleration, &imu_linear_acceleration);
  imu_linear_acceleration[0]= msg.linear_acceleration.x;
  imu_linear_acceleration[1]= msg.linear_acceleration.y;
  imu_linear_acceleration[2]= msg.linear_acceleration.z;

  // tf::vectorMsgToKindr(msg.angular_velocity, &imu_angular_velocity);
 imu_angular_velocity[0]= msg.angular_velocity.x;
 imu_angular_velocity[1]= msg.angular_velocity.y;
 imu_angular_velocity[2]= msg.angular_velocity.z;

  // find changes in angular velocity and rotation delta
  const V3 final_angular_velocity =(imu_angular_velocity - imu_angular_velocity_bias_);
  const V3 delta_angle =delta_time * (final_angular_velocity + angular_velocity_) / 2.0;
  angular_velocity_ = final_angular_velocity;

  // apply half of the rotation delta
    // Eigen::Quaterniond delta_q = Eigen::Quaterniond(1, 0.5*w(0), 0.5*w(1), 0.5*w(2));
  const Q det_q = Q(1,delta_angle[0]/2.0,delta_angle[1]/2,delta_angle[2]/2);

  // const Rotation half_delta_rotation = Rotation::exp(delta_angle / 2.0);


  // transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
 Quat = Quat * det_q;


  // find changes in linear velocity and position
  const V3 delta_linear_velocity =delta_time * (imu_linear_acceleration +g-imu_linear_acceleration_bias_);


  Trans+= delta_time*(linear_velocity_ + delta_linear_velocity/2);
      // transform_.getPosition() + 
      // transform_.getRotation().rotate(
      //     delta_time * (linear_velocity_ + delta_linear_velocity / 2.0));

  linear_velocity_ += delta_linear_velocity;

  // apply the other half of the rotation delta
  // transform_.getRotation() = transform_.getRotation() * half_delta_rotation;
  Quat = Quat *det_q;

  estimate_timestamp_ = msg.header.stamp;
}

void OdomPredictor::publishOdometry() {
  if (!have_odom_) {
    return;
  }

  nav_msgs::Odometry msg;

  msg.header.frame_id = frame_id_;
  msg.header.seq = seq_;
  msg.header.stamp = estimate_timestamp_;
  msg.child_frame_id = child_frame_id_;

  // tf::poseKindrToMsg(transform_, &msg.pose.pose);
  // msg->pose.pose.position.x = Trans[0];
  // msg->pose.pose.position.y = Trans[1];
  // msg->pose.pose.position.z = Trans[2];

  // msg->pose.pose.orientation.x = Quat.x();
  // msg->pose.pose.orientation.x = Quat.x();
  // msg->pose.pose.orientation.x = Quat.x();
  // msg->pose.pose.orientation.x = Quat.x();

  // Quat.x() = msg->pose.pose.orientation.x;
  // Quat.y() = msg->pose.pose.orientation.y;
  // Quat.z() = msg->pose.pose.orientation.z;
  // Quat.w() = msg->pose.pose.orientation.w;

  msg.pose.pose.position.x = Trans[0];
  msg.pose.pose.position.y = Trans[1];
  msg.pose.pose.position.z = Trans[2];
  msg.pose.pose.orientation.x = Quat.x();
  msg.pose.pose.orientation.y = Quat.y();
  msg.pose.pose.orientation.z = Quat.z();
  msg.pose.pose.orientation.w = Quat.w();

  msg.pose.covariance = pose_covariance_;

  // tf::vectorKindrToMsg(linear_velocity_, &msg.twist.twist.linear);
  // tf::vectorKindrToMsg(angular_velocity_, &msg.twist.twist.angular);
  msg.twist.covariance = twist_covariance_;

  odom_pub_.publish(msg);
}

void OdomPredictor::publishTF() {
  if (!have_odom_) {
    return;
  }

  geometry_msgs::TransformStamped msg;

  msg.header.frame_id = frame_id_;
  msg.header.seq = seq_;
  msg.header.stamp = estimate_timestamp_;
  msg.child_frame_id = child_frame_id_;
  msg.transform.translation.x = Trans[0];
  msg.transform.translation.y = Trans[1];
  msg.transform.translation.z = Trans[2];

  msg.transform.rotation.x =  Quat.x();
  msg.transform.rotation.y =  Quat.y();
  msg.transform.rotation.z =  Quat.z();
  msg.transform.rotation.w =  Quat.w();


  // tf::transformKindrToMsg(transform_, &msg.transform);

  transform_pub_.publish(msg);
  br_.sendTransform(msg);
}


int main(int argc, char** argv)

 {
  ros::init(argc, argv, "odom_predictor");

  ros::NodeHandle n;

  OdomPredictor op;

  ros::spin();

  return 0;
}
