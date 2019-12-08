/**
 * @file agbot_kf_node.cpp
 * @author Josef Biberstein (jxb@mit.edu)
 * @brief
 * @version 0.1
 * @date 2019-12-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <agbot_kf/KFState.h>
#include <agbot_kf/StampedInt.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <matlab_autocode/agbot_simulinkblock_20191202_initialize.h>
#include <spkf/ekf.h> // TODO (josef) Add the option to switch between ekf and ukf. also get ukf working
#include <spkf/ukf.h>

#include "agbot_kf.h"

/* Names of topics to publish and subscribe on */
std::string top_imu_raw;
std::string top_imu_filtered;
std::string top_l_motor_rate, top_r_motor_rate;
std::string top_kalman_filter_state;

/* Use UKF or EKF */
std::string filter_type;

/* Loop rate to try to enforce in Hz */
int loop_rate;

/* Rate at which to publish the kf state */
int kf_pub_rate;

/* Scaling factors for covariances (for tuning) */
double init_state_covar_coeff, proc_covar_coeff, obs_covar_coeff;

/* Inputs for the kf from the YAML config */
std::vector<double> init_state(14, 0);
std::vector<double> init_state_covar(14, 0);
std::vector<double> proc_covar(14, 0);
std::vector<double> obs_covar(6, 0);

/* Publisher for the filtered IMU messages */
ros::Publisher imu_filtered_pub;

/* Debug publisher for KF pos and vel estimate */
ros::Publisher kalman_state_pub;

/* Pointer to the KF options */
spkf::UKF<process_t<double>, observe_t<double>> *ukf;
spkf::EKF<process_t<double>, observe_t<double>> *ekf;

/**
 * @brief Publish the kalman filter pose and twist periodically
 *
 */
void kalmanFilterStateCallback(const ros::TimerEvent &event) {
  agbot_kf::KFState msg;

  tf2::Quaternion q;

  if (filter_type == "ukf") {
    msg.pose.position.x = ukf->state()[5];
    msg.pose.position.y = ukf->state()[6];
    msg.pose.position.z = ukf->state()[7];

    q.setRPY(ukf->state()[1], ukf->state()[0], ukf->state()[2]);
    q.normalize();

    tf2::convert(msg.pose.orientation, q);

    msg.twist.linear.x = ukf->state()[11];
    msg.twist.linear.y = ukf->state()[12];
    msg.twist.linear.z = ukf->state()[13];

    msg.twist.angular.x = ukf->state()[8];
    msg.twist.angular.y = ukf->state()[9];
    msg.twist.angular.z = ukf->state()[10];

  } else {
    msg.pose.position.x = ekf->state()[5];
    msg.pose.position.y = ekf->state()[6];
    msg.pose.position.z = ekf->state()[7];

    q.setRPY(ekf->state()[1], ekf->state()[0], ekf->state()[2]);
    q.normalize();

    tf2::convert(msg.pose.orientation, q);

    msg.twist.linear.x = ekf->state()[11];
    msg.twist.linear.y = ekf->state()[12];
    msg.twist.linear.z = ekf->state()[13];

    msg.twist.angular.x = ekf->state()[8];
    msg.twist.angular.y = ekf->state()[9];
    msg.twist.angular.z = ekf->state()[10];
  }

  /* Timestamep header right before we send the msg */
  msg.header.stamp = ros::Time::now();
  kalman_state_pub.publish(msg);
}

/**
 * @brief Callback to recieve raw IMU measurements from the robot or rosbag
 *
 * @param msg The IMU message
 */
void imuRawCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // ROS_INFO_STREAM_THROTTLE(0.1, "IMU msg " << std::endl
  //                                          << msg->linear_acceleration
  //                                          << std::endl
  //                                          << msg->angular_velocity);

  /* Gather measurements from IMU */
  ObsT<double> observ;
  observ[0] = msg->linear_acceleration.x;
  observ[1] = msg->linear_acceleration.y;
  observ[2] = msg->linear_acceleration.z;
  observ[3] = msg->angular_velocity.x;
  observ[4] = msg->angular_velocity.y;
  observ[5] = msg->angular_velocity.z;

  // ROS_INFO_STREAM_THROTTLE(0.1, "IMU Obs " << std::endl
  //                                          << observ);

  /* Innovate and update the KF using this measurement */
  /* Addative obs noise - we use zero here */
  if (filter_type == "ukf") {
    ukf->innovate(observ, ObsT<double>::Zero());
    ukf->update();
  } else {
    ekf->innovate(observ, ObsT<double>::Zero());
    ekf->update();
  }

  /* Create new IMU packet with same header */
  sensor_msgs::Imu nimu;
  nimu.header = msg->header;

  /* Run the agbot model to get filtered IMU measurements */
  double VARp[14];
  double fax, fay, faz, fwx, fwy, fwz;

  if (filter_type == "ukf") {
    agbot_simulinkblock_20191202(ul, ur, ukf->state().data(), VARp, &fax, &fay,
                                 &faz, &fwx, &fwy, &fwz);
  } else {
    agbot_simulinkblock_20191202(ul, ur, ekf->state().data(), VARp, &fax, &fay,
                                 &faz, &fwx, &fwy, &fwz);
  }

  /* Send out the new IMU message with the new values */
  nimu.angular_velocity.x = fwx;
  nimu.angular_velocity.y = fwy;
  nimu.angular_velocity.z = fwz;
  nimu.linear_acceleration.x = fax;
  nimu.linear_acceleration.y = fay;
  nimu.linear_acceleration.z = faz;
  imu_filtered_pub.publish(nimu); // Publish filtered IMU
}

/**
 * @brief Callback for recieving messages with the left encoder speed. Updates
 * the "ul" variable with the correct value.
 *
 * @param msg The timestamped int message
 */
void leftSpeedCallback(const agbot_kf::StampedInt::ConstPtr &msg) {
  ul = msg->data * -2 * M_PI / (1.079 * 2400);

  // ROS_INFO_STREAM_THROTTLE(0.1, "Left encoder " << ul);
}
/**
 * @brief Callback for recieving messages with the right encoder speed. Updates
 * the "ur" variable with the correct value.
 *
 * @param msg The timestamped int message
 */
void rightSpeedCallback(const agbot_kf::StampedInt::ConstPtr &msg) {
  ur = msg->data * 2 * M_PI / (1.070 * 2400);

  // ROS_INFO_STREAM_THROTTLE(0.1, "Right encoder " << ur);
}

int main(int argc, char **argv) {

  /* Initialize ros */
  ros::init(argc, argv, "ukf");

  /* Initialize the agbot model - autocoded from Matlab */
  agbot_simulinkblock_20191202_initialize();

  /* Ros node handle */
  ros::NodeHandle nh;

  /* Get parameters from ROS config YAML file */
  nh.param<std::string>("IMU_RAW_TOPIC", top_imu_raw, "/camera/imu");
  nh.param<std::string>("IMU_FILTERED_TOPIC", top_imu_filtered, "/ukf/imu");
  nh.param<std::string>("L_MOTOR_RATE_TOPIC", top_l_motor_rate, "/speed1");
  nh.param<std::string>("R_MOTOR_RATE_TOPIC", top_r_motor_rate, "/speed2");
  nh.param<int>("LOOP_RATE", loop_rate, 1000);
  nh.param<int>("KF_PUB_RATE", kf_pub_rate, 10);
  nh.param<std::string>("FILTER_TYPE", filter_type, "ekf");
  nh.param<double>("KF_INITIAL_STATE_COVAR_COEFF", init_state_covar_coeff, 1.0);
  nh.param<double>("KF_PROC_COVAR_COEFF", proc_covar_coeff, 1.0);
  nh.param<double>("KF_OBS_COVAR_COEFF", obs_covar_coeff, 1.0);

  if (!nh.getParam("KF_INITIAL_STATE", init_state))
    ROS_ERROR("Failed to get KF_INITIAL_STATE parameter from config");
  if (!nh.getParam("KF_INITIAL_STATE_COVAR", init_state_covar))
    ROS_ERROR("Failed to get KF_INITIAL_STATE_COVAR parameter from config");
  if (!nh.getParam("KF_PROC_COVAR", proc_covar))
    ROS_ERROR("Failed to get KF_PROC_COVAR parameter from config");
  if (!nh.getParam("KF_OBS_COVAR", obs_covar))
    ROS_ERROR("Failed to get KF_OBS_COVAR parameter from config");

  /* Setup subscribers and publishers */
  // TODO (josef) May want to try to get synch policy working for tred speed
  // topics
  ros::Subscriber imu_raw = nh.subscribe(top_imu_raw, 1, imuRawCallback);
  ros::Subscriber l_motor_rate =
      nh.subscribe(top_l_motor_rate, 1, leftSpeedCallback);
  ros::Subscriber r_motor_rate =
      nh.subscribe(top_r_motor_rate, 1, rightSpeedCallback);
  imu_filtered_pub = nh.advertise<sensor_msgs::Imu>(top_imu_filtered, 100);
  kalman_state_pub =
      nh.advertise<agbot_kf::KFState>(top_kalman_filter_state, 10);

  /* Create timer to publish the kalman filter state at a given rate */
  ros::Timer kf_state_timer = nh.createTimer(
      ros::Duration(1 / float(kf_pub_rate)), kalmanFilterStateCallback);

  /* Create initial states for Kalman filter */

  StateT<double> init_state_vec(init_state.data());
  StateT<double> state_covar_diag(init_state_covar.data());
  StateT<double> proc_covar_diag(proc_covar.data());
  ObsT<double> obs_covar_diag(obs_covar.data());

  const auto init_covar_mat = CovarT<double>(state_covar_diag.asDiagonal());
  const auto proc_covar_mat = CovarT<double>(proc_covar_diag.asDiagonal());
  const auto obs_covar_mat = ObsCovarT<double>(obs_covar_diag.asDiagonal());

  /* Init the control vector */
  ControlT<double> control;

  /* Addative proc - we use zero here */
  StateT<double> proc_noise = StateT<double>::Zero();

  /* Create the KF here */

  if (filter_type == "ukf") {
    ukf = new spkf::UKF<process_t<double>, observe_t<double>>(
        init_state_vec, init_state_covar_coeff * init_covar_mat,
        proc_covar_coeff * proc_covar_mat, obs_covar_coeff * obs_covar_mat);
  } else {
    ekf = new spkf::EKF<process_t<double>, observe_t<double>>(
        init_state_vec, init_state_covar_coeff * init_covar_mat,
        proc_covar_coeff * proc_covar_mat, obs_covar_coeff * obs_covar_mat);
  }

  /* ROS loop rate */
  ros::Rate r(loop_rate);

  /* Loop until dead */
  while (ros::ok()) {

    /* Collect most recent control commands */
    control[0] = ul;
    control[1] = ur;

    /* Run a predict step on the KF */
    if (filter_type == "ukf") {
      ukf->predict(control, 1 / float(loop_rate),
                   proc_noise); // Run a dead reconing step
    } else {
      ekf->predict(control, 1 / float(loop_rate),
                   proc_noise); // Run a dead reconing step
    }

    /* Process ROS callbacks - will innovate KF here if get an IMU measurement
     */
    ros::spinOnce();

    if (filter_type == "ukf") {
      ROS_INFO_STREAM_THROTTLE(1, "Current Kalman state " << std::endl
                                                          << ukf->state());
      ROS_INFO_STREAM_THROTTLE(1, "Current Kalman gain " << std::endl
                                                         << ukf->kalman_gain());
    } else {
      ROS_INFO_STREAM_THROTTLE(1, "Current Kalman state " << std::endl
                                                          << ekf->state());
      ROS_INFO_STREAM_THROTTLE(1, "Current Kalman gain " << std::endl
                                                         << ekf->kalman_gain());
    }

    /* Sleep for the remainder of the loop rate - send info to say if we are
     * running behind */
    bool sleep_success = r.sleep();

    ROS_INFO_STREAM_THROTTLE(0.5, "Was I able to meet the loop rate of "
                                      << loop_rate << " Hz? " << std::boolalpha
                                      << sleep_success);
  }

  return 0;
}