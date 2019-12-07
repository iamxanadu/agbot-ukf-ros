#include <agbot_ukf/StampedInt.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <matlab_autocode/agbot_simulinkblock_20191202_initialize.h>
#include <spkf/ekf.h> // TODO (josef) Add the option to switch between ekf and ukf. also get ukf working
#include <spkf/ukf.h>

#include "agbot-ukf.h"

/* Names of topics to publish and subscribe on */
std::string top_imu_raw;
std::string top_imu_filtered;
std::string top_l_motor_rate, top_r_motor_rate;

/* Loop rate to try to enforce in Hz */
int loop_rate;

/* Publisher for the filtered IMU messages */
ros::Publisher imu_filtered;

/* Pointer to the KF */
spkf::EKF<process_t<double>, observe_t<double>> *ukf;

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
  ukf->innovate(observ, ObsT<double>::Zero());
  ukf->update();

  /* Create new IMU packet with same header */
  sensor_msgs::Imu nimu;
  nimu.header = msg->header;

  /* Run the agbot model to get filtered IMU measurements */
  double VARp[14];
  double fax, fay, faz, fwx, fwy, fwz;
  agbot_simulinkblock_20191202(ul, ur, ukf->state().data(), VARp, &fax, &fay,
                               &faz, &fwx, &fwy, &fwz);

  /* Send out the new IMU message with the new values */
  nimu.angular_velocity.x = fwx;
  nimu.angular_velocity.y = fwy;
  nimu.angular_velocity.z = fwz;
  nimu.linear_acceleration.x = fax;
  nimu.linear_acceleration.y = fay;
  nimu.linear_acceleration.z = faz;
  imu_filtered.publish(nimu); // Publish filtered IMU
}

/**
 * @brief Callback for recieving messages with the left encoder speed. Updates
 * the "ul" variable with the correct value.
 *
 * @param msg The timestamped int message
 */
void leftSpeedCallback(const agbot_ukf::StampedInt::ConstPtr &msg) {
  ul = msg->data * -2 * M_PI / (1.079 * 2400);

  // ROS_INFO_STREAM_THROTTLE(0.1, "Left encoder " << ul);
}
/**
 * @brief Callback for recieving messages with the right encoder speed. Updates
 * the "ur" variable with the correct value.
 *
 * @param msg The timestamped int message
 */
void rightSpeedCallback(const agbot_ukf::StampedInt::ConstPtr &msg) {
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

  /* Setup subscribers and publishers */
  // TODO (josef) May want to try to get synch policy working for tred speed
  // topics
  ros::Subscriber imu_raw = nh.subscribe(top_imu_raw, 1, imuRawCallback);
  ros::Subscriber l_motor_rate =
      nh.subscribe(top_l_motor_rate, 1, leftSpeedCallback);
  ros::Subscriber r_motor_rate =
      nh.subscribe(top_r_motor_rate, 1, rightSpeedCallback);
  imu_filtered = nh.advertise<sensor_msgs::Imu>(top_imu_filtered, 100);

  /* Create initial states for Kalman filter */
  StateT<double> state = StateT<double>::Zero();
  state[0] = -1.4826 * 0.0174532925199433; // Initial state for pitch
  state[7] = 0.165;                        // Initial state for z position

  /* Setup the covariance for initial state, the process model and the
   * observation model */
  const auto covar = Eigen::Matrix<double, 14, 14>(
      (Eigen::VectorXd(14) << 100, 100, 100, 10, 10, 10, 10, 10, 300, 300, 300,
       200, 200, 200)
          .finished()
          .asDiagonal()); // 0.00001 * CovarT<double>::Identity();
  const auto procovar = Eigen::Matrix<double, 14, 14>(
      (Eigen::VectorXd(14) << 0, 0, 0, 0, 0, 0, 0, 0, 300, 300, 300, 0, 0, 0)
          .finished()
          .asDiagonal());
  const auto obscovar = Eigen::Matrix<double, 6, 6>(
      (Eigen::VectorXd(6) << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2)
          .finished()
          .asDiagonal());

  /* Init the control vector */
  ControlT<double> control;

  /* Addative proc - we use zero here */
  StateT<double> proc_noise = StateT<double>::Zero();

  /* Create the KF here */
  ukf = new spkf::EKF<process_t<double>, observe_t<double>>(
      state, 0.0001 * covar, 0.0001 * procovar, obscovar);

  /* ROS loop rate */
  ros::Rate r(loop_rate);

  /* Loop until dead */
  while (ros::ok()) {

    /* Collect most recent control commands */
    control[0] = ul;
    control[1] = ur;

    /* Run a predict step on the KF */
    ukf->predict(control, 1 / float(loop_rate),
                 proc_noise); // Run a dead reconing step

    /* Process ROS callbacks - will innovate KF here if get an IMU measurement
     */
    ros::spinOnce();

    ROS_INFO_STREAM_THROTTLE(1, "Current Kalman state " << std::endl
                                                        << ukf->state());

    ROS_INFO_STREAM_THROTTLE(1, "Current Kalman gain " << std::endl
                                                       << ukf->kalman_gain());
    /* Sleep for the remainder of the loop rate - send info to say if we are
     * running behind */
    bool sleep_success = r.sleep();

    ROS_INFO_STREAM_THROTTLE(0.1, "Was I able to meet the loop rate of "
                                      << loop_rate << " Hz? " << std::boolalpha
                                      << sleep_success);
  }

  return 0;
}