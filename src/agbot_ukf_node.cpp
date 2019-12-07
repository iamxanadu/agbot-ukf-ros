#include "agbot_simulinkblock_20191202_initialize.h"
#include <agbot_ukf/StampedInt.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ukf.h>
#include <ekf.h> //TODO (josef) Had to use an EKF with auto diff. Why is this? UKF should work

#include "agbot-ukf.h"

// Names of topics to publish and subscribe on
std::string top_imu_raw;
std::string top_imu_filtered;
std::string top_l_motor_rate, top_r_motor_rate;

// Loop rate to try to enforce in Hz
int loop_rate;

ros::Publisher imu_filtered;


spkf::EKF<process_t<double>, observe_t<double>> *ukf;

void imuRawCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // ROS_INFO_STREAM_THROTTLE(0.1, "IMU msg " << std::endl
    //                                          << msg->linear_acceleration
    //                                          << std::endl
    //                                          << msg->angular_velocity);

    ObsT<double> observ;
    observ[0] = msg->linear_acceleration.x;
    observ[1] = msg->linear_acceleration.y;
    observ[2] = msg->linear_acceleration.z;
    observ[3] = msg->angular_velocity.x;
    observ[4] = msg->angular_velocity.y;
    observ[5] = msg->angular_velocity.z;

    // ROS_INFO_STREAM_THROTTLE(0.1, "IMU Obs " << std::endl
    //                                          << observ);

    ukf->innovate(observ, ObsT<double>::Zero());
    ukf->update();

    // Republished filtered message
    sensor_msgs::Imu nimu;
    nimu.header = msg->header;

    double VARp[14];
    double fax, fay, faz, fwx, fwy, fwz;
    agbot_simulinkblock_20191202(ul, ur, ukf->state().data(), VARp, &fax, &fay,
                                 &faz, &fwx, &fwy, &fwz);

    nimu.angular_velocity.x =
        fwx; // TODO(josef) these may be in the wrong order because roll pitch yaw
             // do not specify the imu axes. Might be fixed b/c using model funct.
    nimu.angular_velocity.y = fwy;
    nimu.angular_velocity.z = fwz;
    nimu.linear_acceleration.x = fax;
    nimu.linear_acceleration.y = fay;
    nimu.linear_acceleration.z = faz;

    imu_filtered.publish(nimu); // Publish filtered IMU
}

// TODO (josef) might want to sync these up later. Didn't seem to work first
// time I tried.
void leftSpeedCallback(const agbot_ukf::StampedInt::ConstPtr &msg)
{
    ul = msg->data * -2 * M_PI / (1.079 * 2400);
    // ROS_INFO_STREAM_THROTTLE(0.1, "Left encoder " << ul);
}

void rightSpeedCallback(const agbot_ukf::StampedInt::ConstPtr &msg)
{
    ur = msg->data * 2 * M_PI / (1.070 * 2400);
    // ROS_INFO_STREAM_THROTTLE(0.1, "Right encoder " << ur);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ukf");

    agbot_simulinkblock_20191202_initialize();

    // ROS NodeHandle
    ros::NodeHandle nh;

    // Get parameters for topic names
    nh.param<std::string>("IMU_RAW_TOPIC", top_imu_raw, "/camera/imu");
    nh.param<std::string>("IMU_FILTERED_TOPIC", top_imu_filtered, "/ukf/imu");
    nh.param<std::string>("L_MOTOR_RATE_TOPIC", top_l_motor_rate, "/speed1");
    nh.param<std::string>("R_MOTOR_RATE_TOPIC", top_r_motor_rate, "/speed2");

    nh.param<int>("LOOP_RATE", loop_rate, 1000);

    // ROS subscribers and publishers
    ros::Subscriber imu_raw = nh.subscribe(top_imu_raw, 1, imuRawCallback);
    ros::Subscriber l_motor_rate =
        nh.subscribe(top_l_motor_rate, 1, leftSpeedCallback);
    ros::Subscriber r_motor_rate =
        nh.subscribe(top_r_motor_rate, 1, rightSpeedCallback);
    imu_filtered = nh.advertise<sensor_msgs::Imu>(top_imu_filtered, 100);

    // Create intial values for the UKF
    StateT<double> state = StateT<double>::Zero();
    // TODO (josef) Magic initial states?
    state[0] = -1.4826 * 0.0174532925199433;
    state[7] = 0.165;

    const auto covar = Eigen::Matrix<double, 14, 14>((Eigen::VectorXd(14) << 100, 100, 100, 10, 10, 10, 10, 10, 300, 300, 300, 200, 200, 200).finished().asDiagonal()); //0.00001 * CovarT<double>::Identity();
    const auto procovar = Eigen::Matrix<double, 14, 14>((Eigen::VectorXd(14) << 0, 0, 0, 0, 0, 0, 0, 0, 300, 300, 300, 0, 0, 0).finished().asDiagonal());
    // const auto obscovar = Eigen::Matrix<double, 6, 6>((Eigen::VectorXd(6) << 0.0008, 0.0008, 0.0008, 0.00002, 0.00002, 0.00002).finished().asDiagonal());
    const auto obscovar = Eigen::Matrix<double, 6, 6>((Eigen::VectorXd(6) << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2).finished().asDiagonal());

    ControlT<double> control;

    StateT<double> proc_noise = StateT<double>::Zero();
    ObsT<double> obs_noise = ObsT<double>::Zero();

    // Create the UKF
    ukf = new spkf::EKF<process_t<double>, observe_t<double>>(state, 0.0001 * covar,
                                                              0.0001 * procovar, obscovar);

    ros::Rate r(loop_rate); // 1000 Hz default loop rate
    while (ros::ok())
    {

        control[0] = ul;
        control[1] = ur;
        ukf->predict(control, 1 / float(loop_rate),
                     proc_noise); // Run a dead reconing step
        ros::spinOnce();          // Will innovate here if get an accel measurement

        ROS_INFO_STREAM_THROTTLE(1, "Current Kalman state " << std::endl
                                                            << ukf->state());
        
        ROS_INFO_STREAM_THROTTLE(1, "Current Kalman gain " << std::endl
                                                            << ukf->kalman_gain());

        bool sleep_success = r.sleep();

        ROS_INFO_STREAM_THROTTLE(0.1, "Was I able to meet the loop rate of "
                                          << loop_rate << " Hz? " << std::boolalpha
                                          << sleep_success);
    }

    return 0;
}