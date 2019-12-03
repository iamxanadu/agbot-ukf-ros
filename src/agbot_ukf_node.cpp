#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <agbot_ukf/StampedInt.h>

#include <spkf/ukf.h>

#include "agbot-ukf.h"

// Names of topics to publish and subscribe on
std::string top_imu_raw;
std::string top_imu_filtered;
std::string top_l_motor_rate, top_r_motor_rate;

// ROS NodeHandle
ros::NodeHandle nh;

void imuRawCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
}

void encoderCallback(const agbot_ukf::StampedInt::ConstPtr &sp1, const agbot_ukf::StampedInt::ConstPtr &sp2)
{
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ukf");

    // Get parameters for topic names
    nh.param<std::string>("IMU_RAW_TOPIC", top_imu_raw, "/camera/imu");
    nh.param<std::string>("IMU_FILTERED_TOPIC", top_imu_filtered, "/ukf/imu");
    nh.param<std::string>("L_MOTOR_RATE_TOPIC", top_l_motor_rate, "/encoder1");
    nh.param<std::string>("R_MOTOR_RATE_TOPIC", top_r_motor_rate, "/encoder2");

    // ROS subscribers and publishers
    ros::Subscriber imu_raw = nh.subscribe(top_imu_raw, 1, imuRawCallback);
    message_filters::Subscriber<agbot_ukf::StampedInt> l_motor_rate(nh, top_l_motor_rate, 1);
    message_filters::Subscriber<agbot_ukf::StampedInt> r_motor_rate(nh, top_r_motor_rate, 1); // Control inputs (Int64 is a placeholder for StampedInt)
    ros::Publisher imu_filtered = nh.advertise<sensor_msgs::Imu>(top_imu_filtered, 100);

    // Approx time sync pol for encoders
    typedef message_filters::sync_policies::ApproximateTime<agbot_ukf::StampedInt, agbot_ukf::StampedInt> EncoderSyncPolicy;

    // Syncronizer
    message_filters::Synchronizer<EncoderSyncPolicy> sync(EncoderSyncPolicy(10), l_motor_rate, r_motor_rate);
    sync.registerCallback(boost::bind(&encoderCallback, _1, _2));

    // Create intial values for the UKF
    StateT<double> state;
    state[0] = 0;
    

    // Create the UKF
    spkf::UKF<process_t<double>, observe_t<double>> ukf();

    ros::Rate r(1000); // 1000 Hz max loop rate
    while (ros::ok())
    {

        ros::spinOnce();
        ROS_INFO_STREAM("Was I able to meet the loop rate? " << std::boolalpha << r.sleep());
    }

    return 0;
}