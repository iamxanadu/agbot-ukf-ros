#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <agbot_ukf/StampedInt.h>
#include <ukf.h>
#include "agbot_simulinkblock_20191202_initialize.h"

#include "agbot-ukf.h"

// Names of topics to publish and subscribe on
std::string top_imu_raw;
std::string top_imu_filtered;
std::string top_l_motor_rate, top_r_motor_rate;

ros::Publisher imu_filtered;

// Current encoder measurements - system input
double ul, ur; // rad/s

spkf::UKF<process_t<double>, observe_t<double>> *ukf;

void imuRawCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    ObsT<double> observ;
    observ[0] = msg->linear_acceleration.x;
    observ[1] = msg->linear_acceleration.y;
    observ[2] = msg->linear_acceleration.z;
    observ[3] = msg->angular_velocity.x;
    observ[4] = msg->angular_velocity.y;
    observ[5] = msg->angular_velocity.z;
    ukf->innovate(observ, ObsT<double>::Zero());
    ukf->update();

    // Republished filtered message
    sensor_msgs::Imu nimu;
    nimu.header = msg->header;

    double VARp[14];
    double ax, ay, az, wx, wy, wz;
    agbot_simulinkblock_20191202(ul, ur, ukf->state().data(), VARp, &ax, &ay, &az, &wx, &wy, &wz);

    nimu.angular_velocity.x = wx; // TODO(josef) these may be in the wrong order because roll pitch yaw do not specify the imu axes. Might be fixed b/c using model funct.
    nimu.angular_velocity.y = wy;
    nimu.angular_velocity.z = wz;
    nimu.linear_acceleration.x = ax;
    nimu.linear_acceleration.y = ay;
    nimu.linear_acceleration.z = az;

    imu_filtered.publish(nimu); // Publish filtered IMU
}

void encoderCallback(const agbot_ukf::StampedInt::ConstPtr &sp1, const agbot_ukf::StampedInt::ConstPtr &sp2)
{
    // TODO (josef) these may be in degrees/s; double check with Marc to find out which the func takes
    ul = sp1->data * -2 * M_PI / (1.079 * 2400);
    ur = sp1->data * 2 * M_PI / (1.070 * 2400);
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
    nh.param<std::string>("L_MOTOR_RATE_TOPIC", top_l_motor_rate, "/encoder1");
    nh.param<std::string>("R_MOTOR_RATE_TOPIC", top_r_motor_rate, "/encoder2");

    // ROS subscribers and publishers
    ros::Subscriber imu_raw = nh.subscribe(top_imu_raw, 1, imuRawCallback);
    message_filters::Subscriber<agbot_ukf::StampedInt> l_motor_rate(nh, top_l_motor_rate, 1);
    message_filters::Subscriber<agbot_ukf::StampedInt> r_motor_rate(nh, top_r_motor_rate, 1); // Control inputs (Int64 is a placeholder for StampedInt)
    imu_filtered = nh.advertise<sensor_msgs::Imu>(top_imu_filtered, 100);
    
    // Approx time sync pol for encoders
    typedef message_filters::sync_policies::ApproximateTime<agbot_ukf::StampedInt, agbot_ukf::StampedInt> EncoderSyncPolicy;

    // Syncronizer
    message_filters::Synchronizer<EncoderSyncPolicy> sync(EncoderSyncPolicy(10), l_motor_rate, r_motor_rate);
    sync.registerCallback(boost::bind(&encoderCallback, _1, _2));

    // Create intial values for the UKF
    StateT<double> state;
    // y p r is zero
    state[0] = 0;
    state[1] = 0;
    state[2] = 0;
    // qlm qrm zero
    state[3] = 0;
    state[4] = 0;
    // x y z zero
    state[5] = 0;
    state[6] = 0;
    state[7] = 0;
    // y p r rates zero
    state[8] = 0;
    state[9] = 0;
    state[10] = 0;
    // velocities zero
    state[11] = 0;
    state[12] = 0;
    state[13] = 0;

    const auto covar = 10.0 * CovarT<double>::Identity();
    const auto procovar = 5.0 * CovarT<double>::Identity();
    const auto obscovar = 3.0 * ObsCovarT<double>::Identity();

    ControlT<double> control;

    StateT<double> proc_noise = StateT<double>::Zero();
    ObsT<double> obs_noise = ObsT<double>::Zero();

    // Create the UKF
    ukf = new spkf::UKF<process_t<double>, observe_t<double>>(state, covar, procovar, obscovar);

    ros::Rate r(100); // 1000 Hz max loop rate
    while (ros::ok())
    {
        control[0] = ul;
        control[1] = ur;
        ukf->predict(control, 0.001, proc_noise);

        ros::spinOnce();

        ROS_INFO_STREAM("Was I able to meet the loop rate? " << std::boolalpha << r.sleep());
    }

    return 0;
}