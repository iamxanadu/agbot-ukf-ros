#include <iostream>

#include <Eigen/Eigen>

#include "agbot_simulinkblock_20191202.h"

// Current encoder measurements - system input
double ul, ur; // rad/s

/*
qPitch = VAR(1);
qRoll = VAR(2);
qYaw = VAR(3);
q_lm = VAR(4);
q_rm = VAR(5);
x = VAR(6);
y = VAR(7);
z = VAR(8);
qPitchDt = VAR(9);
qRollDt = VAR(10);
qYawDt = VAR(11);
xDt = VAR(12);
yDt = VAR(13);
zDt = VAR(14);
*/
template <typename Scalar>
using StateT = typename Eigen::Matrix<Scalar, 14, 1>;

template <typename Scalar>
using CovarT = typename Eigen::Matrix<Scalar, 14, 14>;

template <typename Scalar>
using ControlT = typename Eigen::Matrix<Scalar, 2, 1>;

template <typename Scalar>
using ObsT = typename Eigen::Matrix<Scalar, 6, 1>;

template <typename Scalar>
using ObsCovarT = typename Eigen::Matrix<Scalar, 6, 6>;

/* process model: x_{k+1} = f(x_k, u_k) + q */
template <typename Scalar>
struct process_t
{

    using scalar_t = Scalar;
    using state_t = StateT<scalar_t>;
    using control_t = ControlT<scalar_t>;

    inline bool operator()(Eigen::Ref<state_t> state_k,
                           const Eigen::Ref<const control_t> &control_k,
                           const Eigen::Ref<const state_t> &proc_noise_k,
                           const Scalar del_k) const
    {

        // Calculate the instantaneous state derivative using the cont. model
        double q_lmdt = control_k(0, 0);
        double q_rmdt = control_k(1, 0);
        state_t del_state;
        double *VARp = del_state.data();
        const double *VAR = state_k.data();
        double ax, ay, az, wx, wy, wz;

        // ROS_INFO_STREAM_THROTTLE(0.01, "left enc " << std::endl << q_lmdt);
        // ROS_INFO_STREAM_THROTTLE(0.01, "right enc " << std::endl << q_rmdt);
        // ROS_INFO_STREAM_THROTTLE(0.01, "state " << std::endl << state_k);

        agbot_simulinkblock_20191202(q_lmdt, q_rmdt, VAR, VARp, &ax, &ay, &az, &wx,
                                     &wy, &wz);

        // Prune off small transitions as zero
        // if (q_lmdt == 0.0 && q_rmdt == 0.0) {
        //   for (int i = 0; i < 14; i++) {
        //     if (del_state[i] < 0.05) {
        //       del_state[i] = 0.0;
        //     }
        //   }
        // }

        // ROS_INFO_STREAM_THROTTLE(0.01, "State delta " << std::endl << del_state);
        // ROS_INFO_STREAM_THROTTLE(0.01, "proc noise " << std::endl << proc_noise_k);

        // Use state derivative to calculate linear approximation of state
        // transition over interval del_k
        state_k += del_k * del_state;
        // Optional additive process noise. Probably set this to zero?
        state_k += proc_noise_k; /* x_{k+1} = x_{k+1} + q (additive process noise)*/

        return true;
    }
};

/* observation model: z_k = h(x_k) + r_k */
template <typename Scalar>
struct observe_t
{

    using scalar_t = Scalar;
    using state_t = StateT<scalar_t>;
    using obs_t = ObsT<scalar_t>;

    inline bool operator()(const Eigen::Ref<const state_t> &state_k,
                           Eigen::Ref<obs_t> obs_k,
                           const Eigen::Ref<const obs_t> &obs_noise_k) const
    {

        state_t del_state;
        double *VARp = del_state.data();
        const double *VAR = state_k.data();
        double ax, ay, az, wx, wy, wz;

        // ROS_INFO_STREAM_THROTTLE(0.01, "left enc in obs " << std::endl << ul);
        // ROS_INFO_STREAM_THROTTLE(0.01, "right enc in obs" << std::endl << ur);

        agbot_simulinkblock_20191202(ul, ur, VAR, VARp, &ax, &ay, &az, &wx,
                                     &wy, &wz);
        obs_k[0] = ax;
        obs_k[1] = ay;
        obs_k[2] = az;
        obs_k[3] = wx;
        obs_k[4] = wy;
        obs_k[5] = wz;

        obs_k += obs_noise_k; /* z_k = z_k + r (additive observation noise) */

        return true;
    }
};
