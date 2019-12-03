#include <iostream>

#include <Eigen/Eigen>

#include "matlab_autocode/AgBot_simulinkfunction.h"

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
        const double VAR[14] = *state_k.data();
        double VARp[14];
        double vrel1_Nx, vrel4_Nx, vrel5_Nx, vrel8_Nx;
        AgBot_simulinkfunction(q_lmdt, q_rmdt, VAR, VARp, &vrel1_Nx, &vrel4_Nx, &vrel5_Nx, &vrel8_Nx);

        // Use state derivative to calculate linear approximation of state transition over interval del_k
        state_t del_state;
        &del_state.data() = VARp;
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

        

        obs_k += obs_noise_k; /* z_k = z_k + r (additive observation noise) */

        return true;
    }
};
