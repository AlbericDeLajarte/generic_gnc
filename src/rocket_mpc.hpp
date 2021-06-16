#ifndef SRC_ROCKET_MPC_HPP
#define SRC_ROCKET_MPC_HPP

#include "rocket_model.hpp"
#include <math.h>

// Global variable with rocket parameters and useful methods
Rocket rocket;

using namespace std;

typedef std::chrono::time_point<std::chrono::system_clock> time_point;
time_point get_time()
{
    /** OS dependent */
#ifdef __APPLE__
    return std::chrono::system_clock::now();
#else
    return std::chrono::high_resolution_clock::now();
#endif
}

#define POLY_ORDER 5
#define NUM_SEG    2
#define NUM_EXP    1

/** benchmark the new collocation class */
using Polynomial = polympc::Chebyshev<POLY_ORDER, polympc::GAUSS_LOBATTO, double>;
using Approximation = polympc::Spline<Polynomial, NUM_SEG>;

POLYMPC_FORWARD_DECLARATION(/*Name*/ guidance_ocp, /*NX*/ 8, /*NU*/ 1, /*NP*/ 1, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)
using namespace Eigen;

class guidance_ocp : public ContinuousOCP<guidance_ocp, Approximation, DENSE>
{
public:
    ~guidance_ocp() = default;

    double cos_vertical_angle;

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        // -------------- Simulation variables -----------------------------
        T mass = (T)rocket.dry_mass + x(6);                   // Instantaneous mass of the rocekt in [kg]
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]
        Matrix<T, 3, 1> thrust_force; thrust_force << (T)0.0, (T)0.0, (T)cos_vertical_angle*x(7);

        // Inertial forces in 3D [N]: Drag, Gravity and Thrust --------------------

        // Drag is drag_coefficient*speed²
        Eigen::Matrix<T, 3, 1> drag; drag << (T)rocket.drag_coeff[0], (T)rocket.drag_coeff[1], (T)rocket.drag_coeff[2];
        drag = drag.cwiseProduct(x.segment(3,3).cwiseProduct(x.segment(3,3)));
        //std::cout << "MPC Drag: " << -drag.transpose() << " at time: " << t << "\n";

        Eigen::Matrix<T, 3, 1> gravity;
        gravity << (T)0, (T)0, g0*mass;

        // Total force in 3D in [N]
        Eigen::Matrix<T, 3, 1> total_force; 
        if(1) total_force = thrust_force - drag - gravity;
        else total_force = - drag - gravity;

        // -------------- Differential equation ---------------------

        // Position variation is mass
        xdot.head(3) = x.segment(3,3);

        // Speed variation is Force/mass
        xdot.segment(3,3) = total_force/mass;  

        // Mass variation is proportional to thrust
        if(1) xdot(6) = -x(7)/((T)rocket.Isp*g0);  // When parameters are working -> Use if(t<p(0))
        else xdot(6) = 0;
        
        xdot(7) = u(0);
        
        xdot *= p(0);
        
        polympc::ignore_unused_var(t);
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                   const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept
    {
        //lagrange = -1e-1*x(6);
        lagrange = (T)0.0;
    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept
    {   
        mayer = p(0);

        polympc::ignore_unused_var(x);
        polympc::ignore_unused_var(u);
        polympc::ignore_unused_var(d);
        polympc::ignore_unused_var(t);
  
    }
};

#endif //SRC_ROCKET_MPC_HPP
