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


// Guidance OCP class ---------------------------------------------------

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


// Control OCP class ---------------------------------------------------

POLYMPC_FORWARD_DECLARATION(/*Name*/ control_ocp, /*NX*/ 14, /*NU*/ 4, /*NP*/ 0, /*ND*/ 0, /*NG*/0, /*TYPE*/ double)

class control_ocp : public ContinuousOCP<control_ocp, Approximation, SPARSE>
{
public:
    ~control_ocp() = default;

    static constexpr double t_start = 0.0;
    static constexpr double t_stop  = CONTROL_HORIZON;

    Eigen::DiagonalMatrix<scalar_t, 14> Q;
    Eigen::DiagonalMatrix<scalar_t, 4> R;
    Eigen::DiagonalMatrix<scalar_t, 14> QN;
    double weight_vertical_angle = 0;

    Eigen::Matrix<scalar_t, 14,1> xs;
    Eigen::Matrix<scalar_t, 4,1> us{0.0, 0.0, 1, 0.0};

    template<typename T>
    inline void dynamics_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                              const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> &d,
                              const T &t, Eigen::Ref<state_t<T>> xdot) const noexcept
    {
        Eigen::Matrix<T, 4,1> input; input << rocket.maxThrust[0]*u(0), rocket.maxThrust[1]*u(1), 0.5*((rocket.maxThrust[2]-rocket.minThrust[2])*u(2) + rocket.maxThrust[2]+rocket.minThrust[2] ), rocket.maxTorque*u(3);

        // -------------- Simulation parameters -------------- -------------
        T g0 = (T)9.81;                             // Earth gravity in [m/s^2]

        Eigen::Matrix<T, 3, 1> J_inv; J_inv << (T)rocket.J_inv[0], (T)rocket.J_inv[1], (T)rocket.J_inv[2]; // Cemter of mass divided by axes inertia

        // -------------- Simulation variables -----------------------------
        T mass = (T)rocket.dry_mass + x(6);                  // Instantaneous mass of the rocket in [kg]

        // Orientation of the rocket with quaternion
        Eigen::Quaternion<T> attitude( x(9), x(6), x(7), x(8));
        Eigen::Matrix<T, 3, 3> rot_matrix = attitude.toRotationMatrix();

        // Z drag --> Big approximation: speed in Z is basically the same between world and rocket
        T drag = (T)(1e6*rocket.drag_coeff[2]*x(5)*x(5)); 
        
        // Force in body frame (drag + thrust) in [N]
        Eigen::Matrix<T, 3, 1> rocket_force; rocket_force << (T)input(0), (T)input(1), (T)(input(2) - drag);

        // Force in inertial frame: gravity
        Eigen::Matrix<T, 3, 1> gravity; gravity << (T)0, (T)0, g0*mass;

        // Total force in inertial frame [N]
        Eigen::Matrix<T, 3, 1> total_force;  total_force = rot_matrix*rocket_force - gravity;

        // Angular velocity omega in quaternion format to compute quaternion derivative
        Eigen::Quaternion<T> omega_quat((T)0.0, x(10), x(11), x(12));
        
        // X, Y force and Z torque in body frame   
        Eigen::Matrix<T, 3, 1> rocket_torque; rocket_torque << input(1), -input(0), input(3);
        
        
        // -------------- Differential equation ---------------------

        // Position variation is speed
        xdot.head(3) = x.segment(3,3);

        // Speed variation is Force/mass
        xdot.segment(3,3) = (T)1e-3*total_force/mass;  

        // Quaternion variation is 0.5*w◦q
        xdot.segment(6, 4) =  (T)0.5*(omega_quat*attitude).coeffs();

        // Angular speed variation is Torque/Inertia
        xdot.segment(10, 3) = rot_matrix*(rocket_torque.cwiseProduct(J_inv));

        // Mass variation is proportional to thrust
        xdot(13) = -input(2)/((T)rocket.Isp*g0);
    }

    template<typename T>
    inline void lagrange_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                   const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                   const scalar_t &t, T &lagrange) noexcept
    {                 
        Eigen::Matrix<T,14,14> Qm = Q.toDenseMatrix().template cast<T>();
        Eigen::Matrix<T,4,4> Rm = R.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,14,1> x_error = x - xs.template cast<T>();
        Eigen::Matrix<T,4,1> u_error = u - us.template cast<T>();

        T cos_vertical_angle = x(9)*x(9) + x(8)*x(8) - x(7)*x(7) -x(6)*x(6);
         

        lagrange = x_error.dot(Qm * x_error) + u_error.dot(Rm * u_error) + (cos_vertical_angle-1)*(cos_vertical_angle-1)*weight_vertical_angle;

    }

    template<typename T>
    inline void mayer_term_impl(const Eigen::Ref<const state_t<T>> x, const Eigen::Ref<const control_t<T>> u,
                                const Eigen::Ref<const parameter_t<T>> p, const Eigen::Ref<const static_parameter_t> d,
                                const scalar_t &t, T &mayer) noexcept
    {
        Eigen::Matrix<T,14,14> Qm = QN.toDenseMatrix().template cast<T>();
        
        Eigen::Matrix<T,14,1> x_error = x - xs.template cast<T>();
                
        mayer = x_error.dot(Qm * x_error);
    }
};



#endif //SRC_ROCKET_MPC_HPP
