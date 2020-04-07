/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/autonomy/MoveToGoalMS/MoveToGoalMS.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/common/CSV.h>
//MDH additions for LQR controller
#include <ct/optcon/optcon.h>  // also includes ct_core
#include <ct/core/core.h>
#include </home/uav/LQR_Work/src/control-toolbox/ct_optcon/examples/exampleDir.h>
#include </home/uav/trafficstop-integration/submodules/gtri-uav/submodules/LQR_Work/src/my_ct_project/main/droneSystem.h>

#include <iostream>
#include <limits>
#include <cmath>
#include <list>

#include <GeographicLib/LocalCartesian.hpp>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::motor_schemas::MoveToGoalMS,
                MoveToGoalMS_plugin)

namespace scrimmage {
namespace autonomy {
namespace motor_schemas {

MoveToGoalMS::MoveToGoalMS() {
}

void MoveToGoalMS::init(std::map<std::string, std::string> &params) {
    if (sc::get("use_initial_heading", params, false)) {
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX()*1e6;
        Eigen::Vector3d unit_vector = rel_pos.normalized();
        unit_vector = state_->quat().rotate(unit_vector);
        wp_local_ = state_->pos() + unit_vector * rel_pos.norm();
    } else {
        std::vector<double> goal_vec;
        if (sc::get_vec<double>("goal", params, ", ", goal_vec, 3)) {
            wp_local_ = sc::vec2eigen(goal_vec);
        } else {
            cout << "Failed to parse MoveToGoalMS' initial goal" << endl;
        }
    }
    cout << " Goal Position:" << wp_local_ <<endl;
    
    // OLD Code for PID Controller 
    // Initialize PID controller class
    //speed_pid_.set_parameters(sc::get("p_gain", params, 0.5),
    //                          sc::get("i_gain", params, 0.0),
    //                          sc::get("d_gain", params, 0.0));
    //speed_pid_.set_integral_band(sc::get("integral_band", params, 0.0));
    //speed_pid_.set_setpoint(0.0);


    // Initialize LQR Controller
    // get the state and control input dimension of the oscillator
    // Drone state is a 6 x 6 state matrix set up in droneSystem.h 
    //It is set as xyz pos and velocity state set up as x,vx,y,vy,z,vz
    const size_t state_dim = ct::core::tpl::droneSystem<ct::core::ADCGScalar>::STATE_DIM;

    // Control is 6 x 3 matrix also set up in droneState.h 
    //In this case, the controls are accelerations
    const size_t control_dim = ct::core::tpl::droneSystem<ct::core::ADCGScalar>::CONTROL_DIM;

    // create an auto-differentiable instance of the drone dynamics
    auto drone_sys_ptr = new ct::core::tpl::droneSystem<ct::core::ADCGScalar>(5);
    std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, ct::core::ADCGScalar>> droneDynamics(
        drone_sys_ptr);

    // create an Auto-Differentiation Linearizer with code generation on the quadrotor model
    ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(droneDynamics);

    // compile the linearized model just-in-time
    adLinearizer.compileJIT();

    // define the linearization point around steady state
    // x_g here is the goal state for the LQR. The xyz waypoint positions are input here to set up the LQR
    ct::core::StateVector<state_dim> x_g;
    x_g(0) = wp_local_(0);
    x_g(1) = 0;
    x_g(2) = wp_local_(1);
    x_g(3) = 0;
    x_g(4) = wp_local_(2);
    x_g(5) = 0;

    //This vector takes in state position and veloctiy and combines it into one vector to be used in LQR
    ct::core::StateVector<state_dim> x;
    x(0) = state_->pos()[0];
    x(2) = state_->pos()[1];
    x(4) = state_->pos()[2];
    x(1) = state_->vel()[0];
    x(3) = state_->vel()[1];
    x(5) = state_->vel()[2];
    std::cout<< "X is " <<x <<std::endl;

    //The control here is left blank as it is only being used to initialize and calculate the LQR values 
    ct::core::ControlVector<control_dim> u;
    u.setZero();
    double t = 0.0;
    // compute the linearization around the nominal state using the Auto-Diff Linearizer
    auto A = adLinearizer.getDerivativeState(x, u, t);
    auto B = adLinearizer.getDerivativeControl(x, u, t);
    // load the weighting matrices
    ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
    quadraticCost.loadConfigFile("/home/uav/trafficstop-integration/submodules/gtri-uav/submodules/LQR_Work/src/my_ct_project/main/lqrCost.info", "termLQR", true);
    auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
    auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here
    // design the LQR controller
    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;
   // std::cout << "A: " << std::endl << A << std::endl << std::endl;
   // std::cout << "B: " << std::endl << B << std::endl << std::endl;
   // std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    //std::cout << "R: " << std::endl << R << std::endl << std::endl;
    lqrSolver.compute(Q, R, A, B, K);
    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;
    multiplier = (A - B*K);
    accel(0) = 0;
    accel(1) = 0;
    accel(2) = 0;
    //std::cout <<"Multiplier" << std::endl << multiplier <<std::endl;


       

    // Convert XYZ goal to lat/lon/alt
    double lat, lon, alt;
    parent_->projection()->Reverse(wp_local_(0), wp_local_(1),
                                   wp_local_(2), lat, lon, alt);

    wp_ = Waypoint(lat, lon, alt);
    wp_.set_time(0);
    wp_.set_quat(scrimmage::Quaternion(0, 0, 0));
    wp_.set_position_tolerance(1);
    wp_.set_quat_tolerance(1);

    auto wp_cb = [&] (scrimmage::MessagePtr<Waypoint> msg) {
        wp_ = msg->data;
        parent_->projection()->Forward(wp_.latitude(),
                                       wp_.longitude(),
                                       wp_.altitude(), wp_local_(0),
                                       wp_local_(1), wp_local_(2));
    };
    subscribe<Waypoint>("LocalNetwork", "Waypoint", wp_cb);

    // Write the CSV file to the root log directory
    std::string log_filename = parent_->mp()->log_dir() + "/"
         + "state_log.csv";
    cout << "File saved to " << log_filename << endl;
    // Create the log file and define the headers of the columns
    if (!csv.open_output(log_filename)) {
        cout << "SurroundTarget: Couldn't create output file: "
          << log_filename
          << endl;
    }
    csv.set_column_headers("t, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, accel_x, accel_y, accel_z, speed");
}

bool MoveToGoalMS::step_autonomy(double t, double dt) {
    //double measurement = -(wp_local_ - state_->pos()).norm();
    //double speed_factor = speed_pid_.step(dt, measurement);
    //desired_vector_ = (wp_local_ - state_->pos()).normalized() * speed_factor;
    //Eigen::Vector3d desired_accel_ = (desired_vector_ - state_->vel()) * dt;
    Eigen::VectorXd full_state(6);
    Eigen::VectorXd goal(6);
    full_state(0) = state_->pos()[0];
    full_state(1) = state_->vel()[0] + (accel(0) * dt);
    full_state(2) = state_->pos()[1];
    full_state(3) = state_->vel()[1] + (accel(1) * dt);
    full_state(4) = state_->pos()[2];
    full_state(5) = state_->vel()[2] + (accel(2) * dt);
    std::cout << "state Vec: " << std::endl << full_state <<std::endl;
    goal(0) = wp_local_(0);
    goal(1) = 0;
    goal(2) = wp_local_(1);
    goal(3) = 0;
    goal(4) = wp_local_(2);
    goal(5) = 0;
    //std::cout << "goal Vec: " << std::endl << goal <<std::endl;
    Eigen::VectorXd measurement;
    measurement = full_state - goal;
   // std::cout << "meas Vec: " << std::endl << measurement <<std::endl;
    Eigen::VectorXd x_dot;
    x_dot = multiplier * measurement;
    //std::cout << "x_dot Vec: " << std::endl << x_dot <<std::endl;
    Eigen::MatrixXd x_dot_ref;
//     if (x_dot(1) > 10)
//            x_dot(1) = 10; 
//     if (x_dot(1) < -40)
//            x_dot(1) = -40; 
//     if (x_dot(3) > 10)
//            x_dot(3) = 10; 
//     if (x_dot(3) < -40)
//            x_dot(3) = -40; 
//     if (x_dot(5) > 10)
//            x_dot(5) = 10; 
//     if (x_dot(5) < -40)
//            x_dot(5) = -40; 
//     if (full_state(1) > 55){
//            x_dot(0) = 55;
//     //       x_dot(1) = 0;
//     }
//     if (full_state(3) > 55){
//            x_dot(2) = 55;
//     //       x_dot(3) = 0;
//     }
//     if (full_state(5) > 55){
//            x_dot(4) = 55;
//     //       x_dot(5) = 0;
//     }
    
    desired_vector_(0) = x_dot(0);
    desired_vector_(1) = x_dot(2);
    desired_vector_(2) = x_dot(4);
    accel(0) = x_dot(1);
    accel(1) = x_dot(3);
    accel(2) = x_dot(5);
    auto speed_val = desired_vector_.norm();
    //To avoid causing a NaN value to report to motorschemas, if statement checking if 0 during start and adding very small amount 
    if (desired_vector_(0) == 0.0)
       desired_vector_(0) = 0.000001;
    if (desired_vector_(1) == 0.0)
      desired_vector_(1) = 0.000001;
    std::cout << "Desired Vec: " << std::endl << desired_vector_ <<std::endl;
 
    csv.append(scrimmage::CSV::Pairs{
        {"t", t},
        {"pos_x", state_->pos()[0]},
        {"pos_y", state_->pos()[1]},
        {"pos_z", state_->pos()[2]},
        {"vel_x", state_->vel()[0]},
        {"vel_y", state_->vel()[1]},
        {"vel_z", state_->vel()[2]},
        {"accel_x", accel(0)},
        {"accel_y", accel(1)},
        {"accel_z", accel(2)},
        {"speed", speed_val}});

    return true;
}
} // namespace motor_schemas
} // namespace autonomy
} // namespace scrimmage
