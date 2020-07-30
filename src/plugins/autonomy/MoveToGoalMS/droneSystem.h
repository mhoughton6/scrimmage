#pragma once
#include <cmath>
#include <memory>
#include <iostream>
//#include "ControlledSystem.h"

namespace ct {
namespace core {

namespace tpl {
//#include <ct/core/core.h>  // as usual, include CT
// create a class that derives from ct::core::ControlledSystem
template <typename SCALAR>
class droneSystem : public ControlledSystem<6,3, SCALAR>
{
public:
    static const size_t STATE_DIM = 6;
    static const size_t CONTROL_DIM = 3;

    typedef ControlledSystem<6, 3, SCALAR> Base;
    typedef typename Base::time_t time_t;

     //! default constructor
    droneSystem() = delete;

    //! constructor
    droneSystem(int q, std::shared_ptr<Controller<6, 3, SCALAR>> controller = nullptr)
        : ControlledSystem<6, 3, SCALAR>(controller, SYSTEM_TYPE::SECOND_ORDER)
    {
    }
    //! copy constructor
    droneSystem(const droneSystem& arg)
        : ControlledSystem<6, 3, SCALAR>(arg)
    {
    }
    //! deep copy
    droneSystem* clone() const override { return new droneSystem(*this); }
    //! destructor
    virtual ~droneSystem() {}

    //! evaluate the system dynamics
    /*!
	 * @param state current state (position, velocity)
	 * @param t current time (gets ignored)
	 * @param control control action
	 * @param derivative derivative (velocity, acceleration)
	 */
    virtual void computeControlledDynamics(const StateVector<6, SCALAR>& state,
        const time_t& t,
        const ControlVector<3, SCALAR>& control,
        StateVector<6, SCALAR>& derivative) override
    {
        // derivative(0) = state(1) + control(0) * t;
        // derivative(1) = control(0);
        // derivative(2) = state(3) + control(1) * t;
        // derivative(3) = control(1);
        // derivative(4) = state(5) + control(2) * t;
        // derivative(5) = control(2);

        derivative(0) = state(1);
        derivative(1) = control(0);
        derivative(2) = state(3);
        derivative(3) = control(1);
        derivative(4) = state(5);
        derivative(5) = control(2);
    }

};
}
typedef tpl::droneSystem<double> droneSystemboi;
}
}
