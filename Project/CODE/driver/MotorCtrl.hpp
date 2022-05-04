#ifndef _MotorCtrl_hpp
#define _MotorCtrl_hpp

#include "Encoder.hpp"
#include "MotorDRV.hpp"
#include "controller/LADRC.hpp"

class MotorCtrl {
    const MotorDRV& _motor;
    const Encoder& _encoder;
    controller::LADRC& _controller;
    controller::T _u_prev, _y_desired;

 public:
    MotorCtrl(const MotorDRV& motor, const Encoder& encoder, controller::LADRC& controller)
        : _motor(motor), _encoder(encoder), _controller(controller) {
        reset();
    }

    void setTargetSpeed(controller::T speed) { _y_desired = speed; }
    controller::T getTargetSpeed() const { return _y_desired; }
    controller::T getOutput() const { return _u_prev; }

    void update() {
        controller::T u = _controller.update(_u_prev, _encoder.get(), _y_desired);
        _u_prev = _motor.setPWM_Limit(u);
    }

    void reset() {
        _controller.reset();
        _u_prev = _y_desired = 0;
    }

    void handleStateChange(bool enable) {
        if (enable) {
            reset();
        } else {
            _motor.setPWM_Limit(0);
        }
    }
};

#endif  // _MotorCtrl_hpp