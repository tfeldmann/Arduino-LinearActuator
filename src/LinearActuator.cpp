#include "LinearActuator.h"

LinearActuator::LinearActuator(MotorDriver *motor, int pin_pos)
    : motor_(motor),
      pin_pos_(pin_pos),
      state_(State::IDLE),
      last_update_(0)
{
    pid_.setParams(5, 5, 0);
    stop();
    pinMode(pin_pos, INPUT);
}

void LinearActuator::moveSpeed(int speed)
{
    state_ = State::SPEED;
    motor_->setSpeed(speed);
}

void LinearActuator::movePos(int pos)
{
    state_ = State::POSITION_ROUGH;
    start_pos_ = position();
    target_pos_ = pos;
    update();
}

void LinearActuator::stop()
{
    moveSpeed(0);
}

int LinearActuator::speed()
{
    return motor_->speed();
}

int LinearActuator::position()
{
    return 0.5 * (analogRead(pin_pos_) + analogRead(pin_pos_));
}

LinearActuator::State LinearActuator::state()
{
    return state_;
}

void LinearActuator::update()
{
    uint32_t ms = millis();

    int pos = position();
    int err = target_pos_ - pos;

    bool motor_update = false;
    if ((ms - last_update_) >= LINEAR_ACTUATOR_CYCLE_MS)
    {
        motor_update = true;
        last_update_ = ms;
    }

    // rough positioning
    if (state_ == State::POSITION_ROUGH)
    {
        // we must not check for abs(err) <= TOLERANCE here because then if we
        // travel over the tolerance band we never go into fine positioning.

        // start: 100, target: 200, error: 100, 90, 80, ...
        // start: 200, target: 100, error: -100, -90, -80, ...
        if (((target_pos_ >= start_pos_) &&
             (err <= LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS)) ||
            ((target_pos_ <= start_pos_) &&
             (err >= -LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS)))
        {
            state_ = State::POSITION_FINE;
            fine_positioning_start_ = ms;
            pid_.reset();
        }
        else if (motor_update)
        {
            // ramp up to fullspeed in the right direction
            int target_speed = (err > 0) ? 255 : -255;
            int delta = target_speed - motor_->speed();
            int speed = motor_->speed() + constrain(delta, -25, 25);
            motor_->setSpeed(speed);
        }
    }

    // fine positioning
    if (state_ == State::POSITION_FINE)
    {
        if (ms - fine_positioning_start_ > LINEAR_ACTUATOR_FINEPOS_MS)
        {
            state_ = State::IDLE;
            motor_->setSpeed(0);
        }
        else if (motor_update)
        {
            int val = pid_.output(err);
            motor_->setSpeed(val);
        }
    }
}

bool LinearActuator::targetReached()
{
    int err = abs(position() - target_pos_);
    return (state() == LinearActuator::State::IDLE &&
            err <= LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS);
}

bool LinearActuator::targetRoughlyReached()
{
    LinearActuator::State st = state();
    int err = abs(position() - target_pos_);
    return ((st == LinearActuator::State::IDLE ||
             st == LinearActuator::State::POSITION_FINE) &&
            err <= LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS);
}
