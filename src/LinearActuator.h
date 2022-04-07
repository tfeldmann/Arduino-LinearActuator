#pragma once
#include <Arduino.h>
#include <MotorDriver.h>
#include <PID.h>

#ifndef LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS
#define LINEAR_ACTUATOR_ROUGH_TOLERANCE_DIGITS (5)
#endif

#ifndef LINEAR_ACTUATOR_FINEPOS_MS
#define LINEAR_ACTUATOR_FINEPOS_MS (2500)
#endif

#ifndef LINEAR_ACTUATOR_CYCLE_MS
#define LINEAR_ACTUATOR_CYCLE_MS (20)
#endif

class LinearActuator
{
public:
    enum State
    {
        IDLE,
        POSITION_ROUGH,
        POSITION_FINE,
        SPEED,
    };

    LinearActuator(MotorDriver *motor, int pin_pos);
    void update();

    // movement
    void moveSpeed(int speed);
    void movePos(int pos);
    void stop();

    State state();
    virtual int speed();
    virtual int position();
    bool targetReached();
    bool targetRoughlyReached();

private:
    MotorDriver *motor_;
    PID pid_;
    int pin_pos_;

    State state_;
    int start_pos_;
    int target_pos_;
    uint32_t fine_positioning_start_;
    uint32_t last_update_;
};
