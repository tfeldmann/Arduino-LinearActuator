#include <LinearActuator.h>
#include <MotorDriver.h>

const int PIN_DIRECTION = 13;
const int PIN_PWM = 5;
const int PIN_POS_READ = A5;

DirPwmMotor motor(PIN_DIRECTION, PIN_PWM);
LinearActuator cyl(&motor, PIN_POS_READ);

unsigned long long lastUpdate = 0;

void setup()
{
}

void loop()
{
    if (millis() - lastUpdate > 5000) {
        cyl.movePos(random(100, 900));
        lastUpdate = millis();
    }
    cyl.update();
}
