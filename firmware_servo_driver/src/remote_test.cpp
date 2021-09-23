
#include <pigpiod_if2.h>
#include "firmware_servo_driver/pwm_servo_driver.hpp"

uint32_t i2c_flags;
int main(int argc, char * argv[])
{
    int gpio_ = pigpio_start("192.168.1.121", "8888");
    //status = set_mode(gpio_, 2, PI_INPUT);
    //status = set_mode(gpio_, 3, PI_INPUT);
    PWMServoDriver * pwm_ = new PWMServoDriver(gpio_, -1);
    i2c_flags = 0;

    pwm_->setup(1, PCA9685_I2C_ADDRESS, i2c_flags, 0);


    delete(pwm_);
    return 0;
}