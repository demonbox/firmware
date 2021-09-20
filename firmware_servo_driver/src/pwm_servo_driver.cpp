#include <stdexcept>
#include <algorithm>
#include <pigpiod_if2.h>
#include "firmware_servo_driver/pwm_servo_driver.hpp"



//#define ENABLE_DEBUG_OUTPUT

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 */
PWMServoDriver::PWMServoDriver() : gpio_(-1), i2c_(-1) 
{

  
}

/*!
 *  @brief  Instantiates a new PCA9685 PWM driver chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is 0x40
 */
PWMServoDriver::PWMServoDriver(int gpio_, int i2c_) 
{
  this->gpio_ = gpio_;
  this->i2c_ = i2c_;
}


/*!
 *  @brief  设置 I2C 接口和硬件
 *  @param  prescale 设置外部时钟（可选）
 *          
 */
void PWMServoDriver::setup(uint8_t i2c_bus, uint8_t i2c_address, uint8_t prescale)
{


  if ((this->i2c_ = i2c_open(this->gpio_, i2c_bus, i2c_address, 0)) < 0)
  {
    throw std::runtime_error("");
  }
  else
  {
    reset();
    if (prescale) 
    {
      set_external_clock(prescale);
    } 
    else 
    {
      // set a default frequency
      set_pwm_frequency(1000);
    }
    // set the default internal frequency
    set_oscillator_frequency(FREQUENCY_OSCILLATOR);
  }
  return;
}

/*!
 *  @brief  通过 I2C 向 PCA9685 芯片发送复位命令
 */
void PWMServoDriver::reset()
{
  if (i2c_write_byte_data(this->gpio_, this->i2c_ , PCA9685_MODE1, MODE1_RESTART) != 0)
  {
    std::runtime_error("");
  }
  return;
}

/*!
 *  @brief  使板进入睡眠模式
 */
void PWMServoDriver::sleep() 
{
  uint8_t old_mode = i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1);
  uint8_t new_mode = old_mode | MODE1_SLEEP; // set sleep bit high
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, new_mode);
}

/*!
 *  @brief  从睡眠中唤醒板
 */
void PWMServoDriver::wakeup() 
{
  uint8_t old_mode = i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1);
  uint8_t new_mode = old_mode & ~MODE1_SLEEP; // set sleep bit low
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, new_mode);
}

/*!
 *  @brief  设置 EXTCLK 引脚以使用外部时钟
 *  @param  prescale 配置外部时钟使用的预分频值
 *          
 */
void PWMServoDriver::set_external_clock(uint8_t prescale)
{
  uint8_t old_mode = i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1);
  uint8_t new_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, new_mode); // go to sleep, turn off internal oscillator

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, (new_mode |= MODE1_EXTCLK));

  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_PRESCALE, prescale); // set the prescaler

  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, (new_mode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  设置整个芯片的 PWM 频率，高达 ~1.6 KHz
 *  @param  freq 我们将尝试匹配的浮点频率
 */
void PWMServoDriver::set_pwm_frequency(float freq)
{
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif
  // Range output modulation frequency is dependant on oscillator
  float frequency = 0.0;
  if (freq < 1)
    frequency = 1;
  if (freq > 3500)
    frequency = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((this->oscillator_frequency_ / (frequency * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: ");
  Serial.println(prescale);
#endif

  uint8_t old_mode = i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1);
  uint8_t new_mode = (old_mode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, new_mode);                             // go to sleep
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_PRESCALE, prescale); // set the prescaler
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, old_mode);
  // This sets the MODE1 register to turn on auto increment.
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE1, old_mode | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x");
  Serial.println(read8(PCA9685_MODE1), HEX);
#endif
}

/*!
 *  @brief  将 PCA9685 的输出模式设置为
 *          打开排水管或推拉/图腾柱。
 *          警告：集成齐纳二极管的 LED 应该
 *          只能在开漏模式下驱动。
 *  @param  totempole 图腾柱如果为真，如果为假则开漏。
 */
void PWMServoDriver::set_output_mode(bool totempole) 
{
  uint8_t old_mode = i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_MODE2);
  uint8_t new_mode;

  if (totempole) 
  {
    new_mode = old_mode | MODE2_OUTDRV;
  } 
  else 
  {
    new_mode = old_mode & ~MODE2_OUTDRV;
  }
  i2c_write_byte_data(this->gpio_, this->i2c_, PCA9685_MODE2, new_mode);

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting output mode: ");
  Serial.print(totempole ? "totempole" : "open drain");
  Serial.print(" by setting MODE2 to ");
  Serial.println(newmode);
#endif
}

/*!
 *  @brief  从 PCA9685 读取设置预分频
 *  @return 预分频值
 */
uint8_t PWMServoDriver::read_prescale(void)
{
  return i2c_read_byte_data(this->gpio_, this->i2c_, PCA9685_PRESCALE);
}

/*!
 *  @brief  获取 PCA9685 引脚之一的 PWM 输出
 *  @param  num PWM 输出引脚之一，从 0 到 15
 *  @return 请求的 PWM 输出值
 */
uint8_t PWMServoDriver::get_pwm(uint8_t num)
{
  uint32_t buffer = 0;
  i2c_read_block_data(this->gpio_, this->i2c_, PCA9685_LED0_ON_L + 4 * num, reinterpret_cast<char*>(&buffer));
  return buffer;
}

/*!
 *  @brief  设置 PCA9685 引脚之一的 PWM 输出
 *  @param  num PWM 输出引脚之一，从 0 到 15
 *  @param  on 在周期(4096)中的哪个点打开 PWM 输出
 *  @param  off 在周期(4096)中的哪个点关 PWM 输出
 */
void PWMServoDriver::set_pwm(uint8_t num, uint16_t on, uint16_t off) 
{
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM ");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(on);
  Serial.print("->");
  Serial.println(off);
#endif

  char buffer[4] = { 0 };
  if (num <= 15)
  {
    buffer[0] = on & 0xFF;
    buffer[1] = on >> 8;
    buffer[2] = off & 0xFF;
    buffer[3] = off >> 8;
    i2c_write_i2c_block_data(this->gpio_, this->i2c_, PCA9685_LED0_ON_L + 4 * num, buffer, sizeof(buffer));
  }
  else
  {
    buffer[0] = on & 0xFF;
    buffer[1] = on >> 8;
    buffer[2] = off & 0xFF;
    buffer[3] = off >> 8;
    i2c_write_i2c_block_data(this->gpio_, this->i2c_, PCA9685_LED0_ON_L, buffer, sizeof(buffer));
  }
  return;
}

/*!
 *   @brief  帮助设置 PWM 输出引脚。 无需处理即可设置引脚开/关刻度放置并正确处理零值作为完全关闭和4095完全开启。可选的反相参数支持将脉冲反相以接地。
 *   @param  num PWM 输出引脚之一，从 0 到 15
 *   @param  val 4096 中处于活动状态的滴答数，值范围从 0 到 4095（含）。
 *   @param  invert 如果为 true，则反转输出，默认为 'false'
 */
void PWMServoDriver::set_pin(uint8_t num, uint16_t val, bool invert) 
{
  // value between 0 and 4095 inclusive.
  uint16_t value = std::min(val, (uint16_t)4095);

  if (invert) 
  {
    if (value == 0) 
    {
      // 信号完全开启的特定值。
      set_pwm(num, 4096, 0);
    } 
    else if (value == 4095) 
    {
      // 信号完全关闭的特定值。
      set_pwm(num, 0, 4096);
    } 
    else 
    {
      set_pwm(num, 0, 4095 - value);
    }
  } 
  else 
  {
    if (value == 4095) 
    {
      // 信号完全开启的特定值。
      set_pwm(num, 4096, 0);
    } 
    else if (value == 0) 
    {
      // 信号完全关闭的特定值。
      set_pwm(num, 0, 4096);
    } 
    else 
    {
      set_pwm(num, 0, value);
    }
  }
}

/*!
 *  @brief  根据输入的微秒设置 PCA9685 引脚之一的 PWM 输出，输出不精确
 * 
 *  @param  num PWM 输出引脚之一，从 0 到 15
 *  @param  microseconds 打开 PWM 输出的微秒数
 */
void PWMServoDriver::write_microseconds(uint8_t num, uint16_t microseconds) 
{
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM Via Microseconds on output");
  Serial.print(num);
  Serial.print(": ");
  Serial.print(Microseconds);
  Serial.println("->");
#endif

  double pulse = microseconds;
  double pulse_length = 1000000; // 1,000,000 us per second
  
  // Read prescale
  uint16_t prescale = read_prescale();

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(prescale);
  Serial.println(" PCA9685 chip prescale");
#endif

  // Calculate the pulse for PWM based on Equation 1 from the datasheet section
  // 7.3.5
  prescale += 1;
  pulse_length *= prescale;
  pulse_length /= this->oscillator_frequency_;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulselength);
  Serial.println(" us per bit");
#endif

  pulse /= pulse_length;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print(pulse);
  Serial.println(" pulse for PWM");
#endif

  set_pwm(num, 0, pulse);
}

/*!
 *  @brief  用于获取内部跟踪振荡器的频率
 * calculations
 *  @returns 
 * 
 */
uint32_t PWMServoDriver::get_oscillator_frequency(void) 
{
  return this->oscillator_frequency_;
}

/*!
 *  @brief 用于设置内部跟踪振荡器的频率
 * calculations
 *  @param frequency 用于频率计算
 */
void PWMServoDriver::set_oscillator_frequency(uint32_t freq)
{
  this->oscillator_frequency_ = freq;
}

/******************* Low level I2C interface 
uint8_t PWMServoDriver::read_byte(uint8_t addr) 
{
  this->i2c_->beginTransmission(this->i2c_addr);
  this->i2c_->write(addr);
  this->i2c_->endTransmission();

  this->i2c_->requestFrom((uint8_t)this->i2c_addr, (uint8_t)1);
  return this->i2c_->read();
}

void PWMServoDriver::write_byte(uint8_t addr, uint8_t d) 
{
  this->i2c_->beginTransmission(this->i2c_addr);
  this->i2c_->write(addr);
  this->i2c_->write(d);
  this->i2c_->endTransmission();
}
*/