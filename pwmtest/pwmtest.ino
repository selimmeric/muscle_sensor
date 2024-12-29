/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  PWM test - this will drive 16 PWMs in a 'wave'

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "tinyCommand.hpp"
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
static tinyCommand cmd(Serial);
void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
//m0 = 50-540
// m0 = 2185 - 2675
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!

  Wire.setClock(400000);

    cmd.begin();

  cmd.setCmd("spos", set_pos);
}
uint32_t cmdStr2Num(char *str, uint8_t base) {
  return strtol(str, NULL, base);
}
void loop() {
    cmd.scan();
}
int16_t set_pos(int argc, char **argv) {
    int stat = 1;
    long stats =0;
    if (argc > 2) {
          stats = cmdStr2Num(argv[2], 10);
          // Drive each PWM in a 'wave'
          stat = atoi(argv[1]);
          pwm.setPWM(stat, 0, stats );
          delay(20);
          Serial.print(stats);
          Serial.print(" - ");
          Serial.println(stat);

    }else
    if (argc > 1) {
          stats = cmdStr2Num(argv[1], 10);
          // Drive each PWM in a 'wave'
          stat = atoi(argv[1]);
          pwm.setPWM(0, 0, stats );
          delay(20);
          Serial.print(stats);
          Serial.print(" - ");
          Serial.println(stat);

    }
}
