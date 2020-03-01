#ifndef CONTROLLER_H
#define CONTROLLER_H

#define PI 3.1415926535897932384626433832795
#define TAU (PI * 2)

#define CMD_SET_MODE 0x10
#define CMD_SET_POS 0x11
#define CMD_RESET_POS 0x12
#define CMD_ROTATE 0x13
#define CMD_RETURN_STATUS 0x14
#define CMD_STOP_RELEASE 0x15
#define CMD_STOP_EMERGENCY 0x16
#define CMD_FLIP_MOTOR 0x17

#define MODE_STOP 0x0F
#define MODE_SERVO 0x10
#define MODE_BACKDRIVE 0x11
#define MODE_ROTATE 0x12

#define PIN_MTR_PWM 23 // ENABLE / PWM
#define PIN_MTR_IN1 27 // IN1 / DRIVE 1
#define PIN_MTR_IN2 14 // IN2 / DRIVE 2

#define PIN_END_CS  33
#define PIN_END_CLK 25
#define PIN_END_DO  26

#define PIN_ENO_CS  32
#define PIN_ENO_CLK 22
#define PIN_ENO_DO  34

#define EEPROM_SIZE 64

#define EE_SET_1 0x51
#define EE_SET_2 0x22

#define EEADDR_EE_SET_1 0x00
#define EEADDR_EE_SET_2 0x01
#define EEADDR_ENCO_OFFSET_1 0x02
#define EEADDR_ENCO_OFFSET_2 0x03
#define EEADDR_ENCD_OFFSET_1 0x04
#define EEADDR_ENCD_OFFSET_2 0x05
#define EEADDR_MTR_FLIP 0x06
#define EEADDR_SEA_SPRING_RATE_1 0x07
#define EEADDR_SEA_SPRING_RATE_2 0x08

#include "EEPROM.h"

#include "ControllerState.h"
#include "Encoder.h"
#include "LED.h"
#include "Motor.h"
#include "MPU9250.h"
#include "NetworkPacket.h"
#include "PID.h"
#include "Timer.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68

int status;

class Controller {
  private:
    int mode = MODE_ROTATE;
    int modePrev = MODE_ROTATE;

    double pidPos = 0;
    double pidOut = 0;
    double pidGoal = 0;
    double pidThreshold = 0.003;
    double pidMaxSpeed = 100;

    float SEA_SPRING_RATE = 100.0; // Newton-Meters per Radian
    
    ControllerState state;
    
    Encoder encoderDrive = Encoder(PIN_END_CS, PIN_END_CLK, PIN_END_DO);
    Encoder encoderOutput = Encoder(PIN_ENO_CS, PIN_ENO_CLK, PIN_ENO_DO);
    LED led;
    Motor motor = Motor(PIN_MTR_PWM, PIN_MTR_IN1, PIN_MTR_IN2);
    PID pid;
    
    Timer imuTimer = Timer(5); // hz
    MPU9250 imu = MPU9250(Wire,0x68);

    void computeState () {
      float positionDrive = encoderDrive.getAngleRadians();
      float positionOutput = encoderOutput.getAngleRadians();
      float positionDiff = atan2(sin(positionOutput-positionDrive), cos(positionOutput-positionDrive));
      
      state.position = positionDrive;
      state.effort = motor.getEffort();
      state.velocity = 0.0;
      state.torque = positionDiff * SEA_SPRING_RATE;
      
      if (imuTimer.ready()) {
        step_imu();
      }
      
      Serial.println(state.toString());
    }

    double formatAngle (double x) {
      if (x < -TAU) {
        x += TAU;
        return formatAngle(x);
      } else if (x > TAU) {
        x -= TAU;
        return formatAngle(x);
      } else {
        return x;
      }
    }

    void setUpConfig () {
      if (!EEPROM.begin(EEPROM_SIZE))
      {
        Serial.println("Failed to initialise EEPROM");
        while (1) { led.blink(255, 0, 0); delay(50); };
      }
  
      uint8_t ee_set_1 = EEPROM.read(EEADDR_EE_SET_1);
      uint8_t ee_set_2 = EEPROM.read(EEADDR_EE_SET_2);

      // ensures that eeprom has been setup
      if (ee_set_1 == EE_SET_1 && ee_set_2 == EE_SET_2) {
        uint8_t encO_offset_1 = EEPROM.read(EEADDR_ENCO_OFFSET_1);
        uint8_t encO_offset_2 = EEPROM.read(EEADDR_ENCO_OFFSET_2);
        uint8_t encD_offset_1 = EEPROM.read(EEADDR_ENCD_OFFSET_1);
        uint8_t encD_offset_2 = EEPROM.read(EEADDR_ENCD_OFFSET_2);
        uint8_t mtr_flip = EEPROM.read(EEADDR_MTR_FLIP);
        uint8_t sea_spring_rate_1 = EEPROM.read(EEADDR_SEA_SPRING_RATE_1);
        uint8_t sea_spring_rate_2 = EEPROM.read(EEADDR_SEA_SPRING_RATE_2);

        uint16_t encO_offset = (encO_offset_1 + encO_offset_2 * 256);
        uint16_t encD_offset = (encD_offset_1 + encD_offset_2 * 256);
        SEA_SPRING_RATE = (sea_spring_rate_1 * 1.0 + sea_spring_rate_2 * 256.0) / 100.0;
        
        encoderOutput.setOffset(encO_offset);
        encoderDrive.setOffset(encD_offset);
  
        if (mtr_flip != 0) {
          motor.flipDrivePins();
        }
      } else {
        EEPROM.write(EEADDR_EE_SET_1, EE_SET_1);
        EEPROM.write(EEADDR_EE_SET_2, EE_SET_2);
        EEPROM.write(EEADDR_ENCO_OFFSET_1, 0);
        EEPROM.write(EEADDR_ENCO_OFFSET_2, 0);
        EEPROM.write(EEADDR_ENCD_OFFSET_1, 0);
        EEPROM.write(EEADDR_ENCD_OFFSET_2, 0);
        EEPROM.write(EEADDR_MTR_FLIP, 0);
        EEPROM.write(EEADDR_SEA_SPRING_RATE_1, 16);
        EEPROM.write(EEADDR_SEA_SPRING_RATE_2, 39);
        EEPROM.commit();
        setUpConfig();
      }
    }

    void setUpImu() {
      int imuStatus = imu.begin(19, 18);
      if (imuStatus < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) { led.blink(255, 165, 0); delay(50); }
      }
    }
  
  public:

    Controller () { }

    void setUp () {
      led.setUp();
      motor.setUp();
      encoderDrive.setUp();
      encoderOutput.setUp();
      led.white();
      setUpConfig();
      setUpImu();
      step_imu();
    }

    ControllerState* getState () {
      return &state;
    }

    /// ----------------------
    /// --- STEP FUNCTIONS ---
    /// ----------------------

    void step () {
      encoderDrive.step();
      encoderOutput.step();
      computeState();
      
      if (mode == MODE_STOP) {
        step_stop();
      } else if (mode == MODE_ROTATE) {
        step_rotate();
      } else if (mode == MODE_BACKDRIVE) {
        step_backdrive();
      } else if (mode == MODE_SERVO) {
        step_servo();
      } else {
        step_stop();
      }
    }

    void step_imu () {
      imu.readSensor();
      state.accel[0] = imu.getAccelX_mss();
      state.accel[1] = imu.getAccelY_mss();
      state.accel[2] = imu.getAccelZ_mss();
      state.gyro[0] = imu.getGyroX_rads();
      state.gyro[1] = imu.getGyroY_rads();
      state.gyro[2] = imu.getGyroZ_rads();
      state.mag[0] = imu.getMagX_uT();
      state.mag[1] = imu.getMagY_uT();
      state.mag[2] = imu.getMagZ_uT();
      state.temp = imu.getTemperature_C();
    }

    void step_rotate () {
      led.pulse(LED_CYAN);
      motor.executePreparedCommand();
    }

    void step_backdrive () {
      led.pulse(LED_YELLOW);
      int m = state.torque * 5;
      if (m < -100) {
        m = -100;
      } else if (m > 100) {
        m = 100;
      }
      motor.step(m);
    }

    void step_servo () {
      led.pulse(LED_MAGENTA);
      pidPos = state.position;
    
      // ignore if within threshold of goal -- good 'nuff
      if (abs(pidPos - pidGoal) >= pidThreshold) {
        double d = formatAngle(pidPos) - formatAngle(pidGoal);
        if (d > PI) {
          pidGoal = formatAngle(pidGoal);
          pidGoal += TAU;
        } else if (d < -PI) {
          pidPos = formatAngle(pidPos);
          pidPos += TAU;
        } else {
          pidGoal = formatAngle(pidGoal);
          pidPos = formatAngle(pidPos);
        }
    
        pid.Compute();
        int speed = pidOut * 100.0;
        if (speed > pidMaxSpeed) {
          speed = pidMaxSpeed;
        } else if (speed < -pidMaxSpeed) {
          speed = -pidMaxSpeed;
        }
        motor.step(speed);
      } else {
        motor.stop();
      }
    }

    void step_stop () {
      led.pulse(LED_RED);
      motor.stop();
    }

    /// ---------------------
    /// --- CMD FUNCTIONS ---
    /// ---------------------
  
    void parseCmd (NetworkPacket packet) {
      if (packet.command == CMD_SET_MODE) {
        cmd_setMode(packet);
      } else if (packet.command == CMD_SET_POS) {
        cmd_setPosition(packet);
      } else if (packet.command == CMD_RESET_POS) {
        cmd_resetPosition();
      } else if (packet.command == CMD_FLIP_MOTOR) {
        cmd_flipMotorPins();
      } else if (packet.command == CMD_ROTATE) {
        cmd_rotate(packet);
      } else if (packet.command == CMD_STOP_RELEASE) {
        cmd_release();
      } else if (packet.command == CMD_STOP_EMERGENCY) {
        cmd_stop();
      }
    }

    void cmd_setMode (NetworkPacket packet) {
      mode = packet.parameters[0];
      if (mode == MODE_SERVO) {
        pidMaxSpeed = 100;
        pidGoal = (double)state.position;
      }
    }

    void cmd_setPosition (NetworkPacket packet) {
      int param = packet.parameters[0] + packet.parameters[1] * 256;
      pidMaxSpeed = floor(100.0 * packet.parameters[2] / 255.0);
      double pos = param / 65535.0 * TAU;
      pidGoal = formatAngle(pos);
    }

    void cmd_resetPosition () {
      encoderOutput.setEqualTo(0.0);
      encoderDrive.setEqualTo(0.0);
      
      pidGoal = 0;
      pidOut = 0;
      pidPos = 0;
      pid.clear();

      uint16_t encO_offset = encoderOutput.getOffset();
      uint16_t encD_offset = encoderDrive.getOffset();

      uint8_t encO_offset_1 = encO_offset % 256;
      uint8_t encO_offset_2 = floor(encO_offset / 256.0);
      uint8_t encD_offset_1 = encD_offset % 256;
      uint8_t encD_offset_2 = floor(encD_offset / 256.0);

      EEPROM.write(EEADDR_ENCO_OFFSET_1, encO_offset_1);
      EEPROM.write(EEADDR_ENCO_OFFSET_2, encO_offset_2);
      EEPROM.write(EEADDR_ENCD_OFFSET_1, encD_offset_1);
      EEPROM.write(EEADDR_ENCD_OFFSET_2, encD_offset_2);
      EEPROM.commit();
    }

    void cmd_flipMotorPins () {
      motor.flipDrivePins();
      
      EEPROM.write(EEADDR_MTR_FLIP, motor.flipDrivePinsStatus());
      EEPROM.commit();
    }

    void cmd_rotate (NetworkPacket packet) {
      int offsetBinary = 128;
      int motorStep = packet.parameters[0] - offsetBinary;
      int motorDuration = packet.parameters[1] + packet.parameters[2] * 256;
      
      if (motorDuration > 1000) {
        motorDuration = 1000;
      }
      
      motor.prepareCommand(motorStep, motorDuration);
    }

    void cmd_release () {
      mode = modePrev;
    }

    void cmd_stop () {
      if (mode != MODE_STOP) {
        modePrev = mode;
      } else {
        modePrev = MODE_ROTATE;
      }
      mode = MODE_STOP;
    }
};

#endif
