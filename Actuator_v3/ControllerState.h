#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

class ControllerState {
  private:

  public:
    float position = 0.0; // Radians
    float effort = 0.0; // -100 -> 100
    float velocity = 0.0; // Radians per second
    float torque = 0.0; // Newton-Meters
    float accel[3] = {0, 0, 0}; // x, y, z (meter/sec^2)
    float gyro[3] = {0, 0, 0}; // x, y, z (rad/sec)
    float mag[3] = {0, 0, 0}; // x, y, z (uT, microtesla)
    float temp = 0.0; // celsius
  
    String toString() {
      String result = "pos:";
      result += position;
      result += ",eff:";
      result += effort;
      result += ",vel:";
      result += velocity;
      result += ",trq:";
      result += torque;
      result += ",accel:{";
      result += accel[0];
      result += ",";
      result += accel[1];
      result += ",";
      result += accel[2];
      result += "},gyro:{";
      result += gyro[0];
      result += ",";
      result += gyro[1];
      result += ",";
      result += gyro[2];
      result += "},mag:{";
      result += mag[0];
      result += ",";
      result += mag[1];
      result += ",";
      result += mag[2];
      result += "},temp:";
      result += temp;
      result += ";";
      return result;
    }
};

#endif
