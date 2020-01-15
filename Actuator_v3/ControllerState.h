#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

class ControllerState {
  private:

  public:
    float position = 0.0; // Radians
    float effort = 0.0; // -100 -> 100
    float velocity = 0.0; // Radians per second
    float torque = 0.0; // Newton-Meters

    // accelerometer
    // gyroscope
  
    String toString() {
      String result = "";
      result += position;
      result += ",";
      result += effort;
      result += ",";
      result += velocity;
      result += ",";
      result += torque;
      result += ",";
      result += ";";
      return result;
    }
};

#endif
