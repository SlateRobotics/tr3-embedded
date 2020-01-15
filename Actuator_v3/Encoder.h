#ifndef EMS22A_H
#define EMS22A_H

class Encoder {
  private:
    int PIN_CLOCK;
    int PIN_DATA;
    int PIN_CS;

    uint16_t encoderResolution = 4096;
    uint8_t lapNumber = 0;
    uint8_t maxLap = 1;
    static const int prevPositionN = 3;
    uint16_t prevPosition[prevPositionN];
    uint16_t offset = 0;

    void handleLapChange() {
      if (maxLap <= 1) {
        return;
      }
    
      int posCur = prevPosition[0];
      int posPrv = prevPosition[1];
      int posTHi = encoderResolution - 75;
      int posTLo = 75;
    
      bool posCurHi = posCur >= posTHi;
      bool posCurLo = posCur <= posTLo;
      bool posPrvHi = posPrv >= posTHi;
      bool posPrvLo = posPrv <= posTLo;
    
      if (posCurLo && posPrvHi) {
        changeLap(1);
      } else if (posPrvLo && posCurHi) {
        changeLap(-1);
      }
    }
    
    void changeLap(int i) {
      if (i > 0) {
        if (lapNumber >= 2) {
          lapNumber = 0;
        } else {
          lapNumber++;
        }
      } else if (i < 0) {
        if (lapNumber <= 0) {
          lapNumber = 2;
        } else {
          lapNumber--;
        }
      }
    }
  
  public:
    Encoder(int pCs, int pClock, int pData) {
      PIN_CS = pCs;
      PIN_CLOCK = pClock;
      PIN_DATA = pData;
    }
    
    void setUp () {
      pinMode(PIN_CS, OUTPUT);
      pinMode(PIN_CLOCK, OUTPUT);
      pinMode(PIN_DATA, INPUT);
    
      digitalWrite(PIN_CLOCK, HIGH);
      digitalWrite(PIN_CS, HIGH);
      
      readPosition();
      readPosition();
    }
    
    void setLap(uint8_t i) {
      if (i < maxLap && i >= 0) {
        lapNumber = i;
      }
    }
    
    void setMaxLap(uint8_t i) {
      maxLap = i;
      if (lapNumber > i) {
        lapNumber = i;
      }
    }
    
    void setOffset(uint16_t pos) {
      if (pos > encoderResolution || pos < 0) {
        pos = 0;
      }
      offset = pos;
    }
    
    void setEqualTo(float posSet) {  
      int encRead = prevPosition[0];
      float posCur = (float)encRead / (encoderResolution * maxLap) * TAU;
      float posDif = fabs(posSet - posCur);
      
      float lap = floor(posDif / TAU * maxLap) + 1;
      posCur = (((encoderResolution * lap) + encRead) / (encoderResolution * maxLap)) * TAU;
      posDif = fabs(posSet - posCur);
      
      int offset = floor(posDif / TAU * encoderResolution * maxLap);
      
      int recCount = 0;
      while (offset > encoderResolution && recCount < 10) {
          recCount++;
          if (offset > encoderResolution) {
              lap -= 1;
              offset -= encoderResolution;
          }
      }
      
      if (lap >= maxLap) {
          lap -= maxLap;
      }
    
      setOffset(offset);
      setLap(lap);
      
      readPosition();
      readPosition();
    }
    
    int readPosition() {
      unsigned int dataOut = 0;
      digitalWrite(PIN_CS, LOW);
      delayMicroseconds(1);
    
      for(int x=0; x<12; x++){
        digitalWrite(PIN_CLOCK, LOW);
        delayMicroseconds(1);
        digitalWrite(PIN_CLOCK, HIGH);
        delayMicroseconds(1);
        dataOut = (dataOut << 1) | digitalRead(PIN_DATA);
      }
    
      digitalWrite(PIN_CS, HIGH);

      prevPosition[0] = dataOut;
      delayMicroseconds(1);
      
      return dataOut;
    }
    
    void step() {
      readPosition();
    }
    
    int getOffset() {
      return offset;
    }
    
    int getLap() {
      return lapNumber;
    }
    
    int getPosition() {
      if (maxLap <= 1) {
        return prevPosition[0];
      }
      
      return prevPosition[0] + (lapNumber * encoderResolution);
    }
      
    int getMaxResolution() {
      return encoderResolution * maxLap;
    }
    
    float getAngleRadians() {
      float encRead = prevPosition[0];
      float pos = (encoderResolution * lapNumber) + encRead - offset;
      float maxPos = encoderResolution * maxLap;
      float r = pos / maxPos * TAU;
    
      int recCount = 0;
      while ((r < 0 || r > TAU) && recCount < 10) {
        recCount++;
        if (r < 0) {
          r += TAU;
        } else if (r > TAU) {
          r -= TAU;
        }
      }
      
      return r;
    }
    
};

#endif
