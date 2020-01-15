#ifndef NETWORKPACKET_H
#define NETWORKPACKET_H

class NetworkPacket {
  private:
  
  public:
    uint8_t msgId = 0;
    uint8_t address = 0;
    uint8_t length = 0;
    uint8_t command = 0;
    uint8_t parameters[4];

    NetworkPacket () { }
    
    NetworkPacket (uint8_t packet[16]) {
      msgId = packet[0];
      address = packet[1];
      length = packet[2];
      command = packet[3];

      if (length - 4 > 4) {
        length = 4;
      }
      
      for (int i = 4; i < length; i++) {
        parameters[i - 4] = packet[i];
      }
    }

    String toString() {
      String result = "";
      result += msgId;
      result += ",";
      result += address;
      result += ",";
      result += length;
      result += ",";
      result += command;
      result += ",";

      for (int i = 4; i < length; i++) {
        result += parameters[i - 4];
        result += ",";
      }
      
      result += ";";
      return result;
    }
};

#endif
