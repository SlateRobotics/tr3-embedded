#ifndef NETWORKING_H
#define NETWORKING_H

#include "Controller.h"
#include "ControllerState.h"
#include "NetworkPacket.h"
#include "WiFi.h"

class Networking {
  private:
    WiFiClient client;
    char req[512];
    char res[512];
    uint8_t packet[16];
    NetworkPacket networkPacket;

    void connectWifi() {
      WiFi.begin(ssid, pass);
    
      while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.println("Connecting to WiFi..");
      }
      
      Serial.println("WiFi connected");
    }

    void connectServer() {
      if (!client.connect(host, port)) {
        Serial.println("Server connection failed");
        delay(500);
        connectServer();
      }
      
      Serial.println("Server connected");
    }
  
  public:
    Controller* controller;
    
    char* host = "192.168.4.1";
    uint16_t port = 1738;
    char* ssid;
    char* pass;

    void connect() {
      connectWifi();
      connectServer();
    }

    void step (char* actuatorId, ControllerState* state) {
      request(actuatorId, state);
      response();
    }

    void request (char* actuatorId, ControllerState* state) {
      static char stateStr[8];
      dtostrf(state->position, 6, 4, stateStr);
      
      sprintf(req, "%s:%s;\r\n", actuatorId, stateStr);
      client.print(req);
      client.readStringUntil('\r').toCharArray(res, 512);
    }

    void response () {
      int bufIdx = 0;
      char buf[16];
      int packetIdx = 0;
    
      // no command
      if (strstr(res, "nc;") || strlen(res) == 0) {
        return;
      }
    
      for (int i = 0; i < strlen(res); i++) {
        char c = res[i];
        if (c == ':') {
          bufIdx = 0;
        } else if (c == ';' && bufIdx > 0) {
          networkPacket = NetworkPacket(packet);
          controller->parseCmd(networkPacket);
          packetIdx = 0;
          bufIdx = 0;
        } else if (c != ',') {
          buf[bufIdx++] = c;
        } else {
          buf[bufIdx] = '\0';
          bufIdx = 0;
          packet[packetIdx++] = atoi(buf);
        }
      }
    }
};

#endif
