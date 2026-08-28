/*
 * RCWController Test Program
 * for ESP-WROOM-32
 * 2017/4/3 by robo8080
 * https://github.com/robo8080/ESP32_RCWController_Test
 */
 
#include <WiFi.h>
#include <WiFiUdp.h>
#include "RCWController.h"
#include <Streaming.h> //yu
//const char ssid[] = "myDynabook"; //"ESP32-RCWC";  //  your network SSID (name)
//const char pass[] = "toshiba100"; //esp32pass";  // your network password

WiFiUDP udp;
unsigned int localPort = 10000;
const int PACKET_SIZE = 256;
char packetBuffer[PACKET_SIZE];

void RCW_setup() {
  //Serial.begin(115200);

  //WiFi.softAP(ssid, pass);
  IPAddress myIP = WiFi.softAPIP();

  Serial.print("AP IP address: ");
  Serial.println(myIP);

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(localPort);

}
//+yu
extern float steerInput, speedInput;
extern int standupControl, falldownControl, traceon;
extern int ballhit, logena, rcwSpeedOn;
extern void odometry_init(int);
//+yu
void RCW_loop() {
  int rlen;

  rlen = udp.parsePacket();
  if(rlen >= 10) {
    udp.read(packetBuffer, (rlen > PACKET_SIZE) ? PACKET_SIZE : rlen);
    memcpy( &controller.data[0], packetBuffer, sizeof(controller)); 
    /* yu
    Serial.printf("DATA:%02X %02X %d %d %d %d %d %d %d %02X\n\r",controller.data[0],controller.data[1],controller.data[2],controller.data[3],controller.data[4],
                                                               controller.data[5],controller.data[6],controller.data[7],controller.data[8],controller.data[9]);
    if (controller.status.A) { Serial.println("Button A : ON"); }
    else                     { Serial.println("Button A : OFF"); }
    Serial.printf("Left Analog LR : %d\n\r",controller.status.LeftAnalogLR);
    */ //yu

    speedInput = ((float) controller.status.LeftAnalogUD * 200 / 255 - 100); // Normalise between -100 and 100
    //+yu
    if (speedInput >10 || speedInput < -10) rcwSpeedOn = 1;
    else {
      rcwSpeedOn = 0;
      odometry_init(0);
    }
    //+yu
    steerInput = ((float) controller.status.RightAnalogLR * 200 / 255 - 100); 
    //steerInput = abs(steerInput)*steerInput/100; //+ yu
    steerInput /= 2;
    //steerInput = ((float) controller.status.LeftAnalogLR * 200 / 255 - 100);
    if (abs(steerInput) < 1.0) steerInput = 0; //+ yu
    // Serial << speedInput << "\t" << steerInput << endl;

    if (controller.status.R1 == 1 && standupControl == 0){
      standupControl =1 ;
      falldownControl = 0;
    } 
    else if (controller.status.R2 == 1){
      falldownControl = 1;
      standupControl = 0;
    }
    else if (controller.status.L1 == 1){
      ballhit = 1;
    }
    else if (controller.status.L2 == 1){
      if (logena == 0) logena = 1;
      else logena = 0;
    }
  }
  else {
    //speedInput /= 2 ;
    //steerInput /= 2 ;
  }
  
}