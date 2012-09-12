#include <Ethernet.h>
#include <ClientUDP.h>
#include <PS2X_lib.h>

PS2X ps2x; // create PS2 Controller Class

byte vibrate = 0;

byte mac[] = { 0x00, 0x1D, 0x60, 0xAF, 0x03, 0x31 };
byte ip[]  = { 192, 168, 1, 33 };
byte gw[] = { 192, 168, 1, 1 };
byte subnet[] = { 255, 255, 255, 0 };

#define PORT 4444
byte ip_to[] = {192,168,1,22};
uint16_t port_to = PORT;
byte joystick[5];

ClientUDP udp(ip_to,port_to);

void setup() {
 ps2x.config_gamepad(7,5,4,6);   //setup GamePad(clock, command, attention, data) pins
 ps2x.enableRumble();            //enable rumble vibration motors
 ps2x.enablePressures();         //enable reading the pressure values from the buttons.
  
  Ethernet.begin(mac, ip, gw, subnet);  
  udp.open(PORT);
  Serial.begin(9600);
  Serial.println("Robot Operator Interface v0.1 Initialized");
}

void loop() {
  ps2x.read_gamepad(false, vibrate);        //read controller
  //Build the joystick byte array
  joystick[0] = 255 - ps2x.Analog(PSS_LY);
  joystick[1] = ps2x.Analog(PSS_LX);
  joystick[2] = 255 - ps2x.Analog(PSS_RY);
  joystick[3] = ps2x.Analog(PSS_RX);
  if (ps2x.Button(PSB_R2))
    joystick[4] = 1;
  else if (ps2x.Button(PSB_L2))
    joystick[4] = 2;
  else
    joystick[4] = 0;
  udp.write(joystick,sizeof(joystick));
  delay(10);
}
