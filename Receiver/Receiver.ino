/* The source Code from : https://github.com/riyadhasan24
 * By Md. Riyad Hasan
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ================= PINS (ATmega8A) =================
const uint8_t Servo_Base_Pin     = 5;
const uint8_t Servo_Shoulder_Pin = 6;
const uint8_t Servo_Elbow_Pin    = 9;
const uint8_t Servo_Gripper_Pin  = 10;

const uint8_t Nrf_CE_Pin  = 7;
const uint8_t Nrf_CSN_Pin = 8;

RF24 Radio(Nrf_CE_Pin, Nrf_CSN_Pin);
const byte Pipe_Address[6] = "ARM01";

struct ServoPacket
{
  uint8_t Servo1_Position;
  uint8_t Servo2_Position;
  uint8_t Servo3_Position;
  uint8_t Servo4_Position;
};

ServoPacket Rx_Packet;

Servo Servo_Base;
Servo Servo_Shoulder;
Servo Servo_Elbow;
Servo Servo_Gripper;

void setup()
{
  Servo_Base.attach(Servo_Base_Pin);
  Servo_Shoulder.attach(Servo_Shoulder_Pin);
  Servo_Elbow.attach(Servo_Elbow_Pin);
  Servo_Gripper.attach(Servo_Gripper_Pin);

  // Safe start
  Servo_Base.write(90);
  Servo_Shoulder.write(90);
  Servo_Elbow.write(90);
  Servo_Gripper.write(90);

  SPI.begin();

  Radio.begin();
  Radio.setChannel(108);
  Radio.setPALevel(RF24_PA_LOW);
  Radio.setDataRate(RF24_250KBPS);

  Radio.openReadingPipe(0, Pipe_Address);
  Radio.startListening();
}

void loop()
{
  if (Radio.available())
  {
    Radio.read(&Rx_Packet, sizeof(Rx_Packet));

    Servo_Base.write(Rx_Packet.Servo1_Position);
    Servo_Shoulder.write(Rx_Packet.Servo2_Position);
    Servo_Elbow.write(Rx_Packet.Servo3_Position);
    Servo_Gripper.write(Rx_Packet.Servo4_Position);
  }
}
