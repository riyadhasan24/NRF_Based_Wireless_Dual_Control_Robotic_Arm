/* The source Code from : https://github.com/riyadhasan24
 * By Md. Riyad Hasan
 */
 
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

const uint8_t Pot_Base_Pin     = 36;  // GPIO36 (ADC1)
const uint8_t Pot_Shoulder_Pin = 39;  // GPIO39 (ADC1)
const uint8_t Pot_Elbow_Pin    = 34;  // GPIO34 (ADC1)
const uint8_t Pot_Gripper_Pin  = 35;  // GPIO35 (ADC1)

const uint8_t Nrf_CE_Pin  = 33;
const uint8_t Nrf_CSN_Pin = 32;

// LEDs (Mode indication)
const uint8_t Wifi_Status_Led_Pin = 14;
const uint8_t Mode_1_Led_Pin      = 25;
const uint8_t Mode_2_Led_Pin      = 26;

uint8_t Servo1_Min_Position = 50;   // Base
uint8_t Servo1_Max_Position = 160;

uint8_t Servo2_Min_Position = 20;   // Shoulder
uint8_t Servo2_Max_Position = 170;

uint8_t Servo3_Min_Position = 30;   // Elbow
uint8_t Servo3_Max_Position = 160;

uint8_t Servo4_Min_Position = 90;   // Gripper
uint8_t Servo4_Max_Position = 130;

int Pot1_Min_Value = 0;     int Pot1_Max_Value = 1023;
int Pot2_Min_Value = 0;     int Pot2_Max_Value = 1023;
int Pot3_Min_Value = 0;     int Pot3_Max_Value = 1023;
int Pot4_Min_Value = 0;     int Pot4_Max_Value = 1023;

RF24 Radio(Nrf_CE_Pin, Nrf_CSN_Pin);
const byte Pipe_Address[6] = "ARM01";
const uint8_t Radio_Channel = 108;

struct ServoPacket
{
  uint8_t Servo1_Position;
  uint8_t Servo2_Position;
  uint8_t Servo3_Position;
  uint8_t Servo4_Position;
};

ServoPacket Tx_Packet;
ServoPacket Last_Tx_Packet;

const uint8_t Samples_For_Average = 8;   // 5..10 good
const uint8_t Deadband_Degrees    = 2;   // 1..3 good

uint32_t Last_Tx_Millis = 0;
uint16_t Tx_Interval_Ms = 20;            // 50Hz

int Read_Pot_Average(uint8_t Pin)
{
  long Sum = 0;

  for (uint8_t i = 0; i < Samples_For_Average; i++)
  {
    Sum += analogRead(Pin);
    delayMicroseconds(200);
  }

  int Average = (int)(Sum / Samples_For_Average);
  return Average;
}

uint8_t Map_Pot_To_Angle(int PotValue, int PotMin, int PotMax, uint8_t ServoMin, uint8_t ServoMax)
{
  // Safety clamp
  if (PotValue < PotMin) PotValue = PotMin;
  if (PotValue > PotMax) PotValue = PotMax;

  long Angle = map(PotValue, PotMin, PotMax, ServoMin, ServoMax);

  if (Angle < 0)   Angle = 0;
  if (Angle > 180) Angle = 180;

  return (uint8_t)Angle;
}

uint8_t Apply_Deadband(uint8_t NewValue, uint8_t OldValue)
{
  int Diff = abs((int)NewValue - (int)OldValue);

  if (Diff <= Deadband_Degrees)
  {
    return OldValue;
  }

  return NewValue;
}

void Setup_Mode_1_Leds()
{
  pinMode(Wifi_Status_Led_Pin, OUTPUT);
  pinMode(Mode_1_Led_Pin, OUTPUT);
  pinMode(Mode_2_Led_Pin, OUTPUT);

  digitalWrite(Mode_1_Led_Pin, HIGH);
  digitalWrite(Mode_2_Led_Pin, LOW);
  digitalWrite(Wifi_Status_Led_Pin, LOW);   // WiFi OFF in Mode-1
}

void Setup_Adc()
{
  // 10-bit: 0..1023
  analogReadResolution(10);

  // Good full-range stability for 3.3V pots
  analogSetPinAttenuation(Pot_Base_Pin,     ADC_11db);
  analogSetPinAttenuation(Pot_Shoulder_Pin, ADC_11db);
  analogSetPinAttenuation(Pot_Elbow_Pin,    ADC_11db);
  analogSetPinAttenuation(Pot_Gripper_Pin,  ADC_11db);
}

void Setup_Nrf()
{
  SPI.begin(18, 19, 23, Nrf_CSN_Pin);

  Radio.begin();
  Radio.setChannel(Radio_Channel);
  Radio.setPALevel(RF24_PA_LOW);
  Radio.setDataRate(RF24_250KBPS);
  Radio.setAutoAck(true);

  Radio.openWritingPipe(Pipe_Address);
  Radio.stopListening();
}

void setup()
{
  Setup_Mode_1_Leds();
  Setup_Adc();
  Setup_Nrf();

  // Safe startup angle (clamped into your ranges)
  Tx_Packet.Servo1_Position = 90;
  Tx_Packet.Servo2_Position = 90;
  Tx_Packet.Servo3_Position = 90;
  Tx_Packet.Servo4_Position = 90;

  // Apply range clamp
  Tx_Packet.Servo1_Position = constrain(Tx_Packet.Servo1_Position, Servo1_Min_Position, Servo1_Max_Position);
  Tx_Packet.Servo2_Position = constrain(Tx_Packet.Servo2_Position, Servo2_Min_Position, Servo2_Max_Position);
  Tx_Packet.Servo3_Position = constrain(Tx_Packet.Servo3_Position, Servo3_Min_Position, Servo3_Max_Position);
  Tx_Packet.Servo4_Position = constrain(Tx_Packet.Servo4_Position, Servo4_Min_Position, Servo4_Max_Position);

  Last_Tx_Packet = Tx_Packet;

  // Send once immediately
  Radio.write(&Tx_Packet, sizeof(Tx_Packet));
}

void loop()
{
  // 1) Read all potentiometers (average for stability)
  int Pot1_Value = Read_Pot_Average(Pot_Base_Pin);
  int Pot2_Value = Read_Pot_Average(Pot_Shoulder_Pin);
  int Pot3_Value = Read_Pot_Average(Pot_Elbow_Pin);
  int Pot4_Value = Read_Pot_Average(Pot_Gripper_Pin);

  // 2) Convert pot values to angles using your per-servo ranges
  uint8_t Servo1_Angle = Map_Pot_To_Angle(Pot1_Value, Pot1_Min_Value, Pot1_Max_Value, Servo1_Min_Position, Servo1_Max_Position);
  uint8_t Servo2_Angle = Map_Pot_To_Angle(Pot2_Value, Pot2_Min_Value, Pot2_Max_Value, Servo2_Min_Position, Servo2_Max_Position);
  uint8_t Servo3_Angle = Map_Pot_To_Angle(Pot3_Value, Pot3_Min_Value, Pot3_Max_Value, Servo3_Min_Position, Servo3_Max_Position);
  uint8_t Servo4_Angle = Map_Pot_To_Angle(Pot4_Value, Pot4_Min_Value, Pot4_Max_Value, Servo4_Min_Position, Servo4_Max_Position);

  // 3) Deadband (stop small jitter)
  Tx_Packet.Servo1_Position = Apply_Deadband(Servo1_Angle, Last_Tx_Packet.Servo1_Position);
  Tx_Packet.Servo2_Position = Apply_Deadband(Servo2_Angle, Last_Tx_Packet.Servo2_Position);
  Tx_Packet.Servo3_Position = Apply_Deadband(Servo3_Angle, Last_Tx_Packet.Servo3_Position);
  Tx_Packet.Servo4_Position = Apply_Deadband(Servo4_Angle, Last_Tx_Packet.Servo4_Position);

  // 4) Send packet at fixed rate (stable servo hold)
  uint32_t Now = millis();
  if (Now - Last_Tx_Millis >= Tx_Interval_Ms)
  {
    Last_Tx_Millis = Now;
    Radio.write(&Tx_Packet, sizeof(Tx_Packet));
    Last_Tx_Packet = Tx_Packet;
  }
}
