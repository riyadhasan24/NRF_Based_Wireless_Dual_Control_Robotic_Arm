/* The source Code from : https://github.com/riyadhasan24
 * By Md. Riyad Hasan
 */
 
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>

const uint8_t Pot_Base_Pin     = 36;
const uint8_t Pot_Shoulder_Pin = 39;
const uint8_t Pot_Elbow_Pin    = 34;
const uint8_t Pot_Gripper_Pin  = 35;

const uint8_t Nrf_CE_Pin  = 33;
const uint8_t Nrf_CSN_Pin = 32;

const uint8_t Wifi_Status_Led_Pin = 14;
const uint8_t Mode_1_Led_Pin      = 25;
const uint8_t Mode_2_Led_Pin      = 26;

const uint8_t Mode_Change_Button_Pin = 4;   // External pull-up already used (NO internal pullup)
const uint8_t Buzzer_Pin             = 2;  // Recommended (safe pin)

const uint8_t Mode_1 = 1;
const uint8_t Mode_2 = 2;

uint8_t Current_Mode = Mode_1;   // Default on power-up

uint8_t Servo1_Min_Position = 50;    // Base
uint8_t Servo1_Max_Position = 160;

uint8_t Servo2_Min_Position = 20;    // Shoulder
uint8_t Servo2_Max_Position = 170;

uint8_t Servo3_Min_Position = 30;    // Elbow
uint8_t Servo3_Max_Position = 160;

uint8_t Servo4_Min_Position = 90;    // Gripper
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
uint16_t Tx_Interval_Ms = 20;    // 50Hz update rate

uint32_t Button_Press_Start_Millis = 0;
bool Button_Was_Pressed = false;

uint16_t Long_Press_Time_Ms = 1200;   // press & hold time to switch mode
bool Mode_Changed_This_Press = false;

const char* Ap_Ssid     = "Wireless Robotic Arm";
const char* Ap_Password = "12345678";

WebServer Server(80);
bool Wifi_Is_Started = false;

/* WiFi LED blink */
uint32_t Last_Wifi_Blink_Millis = 0;
bool Wifi_Led_State = false;
uint16_t Wifi_Blink_Interval_Ms = 300;

uint8_t Clamp_To_Range(uint8_t Value, uint8_t MinVal, uint8_t MaxVal)
{
  if (Value < MinVal) return MinVal;
  if (Value > MaxVal) return MaxVal;
  return Value;
}

uint8_t Convert_Text_To_Angle(String Text)
{
  int Value = Text.toInt();

  if (Value < 0)   Value = 0;
  if (Value > 180) Value = 180;

  return (uint8_t)Value;
}

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

void Buzzer_Beep(uint8_t Times)
{
  for (uint8_t i = 0; i < Times; i++)
  {
    digitalWrite(Buzzer_Pin, HIGH);
    delay(120);
    digitalWrite(Buzzer_Pin, LOW);
    delay(120);
  }
}

void Update_Mode_Leds()
{
  if (Current_Mode == Mode_1)
  {
    digitalWrite(Mode_1_Led_Pin, HIGH);
    digitalWrite(Mode_2_Led_Pin, LOW);
    digitalWrite(Wifi_Status_Led_Pin, LOW);  // WiFi OFF in Mode-1
  }
  else
  {
    digitalWrite(Mode_1_Led_Pin, LOW);
    digitalWrite(Mode_2_Led_Pin, HIGH);
    // WiFi LED handled by Update_Wifi_Status_Led()
  }
}

void Update_Wifi_Status_Led()
{
  if (Current_Mode != Mode_2)
  {
    digitalWrite(Wifi_Status_Led_Pin, LOW);
    return;
  }

  int Station_Count = WiFi.softAPgetStationNum();

  if (Station_Count <= 0)
  {
    uint32_t Now = millis();
    if (Now - Last_Wifi_Blink_Millis >= Wifi_Blink_Interval_Ms)
    {
      Last_Wifi_Blink_Millis = Now;

      if (Wifi_Led_State == false)
      {
        Wifi_Led_State = true;
        digitalWrite(Wifi_Status_Led_Pin, HIGH);
      }
      else
      {
        Wifi_Led_State = false;
        digitalWrite(Wifi_Status_Led_Pin, LOW);
      }
    }
  }
  else
  {
    digitalWrite(Wifi_Status_Led_Pin, LOW);
  }
}

String Build_Html_Page()
{
  String Page = "";

  Page += "<!DOCTYPE html>";
  Page += "<html>";
  Page += "<head>";
  Page += "  <meta name='viewport' content='width=device-width, initial-scale=1'>";
  Page += "  <title>Wireless Robotic Arm</title>";

  Page += "  <style>";
  Page += "    body{margin:0;font-family:Arial;background:linear-gradient(135deg,#0ea5e9,#a855f7,#f97316);min-height:100vh;display:flex;justify-content:center;}";
  Page += "    .card{background:white;margin:16px;padding:18px;border-radius:18px;max-width:560px;width:95%;box-shadow:0 10px 30px rgba(0,0,0,0.25);}";

  Page += "    h2{text-align:center;margin:6px 0 14px 0;}";

  Page += "    .sliderBox{border-radius:14px;padding:12px;margin:12px 0;}";
  Page += "    .b1{background:linear-gradient(135deg,#fde047,#f97316);}";
  Page += "    .b2{background:linear-gradient(135deg,#86efac,#22c55e);}";
  Page += "    .b3{background:linear-gradient(135deg,#93c5fd,#3b82f6);}";
  Page += "    .b4{background:linear-gradient(135deg,#fbcfe8,#ec4899);}";

  Page += "    .valueLine{font-size:18px;font-weight:bold;text-align:right;background:rgba(255,255,255,0.6);padding:4px 10px;border-radius:12px;width:70px;margin-left:auto;}";
  Page += "    input[type=range]{width:100%;margin-top:10px;}";

  Page += "    .team{background:#f1f5f9;border-radius:14px;padding:12px;margin-top:18px;font-size:14px;}";
  Page += "    .teamTitle{font-weight:bold;margin-bottom:8px;text-align:center;}";
  Page += "    .name{text-align:center;padding:2px 0;}";
  Page += "  </style>";
  Page += "</head>";

  Page += "<body>";
  Page += "  <div class='card'>";

  // Title only
  Page += "    <h2>Wireless Robotic Arm</h2>";

  // Sliders
  Page += "    <div class='sliderBox b1'>";
  Page += "      <div class='valueLine' id='V1'>" + String(Tx_Packet.Servo1_Position) + "</div>";
  Page += "      <input id='S1' type='range' min='" + String(Servo1_Min_Position) + "' max='" + String(Servo1_Max_Position) + "' value='" + String(Tx_Packet.Servo1_Position) + "' oninput='Send()'>";
  Page += "    </div>";

  Page += "    <div class='sliderBox b2'>";
  Page += "      <div class='valueLine' id='V2'>" + String(Tx_Packet.Servo2_Position) + "</div>";
  Page += "      <input id='S2' type='range' min='" + String(Servo2_Min_Position) + "' max='" + String(Servo2_Max_Position) + "' value='" + String(Tx_Packet.Servo2_Position) + "' oninput='Send()'>";
  Page += "    </div>";

  Page += "    <div class='sliderBox b3'>";
  Page += "      <div class='valueLine' id='V3'>" + String(Tx_Packet.Servo3_Position) + "</div>";
  Page += "      <input id='S3' type='range' min='" + String(Servo3_Min_Position) + "' max='" + String(Servo3_Max_Position) + "' value='" + String(Tx_Packet.Servo3_Position) + "' oninput='Send()'>";
  Page += "    </div>";

  Page += "    <div class='sliderBox b4'>";
  Page += "      <div class='valueLine' id='V4'>" + String(Tx_Packet.Servo4_Position) + "</div>";
  Page += "      <input id='S4' type='range' min='" + String(Servo4_Min_Position) + "' max='" + String(Servo4_Max_Position) + "' value='" + String(Tx_Packet.Servo4_Position) + "' oninput='Send()'>";
  Page += "    </div>";

  /*
  // Team members UNDER sliders
  Page += "    <div class='team'>";
  Page += "      <div class='teamTitle'>Team Members</div>";
  Page += "      <div class='name'>ABID MOLLA</div>";
  Page += "      <div class='name'>MAHMUDUL HASAN</div>";
  Page += "      <div class='name'>MD. ABDULLAH ALL FARUK</div>";
  Page += "      <div class='name'>KHADIZA AKTER</div>";
  Page += "      <div class='name'>ROMAN HOSSAIN BIJOY</div>";
  Page += "      <div class='name'>MD. ROBIUL ISLAM</div>";
  Page += "      <div class='name'>MST. MUBASHIRA ZAMAN DUTHI</div>";
  Page += "      <div class='name'>MD. SAKIN HASAN</div>";
  Page += "      <div class='name'>AHSAN HABIB</div>";
  Page += "      <div class='name'>MD. RIYAD HASAN</div>";
  Page += "    </div>";
  */
  // JS
  Page += "    <script>";
  Page += "      function Send(){";
  Page += "        var S1=document.getElementById('S1').value;";
  Page += "        var S2=document.getElementById('S2').value;";
  Page += "        var S3=document.getElementById('S3').value;";
  Page += "        var S4=document.getElementById('S4').value;";
  Page += "        document.getElementById('V1').innerText=S1;";
  Page += "        document.getElementById('V2').innerText=S2;";
  Page += "        document.getElementById('V3').innerText=S3;";
  Page += "        document.getElementById('V4').innerText=S4;";
  Page += "        fetch('/set?b='+S1+'&s='+S2+'&e='+S3+'&g='+S4).catch(function(err){});";
  Page += "      }";
  Page += "    </script>";

  Page += "  </div>";
  Page += "</body>";
  Page += "</html>";

  return Page;
}

void Handle_Root()
{
  String Html = Build_Html_Page();
  Server.send(200, "text/html", Html);
}

void Handle_Set()
{
  // Base
  if (Server.hasArg("b") == true)
  {
    String Text = Server.arg("b");
    uint8_t Angle = Convert_Text_To_Angle(Text);
    Angle = Clamp_To_Range(Angle, Servo1_Min_Position, Servo1_Max_Position);
    Tx_Packet.Servo1_Position = Angle;
  }

  // Shoulder
  if (Server.hasArg("s") == true)
  {
    String Text = Server.arg("s");
    uint8_t Angle = Convert_Text_To_Angle(Text);
    Angle = Clamp_To_Range(Angle, Servo2_Min_Position, Servo2_Max_Position);
    Tx_Packet.Servo2_Position = Angle;
  }

  // Elbow
  if (Server.hasArg("e") == true)
  {
    String Text = Server.arg("e");
    uint8_t Angle = Convert_Text_To_Angle(Text);
    Angle = Clamp_To_Range(Angle, Servo3_Min_Position, Servo3_Max_Position);
    Tx_Packet.Servo3_Position = Angle;
  }

  // Gripper
  if (Server.hasArg("g") == true)
  {
    String Text = Server.arg("g");
    uint8_t Angle = Convert_Text_To_Angle(Text);
    Angle = Clamp_To_Range(Angle, Servo4_Min_Position, Servo4_Max_Position);
    Tx_Packet.Servo4_Position = Angle;
  }

  Server.send(200, "text/plain", "OK");
}

void Start_Wifi_Mode_2()
{
  if (Wifi_Is_Started == true)
  {
    return;
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(Ap_Ssid, Ap_Password);

  Server.on("/", Handle_Root);
  Server.on("/set", Handle_Set);
  Server.begin();

  Wifi_Is_Started = true;

  // reset wifi led blink state
  Wifi_Led_State = false;
  Last_Wifi_Blink_Millis = millis();
}

void Stop_Wifi_Mode_2()
{
  if (Wifi_Is_Started == false)
  {
    return;
  }

  Server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  Wifi_Is_Started = false;

  digitalWrite(Wifi_Status_Led_Pin, LOW);
}


void Update_Packet_From_Pots_Mode_1()
{
  int Pot1_Value = Read_Pot_Average(Pot_Base_Pin);
  int Pot2_Value = Read_Pot_Average(Pot_Shoulder_Pin);
  int Pot3_Value = Read_Pot_Average(Pot_Elbow_Pin);
  int Pot4_Value = Read_Pot_Average(Pot_Gripper_Pin);

  uint8_t Servo1_Angle = Map_Pot_To_Angle(Pot1_Value, Pot1_Min_Value, Pot1_Max_Value, Servo1_Min_Position, Servo1_Max_Position);
  uint8_t Servo2_Angle = Map_Pot_To_Angle(Pot2_Value, Pot2_Min_Value, Pot2_Max_Value, Servo2_Min_Position, Servo2_Max_Position);
  uint8_t Servo3_Angle = Map_Pot_To_Angle(Pot3_Value, Pot3_Min_Value, Pot3_Max_Value, Servo3_Min_Position, Servo3_Max_Position);
  uint8_t Servo4_Angle = Map_Pot_To_Angle(Pot4_Value, Pot4_Min_Value, Pot4_Max_Value, Servo4_Min_Position, Servo4_Max_Position);

  Tx_Packet.Servo1_Position = Apply_Deadband(Servo1_Angle, Last_Tx_Packet.Servo1_Position);
  Tx_Packet.Servo2_Position = Apply_Deadband(Servo2_Angle, Last_Tx_Packet.Servo2_Position);
  Tx_Packet.Servo3_Position = Apply_Deadband(Servo3_Angle, Last_Tx_Packet.Servo3_Position);
  Tx_Packet.Servo4_Position = Apply_Deadband(Servo4_Angle, Last_Tx_Packet.Servo4_Position);
}

void Handle_Mode_Button()
{
  int Button_State = digitalRead(Mode_Change_Button_Pin);

  // Button pressed
  if (Button_State == LOW)
  {
    if (Button_Was_Pressed == false)
    {
      Button_Was_Pressed = true;
      Button_Press_Start_Millis = millis();
      Mode_Changed_This_Press = false;
    }
    else
    {
      // still holding
      if (Mode_Changed_This_Press == false)
      {
        uint32_t Now = millis();
        uint32_t Held_Time = Now - Button_Press_Start_Millis;

        if (Held_Time >= Long_Press_Time_Ms)
        {
          // Change mode
          if (Current_Mode == Mode_1)
          {
            Current_Mode = Mode_2;
            Start_Wifi_Mode_2();
            Buzzer_Beep(2);
          }
          else
          {
            Current_Mode = Mode_1;
            Stop_Wifi_Mode_2();
            Buzzer_Beep(1);
          }

          Update_Mode_Leds();
          Mode_Changed_This_Press = true;
        }
      }
    }
  }
  else
  {
    // Button released
    Button_Was_Pressed = false;
    Mode_Changed_This_Press = false;
  }
}

void Setup_Leds_And_Buzzer()
{
  pinMode(Wifi_Status_Led_Pin, OUTPUT);
  pinMode(Mode_1_Led_Pin, OUTPUT);
  pinMode(Mode_2_Led_Pin, OUTPUT);

  pinMode(Buzzer_Pin, OUTPUT);
  digitalWrite(Buzzer_Pin, LOW);
}

void Setup_Button()
{
  pinMode(Mode_Change_Button_Pin, INPUT);  // NO internal pull-up
}

void Setup_Adc()
{
  analogReadResolution(10);

  // 3.3V pots stable range
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
  Setup_Leds_And_Buzzer();
  Setup_Button();
  Setup_Adc();
  Setup_Nrf();

  // Default mode is Mode-1
  Current_Mode = Mode_1;
  Stop_Wifi_Mode_2();   // ensure WiFi fully OFF

  // Safe startup angles (must be inside min/max)
  Tx_Packet.Servo1_Position = Clamp_To_Range(90, Servo1_Min_Position, Servo1_Max_Position);
  Tx_Packet.Servo2_Position = Clamp_To_Range(90, Servo2_Min_Position, Servo2_Max_Position);
  Tx_Packet.Servo3_Position = Clamp_To_Range(90, Servo3_Min_Position, Servo3_Max_Position);
  Tx_Packet.Servo4_Position = Clamp_To_Range(90, Servo4_Min_Position, Servo4_Max_Position);

  Last_Tx_Packet = Tx_Packet;

  Update_Mode_Leds();

  // Send once immediately
  Radio.write(&Tx_Packet, sizeof(Tx_Packet));

  // Optional: one beep to show system ready in Mode-1
  Buzzer_Beep(1);
}

void loop()
{
  // 1) Handle long-press mode switching
  Handle_Mode_Button();

  // 2) Mode-2 web server
  if (Current_Mode == Mode_2)
  {
    if (Wifi_Is_Started == true)
    {
      Server.handleClient();
    }
  }

  // 3) WiFi LED behavior (only meaningful in Mode-2)
  Update_Wifi_Status_Led();

  // 4) Update packet source depending on mode
  if (Current_Mode == Mode_1)
  {
    Update_Packet_From_Pots_Mode_1();
  }
  // In Mode-2, Tx_Packet is updated by /set requests from web sliders

  // 5) Continuous NRF sending for stable servo hold
  uint32_t Now = millis();
  if (Now - Last_Tx_Millis >= Tx_Interval_Ms)
  {
    Last_Tx_Millis = Now;

    Radio.write(&Tx_Packet, sizeof(Tx_Packet));
    Last_Tx_Packet = Tx_Packet;
  }
}
