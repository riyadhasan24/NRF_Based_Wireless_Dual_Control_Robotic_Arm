/* The source Code from : https://github.com/riyadhasan24
 * By Md. Riyad Hasan
 */
 
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <RF24.h>

const uint8_t Nrf_CE_Pin  = 33;
const uint8_t Nrf_CSN_Pin = 32;

const uint8_t Wifi_Status_Led_Pin = 14;
const uint8_t Mode_1_Led_Pin      = 25;
const uint8_t Mode_2_Led_Pin      = 26;

uint8_t Servo1_Min_Position = 50;
uint8_t Servo1_Max_Position = 160;

uint8_t Servo2_Min_Position = 20;
uint8_t Servo2_Max_Position = 170;

uint8_t Servo3_Min_Position = 30;
uint8_t Servo3_Max_Position = 160;

uint8_t Servo4_Min_Position = 90;
uint8_t Servo4_Max_Position = 130;

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

const char* Ap_Ssid     = "Wireless Robotic Arm";
const char* Ap_Password = "12345678";

WebServer Server(80);

uint32_t Last_Tx_Millis = 0;
uint16_t Tx_Interval_Ms = 20;

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

  /* ---------- TITLE ONLY AT TOP ---------- */
  Page += "    <h2>Wireless Robotic Arm</h2>";

  /* ---------- SLIDERS ---------- */
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

  /* ---------- TEAM MEMBERS (UNDER SLIDERS) ---------- 
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
  /* ---------- JAVASCRIPT ---------- */
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

void Setup_Leds_For_Mode2()
{
  pinMode(Wifi_Status_Led_Pin, OUTPUT);
  pinMode(Mode_1_Led_Pin, OUTPUT);
  pinMode(Mode_2_Led_Pin, OUTPUT);

  digitalWrite(Mode_1_Led_Pin, LOW);
  digitalWrite(Mode_2_Led_Pin, HIGH);
  digitalWrite(Wifi_Status_Led_Pin, LOW);
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

void Setup_Wifi_Ap()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(Ap_Ssid, Ap_Password);
}

void Setup_Web_Server()
{
  Server.on("/", Handle_Root);
  Server.on("/set", Handle_Set);
  Server.begin();
}

void Update_Wifi_Status_Led()
{
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

void setup()
{
  Setup_Leds_For_Mode2();

  // Safe start (must be within min/max)
  Tx_Packet.Servo1_Position = Clamp_To_Range(90, Servo1_Min_Position, Servo1_Max_Position);
  Tx_Packet.Servo2_Position = Clamp_To_Range(90, Servo2_Min_Position, Servo2_Max_Position);
  Tx_Packet.Servo3_Position = Clamp_To_Range(90, Servo3_Min_Position, Servo3_Max_Position);
  Tx_Packet.Servo4_Position = Clamp_To_Range(90, Servo4_Min_Position, Servo4_Max_Position);

  Setup_Nrf();
  Setup_Wifi_Ap();
  Setup_Web_Server();

  Radio.write(&Tx_Packet, sizeof(Tx_Packet));
}

void loop()
{
  Server.handleClient();
  Update_Wifi_Status_Led();

  uint32_t Now = millis();
  if (Now - Last_Tx_Millis >= Tx_Interval_Ms)
  {
    Last_Tx_Millis = Now;
    Radio.write(&Tx_Packet, sizeof(Tx_Packet));
  }
}
