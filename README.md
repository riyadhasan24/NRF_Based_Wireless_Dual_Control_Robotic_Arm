# NRF_Based_Wireless_Dual_Control_Robotic_Arm

Wireless Robotic Arm Controller (ESP32 + NRF24L01)

This project controls a 4-DOF robotic arm wirelessly using an ESP32 transmitter and an NRF24L01 radio link.
The arm can be operated in two modes:
Manual potentiometer mode and Wi-Fi slider mode through a built-in web interface.

Features,
4-axis control: Base, Shoulder, Elbow, Gripper
Mode-1: Potentiometer control (analog input)
Mode-2: Wi-Fi control using sliders in the browser
Long-press button switches mode
NRF24L01 wireless packet transmission
LED indicators for mode & Wi-Fi status

Pins (ESP32 Transmitter),
Potentiometers:
  Base → GPIO36
  Shoulder → GPIO39
  Elbow → GPIO34
  Gripper → GPIO35

NRF24L01:
  CE → GPIO33
  CSN → GPIO32

Control:
  Mode Button → GPIO4
  Buzzer → GPIO2

LEDs:
  Mode-1 → GPIO25
  Mode-2 → GPIO26
  Wi-Fi Status → GPIO14

  Pins (ATmega8A / Arduino Receiver)
  Servos:
  Base     → D5
  Shoulder → D6
  Elbow    → D9
  Gripper  → D10

NRF24L01:
  CE  → D7
  CSN → D8
  MOSI → D11
  MISO → D12
  SCK  → D13
  VCC → 3.3V

How to Use,
Power on → Starts in Mode-1 (pot control)
Hold button ~1.2s → Switch to Mode-2 (Wi-Fi)

In Mode-2:
Connect to Wi-Fi → SSID: Wireless Robotic Arm
Open browser → http://192.168.4.1
Move sliders → Arm responds wirelessly


📩 Contact,
For help, suggestions, or collaboration, feel free to reach out:
    **📧 Email: riyadhasan24a@gmail.com
    📱 WhatsApp: +88 01730 288553
