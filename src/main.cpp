
// *******************************************************************
//  ESP32 example code
//
//  Copyright (C) 2020 Francois DESLANDES <koxx33@gmail.com>
//
// *******************************************************************

// *******************************************************************

#include <Arduino.h>

// ########################## DEFINES ##########################
#define SERIAL_BAUD 921600 // [-] Baud rate for built-in Serial (used for the Serial Monitor)

// send command
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_CMD 0x03     // [-] Start frame definition for serial commands
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET 0x02 // [-] Start frame definition for serial commands
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET 0x01 // [-] Start frame definition for serial commands

#define SERIAL_START_FRAME_ESC_TO_DISPLAY 0xFF // [-] Start frame definition for serial commands

#define SERIAL_FRAME_CMD_START 0x01
#define SERIAL_FRAME_CMD_STOP 0x02
#define SERIAL_FRAME_CMD_FAULT_ACK 0x07

#define FRAME_REG_TARGET_MOTOR 0x00
#define FRAME_REG_CONTROL_MODE 0x03
#define FRAME_REG_SPEED 0x04
#define FRAME_REG_TORQUE 0x08
#define FRAME_REG_SPEED_MEESURED 0x1E

#define THROTTLE_TO_TORQUE_FOCTOR 1.5

#define TIME_SEND 10 // [ms] Sending time interval

// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define DEBUG 0
#define PIN_IN_ABRAKE 34
#define PIN_IN_ATHROTTLE 39
#define SECURITY_OFFSET 50

#define BAUD_RATE_SMARTESC 115200
#define PIN_SERIAL_ESP_TO_CNTRL 27
#define PIN_SERIAL_CNTRL_TO_ESP 14

// Global variables
uint8_t idx = 0;       // Index for new data pointer
uint8_t bufStartFrame; // Buffer Start Frame
byte *p;               // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
uint8_t receiveBuffer[16]; // Buffer Start Frame

// Trottle
int32_t analogValueThrottle = 0;
uint16_t analogValueThrottleRaw = 0;
uint16_t analogValueThrottleMinCalibRaw = 0;

// Brake
int32_t analogValueBrake = 0;
uint16_t analogValueBrakeRaw = 0;
uint16_t analogValueBrakeMinCalibRaw = 0;

char print_buffer[500];

#pragma pack(push, 1)
typedef struct
{
  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Command;
  uint8_t CRC8;
} __attribute__((packed)) SerialCommand;
#pragma pack(pop)
SerialCommand command;

#pragma pack(push, 1)
typedef struct
{
  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Reg;
  int32_t Value;
  uint8_t CRC8;
} __attribute__((packed)) SerialRegSet32;
#pragma pack(pop)
SerialRegSet32 regSet32;

#pragma pack(push, 1)
typedef struct
{
  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Reg;
  int16_t Value;
  uint8_t CRC8;
} __attribute__((packed)) SerialRegSet16;
#pragma pack(pop)
SerialRegSet16 regSet16;

#pragma pack(push, 1)
typedef struct
{
  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Reg;
  int8_t Value;
  uint8_t CRC8;
} __attribute__((packed)) SerialRegSet8;
#pragma pack(pop)
SerialRegSet8 regSet8;

#pragma pack(push, 1)
typedef struct
{

  uint8_t Frame_start;
  uint8_t Error;
  uint8_t CRC8;
} __attribute__((packed)) SerialFeedbackOk;
#pragma pack(pop)
SerialFeedbackOk feedbackOk;
SerialFeedbackOk newFeedbackOk;

#pragma pack(push, 1)
typedef struct
{

  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Error;
  uint8_t CRC8;
} __attribute__((packed)) SerialFeedbackError;
#pragma pack(pop)
SerialFeedbackError feedbackError;

HardwareSerial hwSerCntrl(1);

void displayBuffer(uint8_t *buffer, uint8_t size)
{
  for (int i = 0; i < size; i++)
  {
    Serial.printf("%02x ", buffer[i]);
  }
  Serial.printf("\n");
}

// ########################## SEND ##########################

void SendCmd(uint8_t cmd)
{
  // Create command
  command.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_CMD;
  command.Lenght = 1;
  command.Command = cmd;

  uint16_t sum =
      command.Frame_start + command.Lenght //
      + command.Command;
  command.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&command, sizeof(command));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&command, sizeof(command));
}

void SendSpeed(int16_t brake, int16_t throttle)
{
  // Create command
  regSet32.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSet32.Lenght = 5;
  regSet32.Reg = FRAME_REG_SPEED;
  regSet32.Value = 0x0010000;

  uint16_t sum = 0;
  for (int i = 0; i < sizeof(regSet32); i++)
  {
    sum += ((uint8_t *)(&regSet32))[i];
  }
  regSet32.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&regSet32, sizeof(regSet32));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSet32, sizeof(regSet32));
}

void SendTorque(int16_t brake, int16_t throttle)
{
  // Create command
  regSet16.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSet16.Lenght = 3;
  regSet16.Reg = FRAME_REG_TORQUE;
  regSet16.Value = throttle;

  uint16_t sum =
      regSet16.Frame_start + regSet16.Lenght //
      + regSet16.Value + regSet16.Reg;
  regSet16.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&regSet16, sizeof(regSet16));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSet16, sizeof(regSet16));
}

void SendControlMode(int8_t mode)
{
  // Create command
  regSet8.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSet8.Lenght = 2;
  regSet8.Reg = FRAME_REG_CONTROL_MODE;
  regSet8.Value = mode;

  uint16_t sum =
      regSet8.Frame_start + regSet8.Lenght //
      + regSet8.Value + regSet8.Reg;
  regSet8.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&regSet8, sizeof(regSet8));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSet8, sizeof(regSet8));
}

void SendMode(uint8_t mode)
{
  // Create command
  regSet8.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSet8.Lenght = 5;
  regSet8.Reg = FRAME_REG_CONTROL_MODE;
  regSet8.Value = mode;

  uint16_t sum =
      regSet8.Frame_start + regSet8.Lenght //
      + regSet8.Value + regSet8.Reg;
  regSet8.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&regSet8, sizeof(regSet8));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSet8, sizeof(regSet8));
}


void GetReg(uint8_t reg)
{
  // Create command
  command.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET;
  command.Lenght = 1;
  command.Command = reg;

  uint16_t sum =
      command.Frame_start + command.Lenght //
      + command.Command;
  command.CRC8 = (sum & 0xff) + ((sum >> 8) & 0xff);

  displayBuffer((uint8_t *)&command, sizeof(command));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&command, sizeof(command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  uint16_t nbBytes = 0;
  int32_t speed;

  // Check for new data availability in the Serial buffer
  while (hwSerCntrl.available())
  {
    incomingByte = hwSerCntrl.read();      // Read the incoming byte
    receiveBuffer[nbBytes] = incomingByte; // Construct the start frame

    nbBytes++;
  }
  if (nbBytes > 0)
  {
    Serial.print("received : ");
    for (int i = 0; i < nbBytes; i++)
    {
      Serial.printf("%02x ", receiveBuffer[i]);
    }
    Serial.println();

    if (nbBytes == 3)
    {
      Serial.println("=> ok");
    }
    else if (nbBytes == 4)
    {
      Serial.printf("=> ko (err : %d)\n", receiveBuffer[2]);
    }
    else if (nbBytes == 7)
    {
      memcpy(&speed, &(receiveBuffer[2]), 4);
      Serial.printf("=> ok / speed : %d\n", speed);

    }

  }
}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
unsigned long torque = 0;
uint8_t state = 0;
uint32_t iLoop = 0;

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Avoid delay
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;

  if (state == 0)
  {

    Serial.println("send : CMD STOP");
    SendCmd(SERIAL_FRAME_CMD_STOP);
    state++;
  }
  else if (state == 1)
  {

    Serial.println("send : CMD FAULT_ACK");
    SendCmd(SERIAL_FRAME_CMD_FAULT_ACK);
    state++;
  }
  else if (state == 2)
  {

    Serial.println("send : CMD START");
    SendCmd(SERIAL_FRAME_CMD_START);
    state++;
  }
  else if (state == 3)
  {

    Serial.println("send : SET REG CONTROL_MODE");
    SendControlMode(0);
    state++;
  }
  else if (state == 4)
  {

    // Compute throttle
    analogValueThrottleRaw = analogRead(PIN_IN_ATHROTTLE);
    analogValueThrottle = analogValueThrottleRaw - analogValueThrottleMinCalibRaw - SECURITY_OFFSET;
    analogValueThrottle = analogValueThrottle / 4;
    if (analogValueThrottle > 255)
      analogValueThrottle = 255;
    if (analogValueThrottle < 0)
      analogValueThrottle = 0;

    // Compute brake
    analogValueBrakeRaw = analogRead(PIN_IN_ABRAKE);
    analogValueBrake = analogValueBrakeRaw - analogValueBrakeMinCalibRaw - SECURITY_OFFSET;
    analogValueBrake = analogValueBrake / 3;
    if (analogValueBrake > 255)
      analogValueBrake = 255;
    if (analogValueBrake < 0)
      analogValueBrake = 0;

    Serial.println("analogValueThrottleRaw = " + (String)analogValueThrottleRaw + " / analogValueThrottleMinCalibRaw = " + (String)analogValueThrottleMinCalibRaw + " / analogValueThrottle = " + (String)analogValueThrottle);
#if DEBUG
    Serial.println("analogValueBrakeRaw = " + (String)analogValueBrakeRaw + " / analogValueBrakeMinCalibRaw = " + (String)analogValueBrakeMinCalibRaw + " / analogValueBrake = " + (String)analogValueBrake);
#endif

    // Send commands
    Serial.println("send : SET REG TORQUE");
//    SendTorque(analogValueBrake, torque);
    SendTorque(analogValueBrake, analogValueThrottle * THROTTLE_TO_TORQUE_FOCTOR);

#define RAMP 400
    if (iLoop % RAMP < RAMP / 2)
      torque = iLoop % RAMP;
    else
      torque = (RAMP / 2) - ((iLoop % RAMP) - (RAMP / 2));
    state++;
  }
  else if (state == 5)
  {

    Serial.println("command to send : GET REG SPEED");
    GetReg(FRAME_REG_SPEED_MEESURED);
    state--;
  }

  iLoop++;
}

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("SmartESC Serial v2.0");

  pinMode(PIN_IN_ATHROTTLE, INPUT);
  pinMode(PIN_IN_ABRAKE, INPUT);

  // do it twice to improve values
  analogValueThrottleMinCalibRaw = analogRead(PIN_IN_ATHROTTLE);
  analogValueThrottleMinCalibRaw = analogRead(PIN_IN_ATHROTTLE);
  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);
  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);

  hwSerCntrl.begin(BAUD_RATE_SMARTESC, SERIAL_8N1, PIN_SERIAL_CNTRL_TO_ESP, PIN_SERIAL_ESP_TO_CNTRL);
  hwSerCntrl.setUartIrqIdleTrigger(1);
}

// ########################## END ##########################
