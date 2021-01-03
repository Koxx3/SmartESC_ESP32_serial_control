
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
#define FRAME_REG_STATUS 0x02
#define FRAME_REG_CONTROL_MODE 0x03
#define FRAME_REG_SPEED 0x04
#define FRAME_REG_TORQUE 0x08
#define FRAME_REG_SPEED_MEESURED 0x1E

#define THROTTLE_TO_TORQUE_FOCTOR 10
#define THROTTLE_TO_BRAKE_FOCTOR 1

#define TIME_SEND 10 // [ms] Sending time interval

#define DELAY_CMD 200

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

int32_t speed;

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

typedef enum
{
  ICLWAIT = 12,               /*!< Persistent state, the system is waiting for ICL
                           deactivation. Is not possible to run the motor if
                           ICL is active. Until the ICL is active the state is
                           forced to ICLWAIT, when ICL become inactive the state
                           is moved to IDLE */
  IDLE = 0,                   /*!< Persistent state, following state can be IDLE_START
                           if a start motor command has been given or
                           IDLE_ALIGNMENT if a start alignment command has been
                           given */
  IDLE_ALIGNMENT = 1,         /*!< "Pass-through" state containg the code to be executed
                           only once after encoder alignment command.
                           Next states can be ALIGN_CHARGE_BOOT_CAP or
                           ALIGN_OFFSET_CALIB according the configuration. It
                           can also be ANY_STOP if a stop motor command has been
                           given. */
  ALIGN_CHARGE_BOOT_CAP = 13, /*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           ALIGN_OFFSET_CALIB. It can also be ANY_STOP if a stop
                           motor command has been given. */
  ALIGN_OFFSET_CALIB = 14,    /*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           ALIGN_CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  ALIGN_CLEAR = 15,           /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be ALIGNMENT. It can also be ANY_STOP
                           if a stop motor command has been given. */
  ALIGNMENT = 2,              /*!< Persistent state in which the encoder are properly
                           aligned to set mechanical angle, following state can
                           only be ANY_STOP */
  IDLE_START = 3,             /*!< "Pass-through" state containg the code to be executed
                           only once after start motor command.
                           Next states can be CHARGE_BOOT_CAP or OFFSET_CALIB
                           according the configuration. It can also be ANY_STOP
                           if a stop motor command has been given. */
  CHARGE_BOOT_CAP = 16,       /*!< Persistent state where the gate driver boot
                           capacitors will be charged. Next states will be
                           OFFSET_CALIB. It can also be ANY_STOP if a stop motor
                           command has been given. */
  OFFSET_CALIB = 17,          /*!< Persistent state where the offset of motor currents
                           measurements will be calibrated. Next state will be
                           CLEAR. It can also be ANY_STOP if a stop motor
                           command has been given. */
  CLEAR = 18,                 /*!< "Pass-through" state in which object is cleared and
                           set for the startup.
                           Next state will be START. It can also be ANY_STOP if
                           a stop motor command has been given. */
  START = 4,                  /*!< Persistent state where the motor start-up is intended
                           to be executed. The following state is normally
                           SWITCH_OVER or RUN as soon as first validated speed is
                           detected. Another possible following state is
                           ANY_STOP if a stop motor command has been executed */
  SWITCH_OVER = 19,           /**< TBD */
  START_RUN = 5,              /*!< "Pass-through" state, the code to be executed only
                           once between START and RUN states itâ€™s intended to be
                           here executed. Following state is normally  RUN but
                           it can also be ANY_STOP  if a stop motor command has
                           been given */
  RUN = 6,                    /*!< Persistent state with running motor. The following
                           state is normally ANY_STOP when a stop motor command
                           has been executed */
  ANY_STOP = 7,               /*!< "Pass-through" state, the code to be executed only
                           once between any state and STOP itâ€™s intended to be
                           here executed. Following state is normally STOP */
  STOP = 8,                   /*!< Persistent state. Following state is normally
                           STOP_IDLE as soon as conditions for moving state
                           machine are detected */
  STOP_IDLE = 9,              /*!< "Pass-through" state, the code to be executed only
                           once between STOP and IDLE itâ€™s intended to be here
                           executed. Following state is normally IDLE */
  FAULT_NOW = 10,             /*!< Persistent state, the state machine can be moved from
                           any condition directly to this state by
                           STM_FaultProcessing method. This method also manage
                           the passage to the only allowed following state that
                           is FAULT_OVER */
  FAULT_OVER = 11,            /*!< Persistent state where the application is intended to
                          stay when the fault conditions disappeared. Following
                          state is normally STOP_IDLE, state machine is moved as
                          soon as the user has acknowledged the fault condition.
                      */
  WAIT_STOP_MOTOR = 20

} State_t;

HardwareSerial hwSerCntrl(1);

void displayBuffer(uint8_t *buffer, uint8_t size)
{
  for (int i = 0; i < size; i++)
  {
    Serial.printf("%02x ", buffer[i]);
  }
  Serial.printf("\n");
}
uint8_t getCrc(uint8_t *buffer, uint8_t size)
{
  uint16_t crc = 0;
  for (int i = 0; i < size - 1; i++)
  {
    crc = crc + buffer[i];
    Serial.printf("crc = %02x\n", crc);
  }

  uint8_t finalCrc = (uint8_t)(crc & 0xff) + ((crc >> 8) & 0xff);
  Serial.printf("finalCrc = %02x\n", finalCrc);
  Serial.printf("\n");
  return finalCrc;
}

// ########################## SEND ##########################

void SendCmd(uint8_t cmd)
{
  // Create command
  command.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_CMD;
  command.Lenght = 1;
  command.Command = cmd;
  command.CRC8 = getCrc((uint8_t *)&command, sizeof(command));

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
  regSet32.CRC8 = getCrc((uint8_t *)&regSet32, sizeof(regSet32));

  displayBuffer((uint8_t *)&regSet32, sizeof(regSet32));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSet32, sizeof(regSet32));
}

void SendTorque(int16_t brake, int16_t throttle)
{
  int16_t torque = 0;

  if ((brake > 0) && (speed > 0))
    torque = -brake;
  else
    torque = throttle;

  Serial.printf("torque = %02x\n", torque);

  // Create command
  regSet16.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSet16.Lenght = 3;
  regSet16.Reg = FRAME_REG_TORQUE;
  regSet16.Value = torque;
  regSet16.CRC8 = getCrc((uint8_t *)&regSet16, sizeof(regSet16));

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
  regSet8.CRC8 = getCrc((uint8_t *)&regSet16, sizeof(regSet16)); // (sum & 0xff) + ((sum >> 8) & 0xff);

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
  regSet8.CRC8 = getCrc((uint8_t *)&regSet8, sizeof(regSet8));

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
  command.CRC8 = getCrc((uint8_t *)&command, sizeof(command));

  displayBuffer((uint8_t *)&command, sizeof(command));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&command, sizeof(command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  uint16_t nbBytes = 0;

  // Check for new data availability in the Serial buffer
  while (hwSerCntrl.available())
  {
    incomingByte = hwSerCntrl.read();      // Read the incoming byte
    receiveBuffer[nbBytes] = incomingByte; // Construct the start frame

    nbBytes++;
  }
  if (nbBytes > 0)
  {
    // display frame
    Serial.print("   received : ");
    for (int i = 0; i < nbBytes; i++)
    {
      Serial.printf("%02x ", receiveBuffer[i]);
    }
    Serial.println();

    if (receiveBuffer[0] == 0xff)
    {
      Serial.printf("   => ko (err : %d) !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", receiveBuffer[2]);
    }
    else
    {
      if (nbBytes == 3)
      {
        Serial.println("   => ok / CMD or REG_SET");
      }
      else if (nbBytes == 4)
      {
        Serial.printf("   => ok / status : %d\n", receiveBuffer[2]);
      }
      else if (nbBytes == 7)
      {
        memcpy(&speed, &(receiveBuffer[2]), 4);
        Serial.printf("   => ok / speed : %d\n", speed);
      }
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
    Serial.println("send : GET REG STATUS");
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 1)
  {
    Serial.println("send : CMD STOP");
    SendCmd(SERIAL_FRAME_CMD_STOP);
    state++;

    delay(DELAY_CMD);
  }

  else if (state == 2)
  {
    Serial.println("send : GET REG STATUS");
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 3)
  {

    Serial.println("send : CMD FAULT_ACK");
    SendCmd(SERIAL_FRAME_CMD_FAULT_ACK);
    state++;

    delay(DELAY_CMD);
  }

  else if (state == 4)
  {
    Serial.println("send : GET REG STATUS");
    GetReg(FRAME_REG_STATUS);
    state++;

    delay(DELAY_CMD);
  }
  else if (state == 5)
  {

    Serial.println("send : CMD START");
    SendCmd(SERIAL_FRAME_CMD_START);
    state++;

    delay(DELAY_CMD);
  }
  else if (state == 6)
  {
    Serial.println("send : GET REG STATUS");
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 7)
  {

    Serial.println("send : SET REG CONTROL_MODE");
    SendControlMode(1);
    state++;
  }
  else if (state == 8)
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

    Serial.println("analogValueThrottleRaw = " + (String)analogValueThrottleRaw + " / analogValueThrottleMinCalibRaw = " + (String)analogValueThrottleMinCalibRaw + " / analogValueThrottle = " + (String)analogValueThrottle + " / analogValueBrakeRaw = " + (String)analogValueBrakeRaw + " / analogValueBrakeMinCalibRaw = " + (String)analogValueBrakeMinCalibRaw + " / analogValueBrake = " + (String)analogValueBrake);

    // Send commands
    Serial.println("send : SET REG TORQUE");
    //    SendTorque(analogValueBrake, torque);
    SendTorque(analogValueBrake * THROTTLE_TO_BRAKE_FOCTOR, analogValueThrottle * THROTTLE_TO_TORQUE_FOCTOR);

#if RAMP_ENABLED
#define RAMP 400
    if (iLoop % RAMP < RAMP / 2)
      torque = iLoop % RAMP;
    else
      torque = (RAMP / 2) - ((iLoop % RAMP) - (RAMP / 2));
#endif

    state++;
  }
  else if (state == 9)
  {

    Serial.println("send : GET REG STATUS");
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 10)
  {

    Serial.println("send : GET REG SPEED");
    GetReg(FRAME_REG_SPEED_MEESURED);
    state = state - 2;
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
