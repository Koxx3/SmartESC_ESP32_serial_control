
// *******************************************************************
//  ESP32 example code
//
//  Copyright (C) 2020 Francois DESLANDES <koxx33@gmail.com>
//
// *******************************************************************

// *******************************************************************

#include <Arduino.h>

// ########################## DEFINES ##########################

#define DEBUG 0
#define DEBUG_SERIAL 0
#define TEST_DYNAMIC_FLUX 0
#define PATCHED_ESP32_FWK 1
#define START_AND_STOP 0
#define KICK_START 1

// serial
#define SERIAL_BAUD 921600        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define BAUD_RATE_SMARTESC 115200 //115200

// pinout
#define PIN_SERIAL_ESP_TO_CNTRL 27 //TX
#define PIN_SERIAL_CNTRL_TO_ESP 14 //RX
#define PIN_IN_ABRAKE 34           //Brake
#define PIN_IN_ATHROTTLE 39        //Throttle

// delays
#define TIME_SEND 20         // [ms] Sending time interval
#define TIME_SEND_ERROR 1000 // [ms] Sending time interval
#define DELAY_CMD 10

// send frame headers
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET 0x01
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET 0x02
#define SERIAL_START_FRAME_DISPLAY_TO_ESC_CMD 0x03 // [-] Start frame definition for serial commands
#define SERIAL_START_FRAME_ESC_TO_DISPLAY_OK 0xF0
#define SERIAL_START_FRAME_ESC_TO_DISPLAY_ERR 0xFF

// commandes
#define SERIAL_FRAME_CMD_START 0x01
#define SERIAL_FRAME_CMD_STOP 0x02
#define SERIAL_FRAME_CMD_FAULT_ACK 0x07

// registers
#define FRAME_REG_TARGET_MOTOR 0x00
#define FRAME_REG_FLAGS 0x01
#define FRAME_REG_STATUS 0x02
#define FRAME_REG_CONTROL_MODE 0x03
#define FRAME_REG_SPEED 0x04
#define FRAME_REG_TORQUE 0x08
#define FRAME_REG_TORQUE_KP 0x09
#define FRAME_REG_TORQUE_KI 0x0A
#define FRAME_REG_FLUX_REF 0x0C
#define FRAME_REG_FLUX_KI 0x0D
#define FRAME_REG_FLUX_KP 0x0E
#define FRAME_REG_SPEED_MEASURED 0x1E

// motor orders
#define THROTTLE_TO_TORQUE_FACTOR 90 // 128 for max
#define BRAKE_TO_TORQUE_FACTOR 80
#define THROTTLE_MINIMAL_TORQUE 0

#define MIN_KICK_START_RPM 30 // minimal RPM speed before applying torque -- used only if KICK_START is enabled 
#define MIN_BRAKE_RPM 30 // minimal RPM speed for electric brake
#define TORQUE_KP 200 // divided by 1024
#define TORQUE_KI 50  // divided by 16384
#define FLUX_KP 1800  // divided by 1024 // default 3649
#define FLUX_KI 1000  // divided by 16384 // default 1995
#define STARUP_FLUX_REFERENCE 0

#define SECURITY_OFFSET 100

// Global variables
uint8_t idx = 0;       // Index for new data pointer
uint8_t bufStartFrame; // Buffer Start Frame
byte *p;               // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
uint8_t receiveBuffer[1000]; // Buffer Start Frame

// Trottle
int32_t analogValueThrottle = 0;
uint16_t analogValueThrottleRaw = 0;
uint16_t analogValueThrottleMinCalibRaw = 0;

// Brake
int32_t analogValueBrake = 0;
uint16_t analogValueBrakeRaw = 0;
uint16_t analogValueBrakeMinCalibRaw = 0;

uint32_t lastOrderType;
uint32_t lastOrderValue;

int32_t speed = 0;
uint32_t flags = 0;
int16_t torque = 0;

unsigned long timeSend = 0;
unsigned long timeLastReply;
int8_t state = 0;
uint32_t iLoop = 0;

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
SerialRegSet16 regSetS16;

#pragma pack(push, 1)
typedef struct
{
  uint8_t Frame_start;
  uint8_t Lenght;
  uint8_t Reg;
  uint16_t Value;
  uint8_t CRC8;
} __attribute__((packed)) SerialRegSetU16;
#pragma pack(pop)
SerialRegSetU16 regSetU16;

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

/** @name Fault source error codes */
/** @{ */
#define MC_NO_ERROR (uint16_t)(0x0000u)     /**< @brief No error.*/
#define MC_NO_FAULTS (uint16_t)(0x0000u)    /**< @brief No error.*/
#define MC_FOC_DURATION (uint16_t)(0x0001u) /**< @brief Error: FOC rate to high.*/
#define MC_OVER_VOLT (uint16_t)(0x0002u)    /**< @brief Error: Software over voltage.*/
#define MC_UNDER_VOLT (uint16_t)(0x0004u)   /**< @brief Error: Software under voltage.*/
#define MC_OVER_TEMP (uint16_t)(0x0008u)    /**< @brief Error: Software over temperature.*/
#define MC_START_UP (uint16_t)(0x0010u)     /**< @brief Error: Startup failed.*/
#define MC_SPEED_FDBK (uint16_t)(0x0020u)   /**< @brief Error: Speed feedback.*/
#define MC_BREAK_IN (uint16_t)(0x0040u)     /**< @brief Error: Emergency input (Over current).*/
#define MC_SW_ERROR (uint16_t)(0x0080u)     /**< @brief Software Error.*/

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
    //Serial.printf("crc = %02x\n", crc);
  }

  uint8_t finalCrc = (uint8_t)(crc & 0xff) + ((crc >> 8) & 0xff);
  //Serial.printf("finalCrc = %02x\n", finalCrc);
  //Serial.printf("\n");
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

  // store last command
  lastOrderType = SERIAL_START_FRAME_DISPLAY_TO_ESC_CMD;
  lastOrderValue = cmd;
}

void GetReg(uint8_t reg)
{
  // Create command
  command.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET;
  command.Lenght = 1;
  command.Command = reg;
  command.CRC8 = getCrc((uint8_t *)&command, sizeof(command));

  displayBuffer((uint8_t *)&command, sizeof(command));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&command, sizeof(command));
  // store last command
  lastOrderType = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET;
  lastOrderValue = reg;
}

void SetRegU16(uint8_t reg, uint16_t val)
{
  // Create command
  regSetU16.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSetU16.Lenght = 3;
  regSetU16.Reg = reg;
  regSetU16.Value = val;
  regSetU16.CRC8 = getCrc((uint8_t *)&regSetU16, sizeof(regSetU16));

  displayBuffer((uint8_t *)&regSetU16, sizeof(regSetU16));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSetU16, sizeof(regSetU16));

  // store last command
  lastOrderType = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  lastOrderValue = reg;
}

void SetRegS16(uint8_t reg, int16_t val)
{
  // Create command
  regSetS16.Frame_start = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  regSetS16.Lenght = 3;
  regSetS16.Reg = reg;
  regSetS16.Value = val;
  regSetS16.CRC8 = getCrc((uint8_t *)&regSetS16, sizeof(regSetS16));

  displayBuffer((uint8_t *)&regSetS16, sizeof(regSetS16));

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&regSetS16, sizeof(regSetS16));

  // store last command
  lastOrderType = SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_SET;
  lastOrderValue = reg;
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

    timeLastReply = millis();

#if DEBUG_SERIAL
    // display frame
    Serial.print("   received : ");
    for (int i = 0; i < nbBytes; i++)
    {
      Serial.printf("%02x ", receiveBuffer[i]);
    }
    Serial.println();
#endif

    int iFrame = 0;
    bool continueReading = true;
    uint16_t msgSize = 0;
    while (continueReading)
    {
      msgSize = receiveBuffer[iFrame + 1];
#if DEBUG_SERIAL
      Serial.printf("   size = %02x\n", msgSize);
#endif
      if (receiveBuffer[iFrame] == SERIAL_START_FRAME_ESC_TO_DISPLAY_OK)
      {
        Serial.printf("   ==> ok");
      }
      else if (receiveBuffer[iFrame] == SERIAL_START_FRAME_ESC_TO_DISPLAY_ERR)
      {
        Serial.printf("   ==> KO !!!!!!!!!");
      }

      if (msgSize == 0)
      {
        Serial.println("   ===> CMD or REG_SET");
      }
      else if (msgSize == 1)
      {
        Serial.printf("   ===> value = %02x\n", receiveBuffer[iFrame + 2]);

        if (lastOrderType == SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET)
        {
          if ((receiveBuffer[iFrame] == SERIAL_START_FRAME_ESC_TO_DISPLAY_OK) && ((receiveBuffer[iFrame + 2] == FAULT_NOW) || (receiveBuffer[iFrame + 2] == FAULT_OVER)) && (state >= 8))
          {
            state = -1;
            Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
          }
        }
      }
      else if (msgSize == 2)
      {
        Serial.printf("   ===> unknonw");
      }
      else if (msgSize == 4)
      {

        //        Serial.printf("   ===> lastOrderType = %d / lastOrderValue : %d\n", lastOrderType, lastOrderValue);

        if ((lastOrderType == SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET) && (lastOrderValue == FRAME_REG_SPEED_MEASURED))
        {
          memcpy(&speed, &(receiveBuffer[iFrame + 2]), 4);
          Serial.printf("   ===> speed : %d\n", speed);
        }
        else if ((lastOrderType == SERIAL_START_FRAME_DISPLAY_TO_ESC_REG_GET) && (lastOrderValue == FRAME_REG_FLAGS))
        {
          memcpy(&flags, &(receiveBuffer[iFrame + 2]), 4);
          Serial.printf("   ===> flags : %08x\n", flags);

          // try current and last faults
          if (flags >> 16 == 0)
            flags = flags & 0xffff;
          else
            flags = flags >> 16;

          // decode faults
          if (flags != 0)
          {
            Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! FLASG ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

            if (flags == MC_NO_ERROR)
            {
              Serial.printf("   ===> flags : MC_NO_ERROR\n");
            }
            else if (flags == MC_NO_FAULTS)
            {
              Serial.printf("   ===> flags : MC_NO_FAULTS\n");
            }
            else if (flags == MC_FOC_DURATION)
            {
              Serial.printf("   ===> flags : MC_FOC_DURATION\n");
            }
            else if (flags == MC_OVER_VOLT)
            {
              Serial.printf("   ===> flags : MC_OVER_VOLT\n");
            }
            else if (flags == MC_UNDER_VOLT)
            {
              Serial.printf("   ===> flags : MC_UNDER_VOLT\n");
            }
            else if (flags == MC_OVER_TEMP)
            {
              Serial.printf("   ===> flags : MC_OVER_TEMP\n");
            }
            else if (flags == MC_START_UP)
            {
              Serial.printf("   ===> flags : MC_START_UP\n");
            }
            else if (flags == MC_SPEED_FDBK)
            {
              Serial.printf("   ===> flags : MC_SPEED_FDBK\n");
            }
            else if (flags == MC_BREAK_IN)
            {
              Serial.printf("   ===> flags : MC_BREAK_IN\n");
            }
            else if (flags == MC_SW_ERROR)
            {
              Serial.printf("   ===> flags : MC_SW_ERROR\n");
            }
            else
            {
              Serial.printf("   ===> flags : unkown\n");
            }
          }
        }
      }
      else
      {
        Serial.printf("   ===> ko (unknonw data) ----------------------------------------\n");
      }

#if DEBUG_SERIAL
      Serial.printf("   nbBytes = %d / iFrame = %d / nextIFrame = %d\n", nbBytes, iFrame, msgSize + 3);
#endif
      iFrame += msgSize + 3;

#if DEBUG_SERIAL
      Serial.printf("   next msg : iFrame = %d / msgSize = %02x\n", iFrame, msgSize);
#endif

      if (iFrame < nbBytes)
      {
        continueReading = true;
#if DEBUG_SERIAL
        Serial.println("      continue");
#endif
      }
      else
      {
        continueReading = false;
#if DEBUG_SERIAL
        Serial.println("      stop");
#endif
      }
    }

    Serial.println();
  }
}

// ########################## THROTTLE / BRAKE ##########################

void readAnalogData()
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

  Serial.println("throttleRaw = " + (String)analogValueThrottleRaw + " / throttleMinCalibRaw = " +                            //
                 (String)analogValueThrottleMinCalibRaw + " / throttle = " + (String)analogValueThrottle + " / brakeRaw = " + //
                 (String)analogValueBrakeRaw + " / brakeMinCalibRaw = " + (String)analogValueBrakeMinCalibRaw +               //
                 " / brake = " + (String)analogValueBrake + " / torque = " + (String)torque + " / speed = " + (String)speed);
}

// ########################## LOOP ##########################

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // Avoid delay
  if (timeSend > timeNow)
    return;
  timeSend = timeNow + TIME_SEND;

  // no reply received for a long time... restart at state 0
  if (timeLastReply > timeNow + TIME_SEND_ERROR)
    state = 0;

  if (state == -1)
  {
    timeSend = timeNow + TIME_SEND_ERROR;
    state++;
  }
  else if (state == 0)
  {

    analogValueThrottleRaw = 0;
    analogValueBrakeRaw = 0;
    torque = 0;

    Serial.printf("%d / send : GET REG FRAME_REG_STATUS : ", state);
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 1)
  {
    Serial.printf("%d / send : CMD STOP : ", state);
    SendCmd(SERIAL_FRAME_CMD_STOP);
    state++;

    delay(DELAY_CMD);
  }

  else if (state == 2)
  {
    Serial.printf("%d / send : GET REG FRAME_REG_FLAGS : ", state);
    GetReg(FRAME_REG_FLAGS);
    state++;
  }

  else if (state == 3)
  {
    Serial.printf("%d / send : GET REG FRAME_REG_STATUS : ", state);
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 4)
  {
    Serial.printf("%d / send : CMD FAULT_ACK : ", state);
    SendCmd(SERIAL_FRAME_CMD_FAULT_ACK);
    state++;
  }

  else if (state == 5)
  {

    Serial.printf("%d / send : SET REG CONTROL_MODE : ", state);
    SetRegU16(FRAME_REG_CONTROL_MODE, 0x00);

    delay(10);

    Serial.printf("%d / send : REG_TORQUE_KI : ", state);
    SetRegU16(FRAME_REG_TORQUE_KI, TORQUE_KI);

    delay(10);

    Serial.printf("%d / send : REG_TORQUE_KP : ", state);
    SetRegU16(FRAME_REG_TORQUE_KP, TORQUE_KP);

    delay(10);

#if TEST_DYNAMIC_FLUX
    Serial.printf("%d / send : REG_FLUX_KI : ", state);
    SetRegU16(FRAME_REG_FLUX_KI, FLUX_KI);

    delay(10);

    Serial.printf("%d / send : REG_FLUX_KP : ", state);
    SetRegU16(FRAME_REG_FLUX_KP, FLUX_KP);

    delay(10);

    Serial.printf("%d / send : REG_FLUX_REF : ", state);
    SetRegU16(FRAME_REG_FLUX_REF, STARUP_FLUX_REFERENCE);
#endif

    state++;

    delay(DELAY_CMD);
  }

  else if (state == 6)
  {
    Serial.printf("%d / send : GET REG FRAME_REG_FLAGS : ", state);
    GetReg(FRAME_REG_FLAGS);
    state++;

    delay(500);
  }
  else if (state == 7)
  {

    Serial.printf("%d / send : CMD START : ", state);
    SendCmd(SERIAL_FRAME_CMD_START);

    state++;

    delay(DELAY_CMD);
  }
  else if (state == 8)
  {

    Serial.printf("%d / send : GET REG FRAME_REG_FLAGS : ", state);
    GetReg(FRAME_REG_FLAGS);
    state++;
  }
  else if (state == 9)
  {
    Serial.printf("%d / send : GET REG FRAME_REG_STATUS : ", state);
    GetReg(FRAME_REG_STATUS);
    state++;
  }
  else if (state == 10)
  {

    readAnalogData();

    // Send torque commands
    if (analogValueBrake > 0)
    {
      if (speed > MIN_BRAKE_RPM)
      {
        torque = -analogValueBrake * BRAKE_TO_TORQUE_FACTOR;
      }
      else
      {
        torque = 0;
      }
    }
    else if (analogValueThrottle > 0)
    {
#if KICK_START
      if (speed >= MIN_KICK_START_RPM)
      {
        torque = THROTTLE_MINIMAL_TORQUE + (analogValueThrottle * THROTTLE_TO_TORQUE_FACTOR);
      }
#else
      torque = THROTTLE_MINIMAL_TORQUE + (analogValueThrottle * THROTTLE_TO_TORQUE_FACTOR);
#endif
    }
    else
    {
      torque = 0;
    }

    Serial.printf("%d / torque : %d\n", state, torque);
    Serial.printf("%d / send : SET REG FRAME_REG_TORQUE : ", state);
    SetRegS16(FRAME_REG_TORQUE, torque);

#if RAMP_ENABLED
#define RAMP 400
    if (iLoop % RAMP < RAMP / 2)
      torque = iLoop % RAMP;
    else
      torque = (RAMP / 2) - ((iLoop % RAMP) - (RAMP / 2));
#endif

    state++;
  }
  else if (state == 11)
  {

    Serial.printf("%d / send : GET REG FRAME_REG_FLAGS : ", state);
    GetReg(FRAME_REG_FLAGS);
    state++;
  }

  else if (state == 12)
  {
    Serial.printf("%d / send : GET REG FRAME_REG_STATUS : ", state);
    GetReg(FRAME_REG_STATUS);

    state++;
  }

  else if (state == 13)
  {

#if TEST_DYNAMIC_FLUX
    if (speed > 100)
    {
      Serial.printf("%d / send : SET REG FRAME_REG_FLUX_REF : ", state);
      SetRegU16(FRAME_REG_FLUX_REF, 0);
    }
#endif

    state++;
  }
  else if (state == 14)
  {

    Serial.printf("%d / send : GET REG SPEED : ", state);
    GetReg(FRAME_REG_SPEED_MEASURED);
    state++;
  }

  else if (state == 15)
  {
#if START_AND_STOP
    if ((speed > 0) || (analogValueThrottle > 0))
    {
      state = state - 2;
    }
    else
    {
      state++;

      delay(500);

      Serial.printf("%d / send : CMD STOP : ", state);
      SendCmd(SERIAL_FRAME_CMD_STOP);

      delay(500);
    }
#else
    state = state - 7;
#endif
  }
  else if (state == 16)
  {

    Serial.printf("%d / send : GET REG SPEED : ", state);
    GetReg(FRAME_REG_SPEED_MEASURED);

    readAnalogData();

    if ((speed > 0) || (analogValueThrottle > 0))
    {
      state = 5;
    }
  }

  delay(5); // 20 Hz orders

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
#if PATCHED_ESP32_FWK
  hwSerCntrl.setUartIrqIdleTrigger(1);
#endif
}

// ########################## END ##########################
