
// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

#include <Arduino.h>

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 57600               // [-] Baud rate for Serial (used to communicate with the hoverboard)
#define SERIAL_BAUD 57600                     // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define SERIAL_START_FRAME_ESC_TO_DISPLAY 0x5A // [-] Start frame definition for serial commands
#define SERIAL_START_FRAME_DISPLAY_TO_ESC 0xA5 // [-] Start frame definition for serial commands
#define TIME_SEND 100                           // [ms] Sending time interval

// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define DEBUG                         1
#define PIN_IN_ABRAKE 34           //Brake
#define PIN_IN_ATHROTTLE 39        //Throttle
#define SECURITY_OFFSET               50

#define BAUD_RATE_SMARTESC            57600
#define PIN_SERIAL_ESP_TO_CNTRL       27
#define PIN_SERIAL_CNTRL_TO_ESP       14

// Global variables
uint8_t idx = 0;       // Index for new data pointer
uint8_t tx_buffer[12] = {0xAA, 0x00, 0x00,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
char USB_rx_buffer[64];
uint8_t USB_rx_buffer_pointer;
uint8_t bufStartFrame; // Buffer Start Frame
byte *p;               // Pointer declaration for the new received data
byte incomingByte[1];
byte incomingBytePrev;
String USB_command;

// Trottle
int32_t analogValueThrottle = 0;
uint16_t analogValueThrottleRaw = 0;
uint16_t analogValueThrottleMinCalibRaw = 0;

// Brake
int32_t analogValueBrake = 0;
uint16_t analogValueBrakeRaw = 0;
uint16_t analogValueBrakeMinCalibRaw = 0;

char print_buffer[500];

typedef struct
{
  uint8_t Frame_start;
  uint8_t Type;
  uint8_t Destination;
  uint8_t Number_of_ESC;
  uint8_t BMS_protocol;
  uint8_t ESC_Jumps;
  uint8_t Display_Version_Maj;
  uint8_t Display_Version_Main;
  uint8_t Power_ON;
  uint8_t Throttle;
  uint8_t Brake;
  uint8_t Torque;
  uint8_t Brake_torque;
  uint8_t Lock;
  uint8_t Regulator;
  uint8_t Motor_direction;
  uint8_t Hall_sensors_direction;
  uint8_t Ligth_power;
  uint8_t Max_temperature_reduce;
  uint8_t Max_temperature_shutdown;
  uint8_t Speed_limit_;
  uint8_t Motor_start_speed;
  uint8_t CRC8;
} SerialCommand;
SerialCommand command;

typedef struct
{
  //           
  uint8_t a = 0xAA; // 00
  uint8_t b = 0x00; // 00
  uint8_t c = 0x00; // 00
  uint8_t d = 0x06; // 10
  uint8_t e = 0x00; // 00
  uint8_t f = 0x00; // 00
  uint8_t g = 0x00; // 00
  uint8_t h = 0xff; // FF
  uint8_t i = 0x00; // 00
  uint8_t j = 0x00; // 00
  uint8_t k = 0x00; // 00
  uint8_t l = 0x00; // 00
} SerialCommand2;
SerialCommand2 command2;

typedef struct
{

  uint8_t Frame_start;
  uint8_t Type;
  uint8_t ESC_Version_Maj;
  uint8_t ESC_Version_Min;
  uint8_t Throttle;
  uint8_t Brake;
  uint8_t Controller_Voltage_LSB;
  uint8_t Controller_Voltage_MSB;
  uint8_t Controller_Current_LSB;
  uint8_t Controller_Current_MSB;
  uint8_t MOSFET_temperature;
  uint8_t ERPM_LSB;
  uint8_t ERPM_MSB;
  uint8_t Lock_status;
  uint8_t Ligth_status;
  uint8_t Regulator_status;
  uint8_t Phase_1_current_max_LSB;
  uint8_t Phase_1_current_max_MSB;
  uint8_t Phase_1_voltage_max_LSB;
  uint8_t Phase_1_voltage_max_MSB;
  uint8_t BMS_Version_Maj;
  uint8_t BMS_Version_Min;
  uint8_t BMS_voltage_LSB;
  uint8_t BMS_voltage_MSB;
  uint8_t BMS_Current_LSB;
  uint8_t BMS_Current_MSB;
  uint8_t BMS_Cells_status_group_1;
  uint8_t BMS_Cells_status_group_2;
  uint8_t BMS_Cells_status_group_3;
  uint8_t BMS_Cells_status_group_4;
  uint8_t BMS_Cells_status_group_5;
  uint8_t BMS_Cells_status_group_6;
  uint8_t BMS_Cells_status_group_7;
  uint8_t BMS_Cells_status_group_8;
  uint8_t BMS_Cells_status_group_9;
  uint8_t BMS_Cells_status_group_10;
  uint8_t BMS_Cells_status_group_11;
  uint8_t BMS_Cells_status_group_12;
  uint8_t BMS_Cells_status_group_13;
  uint8_t BMS_Cells_status_group_14;
  uint8_t BMS_Cells_status_group_15;
  uint8_t BMS_Cells_status_group_16;
  uint8_t BMS_Cells_status_group_17;
  uint8_t BMS_Cells_status_group_18;
  uint8_t BMS_Cells_status_group_19;
  uint8_t BMS_Cells_status_group_20;
  uint8_t BMS_Cells_status_group_21;
  uint8_t BMS_Cells_status_group_22;
  uint8_t BMS_Cells_status_group_23;
  uint8_t BMS_Cells_status_group_24;
  uint8_t BMS_Battery_tempature_1;
  uint8_t BMS_Battery_tempature_2;
  uint8_t BMS_Charge_cycles_full_LSB;
  uint8_t BMS_Charge_cycles_full_MSB;
  uint8_t BMS_Charge_cycles_partial_LSB;
  uint8_t BMS_Charge_cycles_partial_MSB;
  uint8_t Errors_LSB;
  uint8_t Errors_MSB;
  uint8_t CRC8;
} SerialFeedback;
SerialFeedback feedback;
SerialFeedback newFeedback;

HardwareSerial hwSerCntrl(1);
void Process_USB_command(String command);


// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("SmartESC Serial v1.0");

  pinMode(PIN_IN_ATHROTTLE, INPUT);
  pinMode(PIN_IN_ABRAKE, INPUT);

  // do it twice to improve values
  analogValueThrottleMinCalibRaw = analogRead(PIN_IN_ATHROTTLE);
  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);
  analogValueThrottleMinCalibRaw = analogRead(PIN_IN_ATHROTTLE);
  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);

  hwSerCntrl.begin(BAUD_RATE_SMARTESC, SERIAL_8N1, PIN_SERIAL_CNTRL_TO_ESP, PIN_SERIAL_ESP_TO_CNTRL);
}

// ########################## SEND ##########################
void Send(int16_t brake, int16_t throttle)
{
  /*
  // Create command
  command.Frame_start = (uint16_t)SERIAL_START_FRAME_DISPLAY_TO_ESC;
  command.Brake = (int16_t)brake;
  command.Throttle = (int16_t)throttle;

  command.CRC8 = (uint8_t)(
      command.Frame_start ^ command.Type //
      ^ command.Destination              //
      ^ command.Number_of_ESC            //
      ^ command.BMS_protocol             //
      ^ command.ESC_Jumps                //
      ^ command.Display_Version_Maj      //
      ^ command.Display_Version_Main     //
      ^ command.Power_ON                 //
      ^ command.Throttle                 //
      ^ command.Brake                    //
      ^ command.Torque                   //
      ^ command.Brake_torque             //
      ^ command.Lock                     //
      ^ command.Regulator                //
      ^ command.Motor_direction          //
      ^ command.Hall_sensors_direction   //
      ^ command.Ligth_power              //
      ^ command.Max_temperature_reduce   //
      ^ command.Max_temperature_shutdown //
      ^ command.Speed_limit_             //
      ^ command.Motor_start_speed        //
  );
*/

//00 00 00 10 00 00 00 FF 00 00 00 00

throttle=throttle *2;
tx_buffer[7] = (throttle & 0xff); 
tx_buffer[8] = (throttle >> 8) & 0xff; 
tx_buffer[4] = 0; 
uint8_t chkSum = 0;

for (uint8_t i = 0; i < 11; i++) {
			chkSum ^= tx_buffer[i];
}
tx_buffer[11] = chkSum;
//Serial.printf("command2.h = %02x / command2.i = %02x\n", command2.h, command2.i);

  // Write to Serial
  hwSerCntrl.write((uint8_t *)&tx_buffer, sizeof(tx_buffer));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (hwSerCntrl.available())
  {
    incomingByte[0] = hwSerCntrl.read(); // Read the incoming byte
    bufStartFrame = incomingByte[0];     // Construct the start frame
    Serial.write((uint8_t *)&incomingByte, 1); // pipe the character to the USB serial
  }
  else
  {
    return;
  }

  

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
//  Serial.printf("%02x\n", incomingByte);
#endif

  // Copy received data
  if ((bufStartFrame == SERIAL_START_FRAME_ESC_TO_DISPLAY) && (idx == 0))
  {
    // Initialize if new data is detected
    //Serial.println("SERIAL_START_FRAME_ESC_TO_DISPLAY detected");
    p = (byte *)&newFeedback;
    *p++ = incomingByte[0];
    idx = 1;
    //Serial.printf(">>>\nincomingByte = %02x / idx = %d\n",incomingByte, idx -1 );
  }
  else if (idx >= 1 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte[0];
    idx++;
    //Serial.printf("incomingByte = %02x / idx = %d\n",incomingByte, idx -1);
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {

    uint8_t checksum;
    checksum = (uint8_t)(
        //
        newFeedback.Frame_start                     //
        ^ newFeedback.Type                          //
        ^ newFeedback.ESC_Version_Maj               //
        ^ newFeedback.ESC_Version_Min               //
        ^ newFeedback.Throttle                      //
        ^ newFeedback.Brake                         //
        ^ newFeedback.Controller_Voltage_LSB        //
        ^ newFeedback.Controller_Voltage_MSB        //
        ^ newFeedback.Controller_Current_LSB        //
        ^ newFeedback.Controller_Current_MSB        //
        ^ newFeedback.MOSFET_temperature            //
        ^ newFeedback.ERPM_LSB                      //
        ^ newFeedback.ERPM_MSB                      //
        ^ newFeedback.Lock_status                   //
        ^ newFeedback.Ligth_status                  //
        ^ newFeedback.Regulator_status              //
        ^ newFeedback.Phase_1_current_max_LSB       //
        ^ newFeedback.Phase_1_current_max_MSB       //
        ^ newFeedback.Phase_1_voltage_max_LSB       //
        ^ newFeedback.Phase_1_voltage_max_MSB       //
        ^ newFeedback.BMS_Version_Maj               //
        ^ newFeedback.BMS_Version_Min               //
        ^ newFeedback.BMS_voltage_LSB               //
        ^ newFeedback.BMS_voltage_MSB               //
        ^ newFeedback.BMS_Current_LSB               //
        ^ newFeedback.BMS_Current_MSB               //
        ^ newFeedback.BMS_Cells_status_group_1      //
        ^ newFeedback.BMS_Cells_status_group_2      //
        ^ newFeedback.BMS_Cells_status_group_3      //
        ^ newFeedback.BMS_Cells_status_group_4      //
        ^ newFeedback.BMS_Cells_status_group_5      //
        ^ newFeedback.BMS_Cells_status_group_6      //
        ^ newFeedback.BMS_Cells_status_group_7      //
        ^ newFeedback.BMS_Cells_status_group_8      //
        ^ newFeedback.BMS_Cells_status_group_9      //
        ^ newFeedback.BMS_Cells_status_group_10     //
        ^ newFeedback.BMS_Cells_status_group_11     //
        ^ newFeedback.BMS_Cells_status_group_12     //
        ^ newFeedback.BMS_Cells_status_group_13     //
        ^ newFeedback.BMS_Cells_status_group_14     //
        ^ newFeedback.BMS_Cells_status_group_15     //
        ^ newFeedback.BMS_Cells_status_group_16     //
        ^ newFeedback.BMS_Cells_status_group_17     //
        ^ newFeedback.BMS_Cells_status_group_18     //
        ^ newFeedback.BMS_Cells_status_group_19     //
        ^ newFeedback.BMS_Cells_status_group_20     //
        ^ newFeedback.BMS_Cells_status_group_21     //
        ^ newFeedback.BMS_Cells_status_group_22     //
        ^ newFeedback.BMS_Cells_status_group_23     //
        ^ newFeedback.BMS_Cells_status_group_24     //
        ^ newFeedback.BMS_Battery_tempature_1       //
        ^ newFeedback.BMS_Battery_tempature_2       //
        ^ newFeedback.BMS_Charge_cycles_full_LSB    //
        ^ newFeedback.BMS_Charge_cycles_full_MSB    //
        ^ newFeedback.BMS_Charge_cycles_partial_LSB //
        ^ newFeedback.BMS_Charge_cycles_partial_MSB //
        ^ newFeedback.Errors_LSB                    //
        ^ newFeedback.Errors_MSB                    //
    );

    //Serial.printf("checksum = %02x / newFeedback.CRC8 = %02x\n",checksum, newFeedback.CRC8);

    // Check validity of the new data
    if (newFeedback.Frame_start == SERIAL_START_FRAME_ESC_TO_DISPLAY && checksum == newFeedback.CRC8)
    {
      // Copy the new data
      memcpy(&feedback, &newFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("Throttle = ");
      Serial.print(feedback.Throttle);
      Serial.print(" / Brake = ");
      Serial.print(feedback.Brake);
      Serial.println();
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte[0];
}

void Receive_USB()
{
  // Check for new data availability in the Serial buffer
  if (Serial.available())
  {
    USB_rx_buffer[USB_rx_buffer_pointer] = Serial.read(); // Read the incoming byte
    if(USB_rx_buffer[USB_rx_buffer_pointer]==0x0A && USB_rx_buffer[USB_rx_buffer_pointer-1]==0x0D){
    USB_command ="";
    for(int i = 0;i<USB_rx_buffer_pointer-1;i++)  USB_command+=(char)USB_rx_buffer[i];
    USB_rx_buffer_pointer=0;    
    Serial.print("received command:" + USB_command +"\r\n");
    Process_USB_command(USB_command);
    
    }
    else USB_rx_buffer_pointer++;    

  }
  else
  {
    return;
  }
}

void Process_USB_command(String command)
{
  
    if (command == "autodetect"){
      tx_buffer[7] = 0; 
      tx_buffer[8] = 0; 
      tx_buffer[4] = 1; 
      uint8_t chkSum = 0;

      for (uint8_t i = 0; i < 11; i++) {
			  chkSum ^= tx_buffer[i];
      }
      tx_buffer[11] = chkSum;
      // Write to Serial
      hwSerCntrl.write((uint8_t *)&tx_buffer, sizeof(tx_buffer));
      Serial.print("command executed!\r\n");
    }
    else if (command == "help"){
    Serial.print("Implemented commands:\r\n\autodetect : find motorspecific hall/phase configuration\r\n");
    }
    else Serial.print("unknown command\r\n");


}

// ########################## LOOP ##########################

unsigned long iTimeSend = 0;

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();
  Receive_USB();

  // Avoid delay
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;

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
  analogValueBrake = analogValueBrake / 4;
  if (analogValueBrake > 255)
    analogValueBrake = 255;
  if (analogValueBrake < 0)
    analogValueBrake = 0;

#if DEBUG
 // Serial.println("analogValueThrottleRaw = " + (String)analogValueThrottleRaw + " / analogValueThrottleMinCalibRaw = " + (String)analogValueThrottleMinCalibRaw+ " / analogValueThrottle = " + (String)analogValueThrottle);
//  Serial.println("analogValueBrakeRaw = " + (String)analogValueBrakeRaw + " / analogValueBrakeMinCalibRaw = " + (String)analogValueBrakeMinCalibRaw+ " / analogValueBrake = " + (String)analogValueBrake);
#endif

  // Send commands
  Send(analogValueBrake, analogValueThrottle);
}

// ########################## END ##########################
