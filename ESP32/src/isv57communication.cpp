#include "isv57communication.h"
#include "Main.h"
#include "isv57_tunedParameters.h"


static void printDecodedAlarmString(uint16_t alarm_code) 
{
  switch (alarm_code & 0x0FFF) { // Mask to get lower 12 bits
    case 0x000: ActiveSerial->println("Normal\n"); break;
    case 0x0E1:
    case 0x0E0: ActiveSerial->println("Overcurrent\n"); break;
    case 0x100: ActiveSerial->println("Overload\n"); break;
    case 0x180: ActiveSerial->println("Excessive position deviation\n"); break;
    case 0x1A0: ActiveSerial->println("Overspeed\n"); break;
    case 0x1A1: ActiveSerial->println("Motor out of control\n"); break;
    case 0x0D0: ActiveSerial->println("Undervoltage\n"); break;
    case 0x0C0: ActiveSerial->println("Overvoltage\n"); break;
    case 0x171:
    case 0x172: ActiveSerial->println("Encoder parameter error\n"); break;
    case 0x190: ActiveSerial->println("Excessive motor vibration\n"); break;
    case 0x150: ActiveSerial->println("Encoder disconnected\n"); break;
    case 0x151:
    case 0x170: ActiveSerial->println("Encoder data error\n"); break;
    case 0x152: ActiveSerial->println("Encoder HALL signal error\n"); break;
    case 0x240: ActiveSerial->println("Parameter saving error\n"); break;
    case 0x570: ActiveSerial->println("Emergency stop\n"); break;
    case 0x120: ActiveSerial->println("Regenerative energy overload\n"); break;
    case 0x153: ActiveSerial->println("Encoder battery error\n"); break;
    case 0x210:
    case 0x211:
    case 0x212: ActiveSerial->println("Input configuration error (Repeated/wrong input)\n"); break;
    default: ActiveSerial->println("Unknown or refer to Chapter 9\n"); break;
  }
}




// initialize the communication
Isv57Communication::Isv57Communication() : Modbus(Serial2)
{
  #if PCB_VERSION == 10 || PCB_VERSION == 9 || PCB_VERSION == 12 || PCB_VERSION == 13 || PCB_VERSION == 14
    Serial2.begin(38400, SERIAL_8N1, ISV57_RXPIN, ISV57_TXPIN, false); // Modbus serial
  #else
    Serial2.begin(38400, SERIAL_8N1, ISV57_RXPIN, ISV57_TXPIN, true); // Modbus serial
  #endif
  setLogging(true);
}




// send tuned servo parameters
void Isv57Communication::setupServoStateReading() {

  // The iSV57 has four registers (0x0191, 0x0192, 0x0193, 0x0194) in which we can write, which values we want to obtain cyclicly
  // These registers can be obtained by sending e.g. the command: 0x63, 0x03, 0x0191, target_sate, CRC
  // tell the modbus slave, which registers will be read cyclicly
  writeAndVerifyDeviceParameter(slaveId, 0x0191, reg_add_position_given_p);
  writeAndVerifyDeviceParameter(slaveId, 0x0192, reg_add_velocity_current_feedback_percent);
  writeAndVerifyDeviceParameter(slaveId, 0x0193, reg_add_position_error_p);
  writeAndVerifyDeviceParameter(slaveId, 0x0194, reg_add_voltage_0p1V);
}


// Disable aixs command
void Isv57Communication::disableAxis()
{
  ActiveSerial->println("Disabling servo");

  // 0x3f, 0x06, 0x00, 0x85, 0x03, 0x03, 0xdc, 0x0c
  //modbus.writeAndVerifyDeviceParameter(slaveId, 0x0085, 0x0303);
  writeHoldingRegisterToDevice(slaveId, 0x0085, 0x0303);
  // 0x3f, 0x06, 0x01, 0x39, 0x00, 0x00, 0x5c, 0xe5
  //modbus.writeAndVerifyDeviceParameter(slaveId, 0x0139, 0x0000); 
  writeHoldingRegisterToDevice(slaveId, 0x0139, 0x0008);
  delay(30);

  // read routine
  readHoldingRegisterFromDevice(0x0085);
  readHoldingRegisterFromDevice(0x0139);
  delay(5);
}

void Isv57Communication::enableAxis()
{
  ActiveSerial->println("Enabling servo");

  // 0x3f, 0x06, 0x00, 0x85, 0x03, 0x83, 0xdd, 0xac
  // Pr4.08: 0x085
  writeHoldingRegisterToDevice(slaveId, 0x0085, 0x0383);
  // 0x3f, 0x06, 0x01, 0x39, 0x00, 0x08, 0x5d, 0x23
  writeHoldingRegisterToDevice(slaveId, 0x0139, 0x0008);
  delay(30);

  // read routine
  readHoldingRegisterFromDevice(0x0085);
  readHoldingRegisterFromDevice(0x0139);
  delay(5);
}

void  Isv57Communication::clearServoUnitPosition()
{
	// According to Leadshines User Manual of 2ELD2-RD DC Servo
	// https://www.leadshine.com/upfiles/downloads/a3d7d12a120fd8e114f6288b6235ac1a_1690179981835.pdf
	// Changing the position unit, will clear the position data

  writeAndVerifyDeviceParameter(slaveId, pr_5_00+20, 0); // encoder output resolution  {0: Encoder units; 1: Command units; 2: 10000pulse/rotation}
  delay(100);
	writeAndVerifyDeviceParameter(slaveId, pr_5_00+20, 1); // encoder output resolution  {0: Encoder units; 1: Command units; 2: 10000pulse/rotation}
  delay(100);
}



// send tuned servo parameters
void Isv57Communication::sendTunedServoParameters(bool commandRotationDirection, uint32_t stepsPerMotorRev_u32) {
  
  bool retValue_b = false;

  
// #define ADAPTIVE_SERVO_PARAMS
// #ifdef ADAPTIVE_SERVO_PARAMS
//   // see https://atbautomation.eu/uploads/User_Manual_Leadshine_iSV2-RS.pdf, p.22, Pr0.00
//   // 1) Pr0.01 = 0 --> position mode
//   // 2) Pr0.02 = 1 --> interpolation mode
//   // 3) Pr0.04 inertia ratio
//   // 4) Pr0.03 machine stiffness
//   // 5) Pr0.00 = 1 --> adaptive bandwidth
//   retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+0, 1); // adaptive bandwidth modell following controll
//   retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+2, 1); // positioning mode with auto tuning
//   retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+3, 9); // machine stiffness
//   retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+4, 1); // inertia
//   retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_2_00+0, 2); // adaptive filter on all the time
// #endif


  // Pr0 register
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+0, tuned_parameters[pr_0_00+0]); // control mode

  // according to the iSV2 manual chapter 5.6, the model following control (MFC) parameter should be larger then Pr1.01, velocity loop gain
  // float mfcLowerLimit_fl32 = tuned_parameters[pr_1_00+1] ;
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+1, tuned_parameters[pr_0_00+1]); // control mode #
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+6, tuned_parameters[pr_0_00+6]); // motor command direction
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+8, (long)stepsPerMotorRev_u32); // microsteps
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+9, tuned_parameters[pr_0_00+9]); // 1st numerator 
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+10, tuned_parameters[pr_0_00+10]); // & denominator
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_0_00+14, tuned_parameters[pr_0_00+14]); // position deviation setup

  // Pr1 register
  //uint16_t special_function_flags = 0x4 | 0x8 | 0x10 | 0x40 | 0x400;
  uint16_t special_function_flags = 0x4 | 0x8 | 0x10 | 0x20| 0x400;
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_1_00+37, tuned_parameters[pr_1_00+37]); // special function register
  // see https://www.oyostepper.com/images/upload/File/ISV57T-180.pdf
  // 0x01: =0: Enablespeedfeed-forwardfiltering; =1:Disablespeed feed-forward filtering
  // 0x02: =0: Enabletorquefeed-forwardfiltering; =2:disabletorque feed-forward filtering
  // 0x04: =0: Enablemotor stall Er1A1 alarm; =4:Blockmotor stall Er1A1 alarm
  // 0x08: =0: Enable overshoot Er180 alarm; =8:Mask overshoot Er180alarm
  // 0x10: =0: Enable overload Er100 alarm; =0x10: Mask overload Er100alarm
  // 0x20: =0: dial input function not assignable; =0x20: dial input function assignable
  // 0x40: =0: Mask drive disable Er260 alarm; =0x40: Enable drive disable Er260 alarm
  // 0x400: =0: Mask undervoltage Er0D0 alarm; =0x400: Enable undervoltage Er0D0 alarm
  
  // Pr4 register
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_4_00+10, tuned_parameters[pr_4_00+10]); // Alarm port signal

  // Pr5 register
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_5_00+20, tuned_parameters[pr_5_00+20]); // encoder output resolution  {0: Encoder units; 1: Command units; 2: 10000pulse/rotation}
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_5_00+35, 1); // lock front panel
  
  // Pr7 register
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_7_00+0, tuned_parameters[pr_7_00+0]); // current loop gain
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_7_00+1, tuned_parameters[pr_7_00+1]); // current loop integral time

  // Enable & tune reactive pumping. This will act like a braking resistor and reduce EMF voltage.
  // See https://en.wikipedia.org/wiki/Bleeder_resistor
  // Info from iSV2 manual: The external resistance is activated when the actual bus voltage is higher than Pr7.32 plus Pr7.33 and is deactivated when the actual bus voltage is lower than Pr7.32 minus Pr7.33
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_7_00+31, tuned_parameters[pr_7_00+31]); // bleeder control mode; 0: is default and seems to enable braking mode, contrary to manual
  retValue_b |= setServoVoltage(SERVO_MAX_VOLTAGE_IN_V_36V);
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_7_00+33, tuned_parameters[pr_7_00+33]); // bleeder hysteresis voltage; Contrary to the manual this seems to be an offset voltage, thus Braking disabling voltage = Pr7.32 + Pr.33

  // disable axis by default
  retValue_b |= writeAndVerifyDeviceParameter(slaveId, pr_4_00+8, tuned_parameters[pr_4_00+8]);
    
  if (!retValue_b) return;
  // store the settings to servos NVM if necesssary
  disableAxis();// disable axis a second time, since the second signal must be send to. Don't know yet the meaning of that signal.
  ActiveSerial->println("Servo registered in NVM have been updated! Please power cycle the servo and the ESP!");    
  writeHoldingRegisterToDevice(slaveId, 0x019A, 0x5555); // store the settings to servos NVM // identified with logic analyzer. See \StepperParameterization\Meesages\StoreSettingsToEEPROM_0.png
  // ToDo: according to iSV57 manual, 0x2211 is the command to write values to EEPROM
  delay(500);
  // ToDo: soft reset servo. The iSV57 docu says Pr0.25: 0x6666 is soft reset
  // writeHoldingRegister(slaveId, 0x019A, 0x6666); // store the settings to servos NVM
  delay(1000);
}

bool Isv57Communication::findServosSlaveId()
{
  // typically the servo address is 63, so start with that
  for (int slaveIdTest = 63; slaveIdTest<256 + 63; slaveIdTest++)
  {
      if(sendRequestAndReceiveResponse(slaveIdTest & 255, 0x03, 0x0000, 2) <= 0) {delay(5); continue;}

      slaveId = (uint8_t)(slaveIdTest & 255);
      ActiveSerial->print("Found servo slave ID:"); ActiveSerial->print(slaveId); ActiveSerial->print("\r\n");
      return true;
  }
  return false;
}


// read servo states
void Isv57Communication::readServoStates() {

  // read the four registers simultaneously
  int bytesReceived_i = sendRequestAndReceiveResponse(slaveId, 0x03, ref_cyclic_read_0, NUMBER_OF_ISV57_REGISTERS_TO_READ_IN_CYCLIC_READ);

  isv57dynamicStates_.servo_receivedPacketIsValid_b = false;

  if(bytesReceived_i != (NUMBER_OF_ISV57_REGISTERS_TO_READ_IN_CYCLIC_READ*2)) return;

  // write to public variables
  isv57dynamicStates_.servo_pos_given_p = convertRxBufferToInt16(0);
  isv57dynamicStates_.servo_current_percent = convertRxBufferToInt16(1);
  isv57dynamicStates_.servo_pos_error_p = convertRxBufferToInt16(2);
  isv57dynamicStates_.servoVoltage0p1V_i16 = convertRxBufferToInt16(3);

  isv57dynamicStates_.lastUpdateTimeInMS_u32 = millis();
  isv57dynamicStates_.servo_cycleCounter_u32++;
  isv57dynamicStates_.servo_receivedPacketIsValid_b = (isv57dynamicStates_.servoVoltage0p1V_i16 >= 50); // check if signals are in valid ranges
}


bool Isv57Communication::readAlarmHistory() {

  bool alarmWasFound_b = false;
	ActiveSerial->print("\niSV57 alarm history: ");
	for (uint8_t idx=0; idx < 12; idx++)
	{
	  // example signal, read the 9th alarm
	  // 0x3f, 0x03, 0x12, 0x09, 0x00, 0x01, 0x55, 0xAE

	  // read the four registers simultaneously
	  int bytesReceived_i = sendRequestAndReceiveResponse(slaveId, 0x03, 0x1200 + idx, 1);
    
	  if(bytesReceived_i != 2) continue;

    for (uint8_t regIdx = 0; regIdx < 1; regIdx++)
    { 
      uint16_t alarm_code = convertRxBufferToInt16(regIdx) & 0x0FFF; // mask the first half byte as it does not contain info

      if (alarm_code > 0)
      {
        ActiveSerial->print("Alarm Idx: "); ActiveSerial->print(idx); ActiveSerial->print(",    Alarm Code: ");
        ActiveSerial->print( alarm_code, HEX); ActiveSerial->print(" --> "); printDecodedAlarmString(alarm_code);
        alarmWasFound_b = true;
      }
      
    }
	}

  // In case of no alarm --> indicate with string
  if (false == alarmWasFound_b) ActiveSerial->print("No alarm was found.");
	ActiveSerial->print("\n");    
	return 1;
}


void Isv57Communication::resetToFactoryParams() 
{
  // Identified with Free Device Monitoring Studio: https://hhdsoftware.com/device-monitoring-studio
  // Data view
  // Write:  3F 03 01 F0 00 01 81 1B
  // Read: 3F 03 02 00 00 91 81

  // Write:  3F 06 01 9A 44 44 9F F4
  // Read:  3F 06 01 9A 44 44 9F F4

  // Write:  3F 03 01 F7 00 01 30 DA
  // Read:  3F 03 02 55 55 6E EE

  // disable axis first
  disableAxis();
  ActiveSerial->println("Disabling axis first\n");
  delay(1000);
  // identified with logic analyzer. See \StepperParameterization\Meesages\ResetToFactorySettings_0.png
  if (readHoldingRegisterFromDevice(0x01F0) != 0x00) return;
  ActiveSerial->println("First test passed\n");
  writeHoldingRegisterToDevice(slaveId, 0x019a, 0x4444);
  if (readHoldingRegisterFromDevice(0x01F7) != 0x5555) return;
  ActiveSerial->println("Reset to factory settings successfull\n");  
}
