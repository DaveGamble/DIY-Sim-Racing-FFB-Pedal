#include "Modbus.h"
#include "Main.h"


int32_t Modbus::computeCrc(uint8_t *buffer_pu8, int32_t bufferLength_i32, bool store) // We should use https://github.com/LacobusVentura/MODBUS-CRC16 if we can afford the space.
{
  // CRC-16-Modbus constants
  const int32_t g_crcPolynomial_i32 = 0xA001;
  const int32_t g_crcInitialValue_i32 = 0xFFFF;
  int32_t crc_i32 = g_crcInitialValue_i32; 
  for (uint8_t pos_u8 = 0; pos_u8 < bufferLength_i32; pos_u8++)
  {
    crc_i32 ^= (uint32_t)buffer_pu8[pos_u8];
    for (uint8_t i_u8 = 8; i_u8 != 0; i_u8--, crc_i32 >>= 1) if (crc_i32 & 1) crc_i32 ^= (g_crcPolynomial_i32 << 1);
  }
  if (!store) return crc_i32; 
  buffer_pu8[bufferLength_i32] = crc_i32 & 0xff;
  buffer_pu8[bufferLength_i32 + 1] = (crc_i32 >> 8) & 0xff;
  return crc_i32; 
}

void Modbus::transmit(uint8_t *buffer, int32_t len) const
{
  computeCrc(buffer, len);

  while(serial_pHS->available()) serial_pHS->read();
  serial_pHS->write(buffer, len + 2);
  serial_pHS->flush();
}

int32_t Modbus::sendRequestAndReceiveResponse(uint8_t slaveId_u8, uint8_t functionCode_u8, uint16_t registerAddress_u16, uint16_t numberOfRegisters_u16)
{
    uint8_t txBuffer_au8[9] = {slaveId_u8, functionCode_u8, (uint8_t)(registerAddress_u16 >> 8), (uint8_t)(registerAddress_u16 & 0xFF), (uint8_t)(numberOfRegisters_u16 >> 8), (uint8_t)(numberOfRegisters_u16 & 0xFF),0,0,0};
    transmit(txBuffer_au8, 6);

    const uint32_t endTime_u32 = millis() + timeout_u32;
    rawRxBufferLength_i32   = 0;
    int32_t echoMatchCount_i32 = 0;
    uint8_t receiveState_u8 = 0;

    bool allDataReceived_b = false;
    while (!allDataReceived_b && (millis() < endTime_u32))
    {
       delay(1);
       
       while(serial_pHS->available() && !allDataReceived_b)
       {
          const int32_t receivedByte_i32 = serial_pHS->read();

          switch (receiveState_u8)
          {
          case 0: // receive back echo of first two bytes
            if (txBuffer_au8[echoMatchCount_i32] != receivedByte_i32) echoMatchCount_i32 = 0;
            else echoMatchCount_i32++;
            if (echoMatchCount_i32 == 2) receiveState_u8 = 1; 
            break;
          case 1: // receive length and prepare
            rawRxBuffer_au8[0] = txBuffer_au8[0];
            rawRxBuffer_au8[1] = txBuffer_au8[1];
            rawRxBuffer_au8[2] = receivedByte_i32;
            rawRxBufferLength_i32 = 3;
            receiveState_u8 = 2;
            break;
          case 2: // receive data
            rawRxBuffer_au8[rawRxBufferLength_i32++] =  receivedByte_i32;
            if(rawRxBufferLength_i32 >= rawRxBuffer_au8[2] + 5) allDataReceived_b = true;
            break;
          }
       }
    }

    if(rawRxBufferLength_i32 <= 2) return -1;

    int32_t receivedCrc_i32 = ((uint16_t)rawRxBuffer_au8[rawRxBufferLength_i32 - 1] << 8) | rawRxBuffer_au8[rawRxBufferLength_i32 - 2];
    if(receivedCrc_i32 != computeCrc(rawRxBuffer_au8, rawRxBufferLength_i32 - 2, false)) return -1;

    return rawRxBuffer_au8[2];
}

int32_t Modbus::writeHoldingRegisterToDevice(uint8_t slaveId_u8, uint16_t registerAddress_u16, uint16_t value_u16) const
{
  uint8_t txBuffer_au8[9] = {slaveId_u8, WRITE_HOLDING_REGISTER_U8, (uint8_t)(registerAddress_u16 >> 8), (uint8_t)(registerAddress_u16 & 0xFF), (uint8_t)(value_u16 >> 8), (uint8_t)(value_u16 & 0xFF),0,0,0};
  transmit(txBuffer_au8, 6);

  uint32_t endTime_u32 = millis() + timeout_u32;
  int32_t echoMatchCount_i32 = 0;
  
  bool responseReceived_b = false;
  while (millis() < endTime_u32 && !responseReceived_b)
  {
      while(serial_pHS->available() && !responseReceived_b)
      {
        int32_t receivedByte_i32 = this->serial_pHS->read();
        if(txBuffer_au8[echoMatchCount_i32] != receivedByte_i32) echoMatchCount_i32 = 0;
        else echoMatchCount_i32++;
        if (echoMatchCount_i32 == 8) responseReceived_b = true;
      }
      if (!responseReceived_b) delay(1);
  }
  delay(5);
  return responseReceived_b;
}

int32_t Modbus::writeHoldingRegistersToDevice(uint8_t slaveId_u8, uint16_t registerAddress_u16, uint16_t* values_u16, uint8_t count_u8) const
{
    uint8_t localTxBuffer[32] = {slaveId_u8, 0x10, (uint8_t)(registerAddress_u16 >> 8), (uint8_t)(registerAddress_u16 & 0xFF), (uint8_t)(count_u8 >> 8), (uint8_t)(count_u8 & 0xFF), (uint8_t)(count_u8 * 2)}; // Max 10 registers supported    
    for (uint8_t i = 0; i < count_u8; i++) {
        localTxBuffer[7 + i*2] = values_u16[i] >> 8;
        localTxBuffer[8 + i*2] = values_u16[i] & 0xFF;
    }
    transmit(localTxBuffer, 7 + count_u8 * 2);

    // 3. Read exact 8-byte response and verify CRC
    const uint32_t endTime_u32 = millis() + timeout_u32;
    uint8_t rxBuffer[8];
    uint8_t rxCount = 0;
    
    bool responseReceived_b = false;
    while (millis() < endTime_u32 && !responseReceived_b)
    {
        while(serial_pHS->available() && !responseReceived_b)
        {
            rxBuffer[rxCount++] = serial_pHS->read();
            
            // Modbus FC16 Exception Response is exactly 5 bytes long (SlaveID, 0x90, ExceptionCode, CRC_L, CRC_H)
            if (rxCount == 5 && rxBuffer[1] == 0x90) break; // Exception detected, abort waiting for 8 bytes to avoid 100ms timeout penalty!
            
            if (rxCount == 8) {
                int32_t receivedCrc = ((uint16_t)rxBuffer[7] << 8) | rxBuffer[6];
                int32_t computedCrc = computeCrc(rxBuffer, 6, false);
                
                if (rxBuffer[0] == slaveId_u8 && rxBuffer[1] == 0x10 && receivedCrc == computedCrc) responseReceived_b = true;
            }
        }
        if (responseReceived_b || rxCount == 8 || (rxCount == 5 && rxBuffer[1] == 0x90)) break; 
        delay(1);
    }
    delay(5);
    return responseReceived_b;
}

bool Modbus::writeAndVerifyDeviceParameter(uint8_t slaveId_u8, uint16_t parameterAddress_u16, uint16_t value_u16)
{
  bool registerWritten_b = false;

  for (uint8_t tryIndex_u8 = 0; tryIndex_u8 < 10; tryIndex_u8++)
  {
    delay(10);

    if(sendRequestAndReceiveResponse(slaveId_u8, 0x03, parameterAddress_u16,  2) > 0 && convertRxBufferToInt16(0) == value_u16)
      return registerWritten_b; // success!

    delay(30);
    if (logEnabled_b)
    { ActiveSerial->print("Parameter adresse: "); ActiveSerial->print(parameterAddress_u16); ActiveSerial->print(",    actual: ");
      ActiveSerial->print(convertRxBufferToInt16(0)); ActiveSerial->print(",    target: "); ActiveSerial->println(value_u16);
    }

    writeHoldingRegisterToDevice(slaveId_u8, parameterAddress_u16, value_u16); 
    registerWritten_b = true;
  }

  return registerWritten_b;
}
