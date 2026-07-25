#ifndef MODBUS_H
#define MODBUS_H

#include <Arduino.h>
#include <Stream.h>

#define COIL_REGISTER_U8            0x01
#define DISCRET_REGISTER_U8         0x02
#define HOLDING_REGISTER_U8         0x03
#define INPUT_REGISTER_U8           0x04
#define WRITE_HOLDING_REGISTER_U8   0x06

class Modbus
{
private:
    /* data */
    bool logEnabled_b = false;
    uint32_t timeout_u32 = 100;
    HardwareSerial* serial_pHS {};
    uint8_t rawRxBuffer_au8[512];
    int32_t rawRxBufferLength_i32 = 0;
    uint8_t dataRxBuffer_au8[512];
    int32_t dataRxBufferLength_i32 = 0;
    int32_t slaveId_i32 = 0x01;
    uint8_t txBuffer_au8[9] = {0,0,0,0,0,0,0,0,0};

    int32_t computeCrc(uint8_t *buffer_pu8, int32_t bufferLength_i32);

    uint8_t readByteFromRxBuffer(int32_t index_i32) const { return rawRxBuffer_au8[index_i32 + 3]; }
    int32_t readBlockFromRxBuffer(int32_t index_i32) { return  (((uint16_t)dataRxBuffer_au8[index_i32 * 2] << 8) | dataRxBuffer_au8[index_i32 * 2 + 1]); }

public:
    
    Modbus(HardwareSerial &_serial_pHS) : serial_pHS(&_serial_pHS) {}
    
    void setLogging(bool enableLogging_b = false) { logEnabled_b = enableLogging_b; }

    int32_t readHoldingRegisterFromDevice(int32_t registerAddress_i32) { return readHoldingRegisterFromDevice(slaveId_i32, registerAddress_i32, 1); }
    int32_t readHoldingRegisterFromDevice(int32_t slaveId_i32, int32_t registerAddress_i32, int32_t block_i32);
    
    int32_t writeHoldingRegisterToDevice(int32_t slaveId_i32, int32_t registerAddress_i32, uint16_t value_u16);
    int32_t writeHoldingRegistersToDevice(int32_t slaveId_i32, int32_t registerAddress_i32, uint16_t* values_u16, uint8_t count_u8);
    void getRawRxBuffer(uint8_t *rawBuffer_pu8, uint8_t &rawBufferLength_u8);

    int32_t sendRequestAndReceiveResponse(int32_t slaveId_i32, int32_t functionCode_i32, int32_t registerAddress_i32, int32_t numberOfRegisters_i32);

    bool writeAndVerifyDeviceParameter(uint16_t slaveId_u16, int16_t parameterAddress_i16, int32_t value_i32);
    void readDeviceParameter(uint16_t slaveId_u16, uint16_t parameterAddress_u16);



    int16_t convertRxBufferToInt16(int32_t index_i32) { return (int16_t)((uint16_t)rawRxBuffer_au8[(index_i32 * 2) + 3] << 8 | rawRxBuffer_au8[(index_i32 * 2) + 4]); }
};

#endif
