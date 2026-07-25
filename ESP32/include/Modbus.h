#ifndef MODBUS_H
#define MODBUS_H

#include <Arduino.h>

class Modbus
{
private:
    enum
    {
        HOLDING_REGISTER_U8 = 3,
        WRITE_HOLDING_REGISTER_U8 = 6
    };
    bool logEnabled_b = false;
    uint32_t timeout_u32 = 100;
    HardwareSerial* serial_pHS {};
    uint8_t rawRxBuffer_au8[512], slaveId_u8 = 0x01;
    int32_t rawRxBufferLength_i32 = 0;

    static int32_t computeCrc(uint8_t *buffer_pu8, int32_t bufferLength_i32, bool store = true);
    void transmit(uint8_t *buffer, int32_t len) const;
public:   
    Modbus(HardwareSerial &_serial_pHS) : serial_pHS(&_serial_pHS) {}
    void setLogging(bool enableLogging_b = false) { logEnabled_b = enableLogging_b; }

    // Main fn
    int32_t sendRequestAndReceiveResponse(uint8_t slaveId_u8, uint8_t functionCode_u8, uint16_t registerAddress_u16, uint16_t numberOfRegisters_u16);
    int16_t convertRxBufferToInt16(int32_t index_i32) const { return (int16_t)((uint16_t)rawRxBuffer_au8[(index_i32 * 2) + 3] << 8 | rawRxBuffer_au8[(index_i32 * 2) + 4]); }

    // Holding regs
    int32_t readHoldingRegisterFromDevice(uint16_t registerAddress_u16)
    { return sendRequestAndReceiveResponse(slaveId_u8, HOLDING_REGISTER_U8, registerAddress_u16, 1) ? convertRxBufferToInt16(0) : -1; }
    int32_t writeHoldingRegisterToDevice(uint8_t slaveId_u8, uint16_t registerAddress_u16, uint16_t value_u16) const;
    int32_t writeHoldingRegistersToDevice(uint8_t slaveId_u8, uint16_t registerAddress_u16, uint16_t* values_u16, uint8_t count_u8) const;

    // Device params
    bool writeAndVerifyDeviceParameter(uint8_t slaveId_u8, uint16_t parameterAddress_i16, uint16_t value_u16);
};

#endif
