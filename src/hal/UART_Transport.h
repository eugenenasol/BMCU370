#pragma once

#include "../interfaces/I_MMU_Transport.h"
#include "Hardware.h"

/**
 * @file UART_Transport.h
 * @brief UART implementation of I_MMU_Transport
 * 
 * Reference implementation for serial communication.
 * Wraps the Hardware::UART_* functions.
 */
class UART_Transport : public I_MMU_Transport {
public:
    enum class UARTPort {
        USART1_Main,
        USART3_Aux
    };

    UART_Transport() {}
    virtual ~UART_Transport() {}

    void Init() override;
    void Init(UARTPort port);
    
    uint16_t Available() override;
    int Read() override;
    uint16_t ReadBytes(uint8_t* buffer, uint16_t len) override;
    uint16_t Write(const uint8_t* data, uint16_t len) override;
    void Flush() override;
    
    bool IsConnected() override;
    bool IsBusy() override;
    uint32_t GetDroppedBytes() const { return dropped_bytes; }

    // Internal: Called by RX interrupt to buffer incoming bytes
    void OnByteReceived(uint8_t byte);

private:
    UARTPort _port = UARTPort::USART1_Main;

    // Ring buffer for received bytes (reverted to 1024 for RAM safety)
    static constexpr uint16_t RX_BUFFER_SIZE = 1024;
    volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
    volatile uint16_t rx_head = 0;
    volatile uint32_t dropped_bytes = 0;   // Counter of bytes lost to ring buffer overflow
    volatile uint16_t rx_tail = 0;
};
