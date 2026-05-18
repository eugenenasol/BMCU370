/**
 * @file UART_Transport.cpp
 * @brief UART implementation of I_MMU_Transport
 */
#include "UART_Transport.h"

// Global instance pointers for interrupt callbacks
static UART_Transport* g_transport_main = nullptr;
static UART_Transport* g_transport_aux = nullptr;

// Callback for main USART1 RX interrupt
static void uart_main_rx_callback(uint8_t byte) {
    if (g_transport_main) {
        g_transport_main->OnByteReceived(byte);
    }
}

// Callback for auxiliary USART3 RX interrupt
static void uart_aux_rx_callback(uint8_t byte) {
    if (g_transport_aux) {
        g_transport_aux->OnByteReceived(byte);
    }
}

void UART_Transport::Init() {
    Init(UARTPort::USART1_Main);
}

void UART_Transport::Init(UARTPort port) {
    _port = port;
    if (_port == UARTPort::USART1_Main) {
        g_transport_main = this;
        Hardware::InitUART(true); // Klipper mode UART
        Hardware::UART_SetRxCallback(uart_main_rx_callback);
    } else {
        g_transport_aux = this;
        Hardware::InitUSART3(); // Auxiliary interface
        Hardware::USART3_SetRxCallback(uart_aux_rx_callback);
    }
}

uint16_t UART_Transport::Available() {
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;
    if (head >= tail) {
        return head - tail;
    } else {
        return RX_BUFFER_SIZE - tail + head;
    }
}

int UART_Transport::Read() {
    if (rx_head == rx_tail) {
        return -1; // No data
    }
    uint8_t byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return byte;
}

uint16_t UART_Transport::ReadBytes(uint8_t* buffer, uint16_t len) {
    uint16_t count = 0;
    while (count < len && rx_head != rx_tail) {
        buffer[count++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }
    return count;
}

uint16_t UART_Transport::Write(const uint8_t* data, uint16_t len) {
    if (_port == UARTPort::USART1_Main) {
        Hardware::UART_Send(data, len);
    } else {
        Hardware::USART3_Send(data, len);
    }
    return len;
}

void UART_Transport::Flush() {
    // Hardware UART is synchronous, nothing to flush
}

bool UART_Transport::IsConnected() {
    // UART is always "connected" when initialized
    return true;
}

bool UART_Transport::IsBusy() {
    if (_port == UARTPort::USART1_Main) {
        return Hardware::UART_IsBusy();
    } else {
        return Hardware::USART3_IsBusy();
    }
}

void UART_Transport::OnByteReceived(uint8_t byte) {
    uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
    if (next_head != rx_tail) {
        rx_buffer[rx_head] = byte;
        rx_head = next_head;
    } else {
        dropped_bytes++;   // Counter of bytes lost to ring buffer overflow
    }
}
