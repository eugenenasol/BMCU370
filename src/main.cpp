#include <Arduino.h>
#include "BMCU_Hardware.h"
#include "UART_Transport.h"
#include "MMU_Logic.h"
#include "CommandRouter.h"

// Instance Management (Global scope to persist)
static BMCU_Hardware* hal = nullptr;
static UART_Transport* transport = nullptr;
static UART_Transport* aux_transport = nullptr; // Auxiliary TTL serial on EXIT_TX/RX (USART3)
static MMU_Logic* logic = nullptr;
static CommandRouter* api = nullptr;

void setup() {
    // 1. Create HAL
    hal = new BMCU_Hardware();
    
    // 2. Create Primary Transport (USART1 / RS485)
    transport = new UART_Transport();
    transport->Init(UART_Transport::UARTPort::USART1_Main);
    
    // 3. Create Auxiliary Transport (USART3 / TTL on EXIT_TX/RX)
    aux_transport = new UART_Transport();
    aux_transport->Init(UART_Transport::UARTPort::USART3_Aux);
    
    // 4. Create Logic
    logic = new MMU_Logic(hal);
    logic->Init(); // Initializes HAL and Logic
    
    // 5. Init API/Router with both transports
    api = new CommandRouter();
    api->Init(logic, transport, aux_transport);
}

void loop() {
    // Run Logic
    if (logic) {
        logic->Run();
        if (hal) hal->WatchdogReset();
    }
    
    // Run API (polls both primary and auxiliary channels)
    if (api) api->Run();
}
