#pragma once

#include <stdint.h>
class MMU_Logic;
class I_MMU_Transport;

/*
* DEVELOPMENT STATE: TESTING
* This namespace handles the Klipper JSON protocol which is currently in a testing state.
*/
namespace KlipperCLI {
    
    /**
     * @brief Initialize the CLI with logic and transport dependencies.
     * @param mmu Pointer to MMU_Logic instance.
     * @param main_transport Pointer to primary transport layer.
     * @param aux_transport Pointer to auxiliary transport layer (optional).
     */
    void Init(MMU_Logic* mmu, I_MMU_Transport* main_transport, I_MMU_Transport* aux_transport = nullptr);

    // Main loop processor - handles incoming serial data
    void Run();

    // Check if connected
    bool IsConnected();
    
    // Check if serial has been idle for the specified duration
    bool IsSerialIdle(uint32_t idle_ms);
}
