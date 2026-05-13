#include "CommandRouter.h"
#include "KlipperCLI.h"
#include "../interfaces/I_MMU_Transport.h"
#include "../core/MMU_Logic.h"

CommandRouter::CommandRouter() : _mmu(nullptr), _transport(nullptr), _aux_transport(nullptr) {
}

void CommandRouter::Init(MMU_Logic* mmu, I_MMU_Transport* transport, I_MMU_Transport* aux_transport) {
    _mmu = mmu;
    _transport = transport;
    _aux_transport = aux_transport;
    
    // Initialize KlipperCLI with primary and auxiliary transports
    KlipperCLI::Init(mmu, transport, aux_transport);
}

void CommandRouter::Run() {
    KlipperCLI::Run();
}
