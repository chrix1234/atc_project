#include "CommunicationsSystem.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h> // for usleep
#include <sys/dispatch.h>

void CommunicationsSystem::start() {
    // Start the communications thread if not already running
    if (!Communications_System.joinable()) {
        Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
    }
}

// Constructor
CommunicationsSystem::CommunicationsSystem() {
    std::cout << "CommunicationsSystem initialized.\n";

    // Start the communications handling thread
    Communications_System = std::thread(&CommunicationsSystem::HandleCommunications, this);
}

// Destructor
CommunicationsSystem::~CommunicationsSystem() {
    std::cout << "CommunicationsSystem shutting down.\n";

    // Join the thread if running
    if (Communications_System.joinable()) {
        Communications_System.join();
    }
}

// Thread function to handle incoming/outgoing messages
void CommunicationsSystem::HandleCommunications() {
    std::cout << "CommunicationsSystem thread running.\n";
    name_attach_t* attach = name_attach(nullptr, "communications_channel", 0);
    if (!attach) {
        std::cerr << "CommunicationsSystem: name_attach failed: " << strerror(errno) << "\n";
        return;
    }
    std::cout << "CommunicationsSystem: channel attached as 'communications_channel'.\n";

    bool running = true;
    while (running) {
        Message_inter_process incoming;
        int rcvid = MsgReceive(attach->chid, &incoming, sizeof(incoming), nullptr);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            std::cerr << "CommunicationsSystem: MsgReceive failed: " << strerror(errno) << "\n";
            break;
        }

        // Handle incoming message
        messageAircraft(reinterpret_cast<const Message&>(incoming));

        // Reply if needed
        MsgReply(rcvid, EOK, nullptr, 0);
    }

    name_detach(attach, 0);
    std::cout << "CommunicationsSystem thread exiting.\n";
}

// Send a message to an aircraft (placeholder)
void CommunicationsSystem::messageAircraft(const Message& msg) {
    // Implement message sending logic here
    std::cout << "Sending message to aircraft ID " << msg.planeID << "\n";
}
