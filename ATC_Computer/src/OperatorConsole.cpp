#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>
#include <sys/dispatch.h>
#include <unistd.h>
#include "Msg_structs.h"
#include "OperatorConsole.h"

static constexpr const char* COMPUTER_SYSTEM_CHANNEL = "computer_system_channel";

OperatorConsole::OperatorConsole(CommunicationsSystem& comms)
    : commsRef(comms) {}


OperatorConsole::~OperatorConsole() {
    std::cout << "OperatorConsole destroyed.\n";
}

void OperatorConsole::start() {
    std::cout << "Operator Console starting...\n";

    // Open connection to ComputerSystem (name_open)
    int coid = name_open(COMPUTER_SYSTEM_CHANNEL, 0);
    if (coid == -1) {
        std::cerr << "OperatorConsole: name_open failed for '" << COMPUTER_SYSTEM_CHANNEL
                  << "': " << strerror(errno) << "\n";
        return;
    }

    bool done = false;
    while (!done) {
        std::cout << "\nOperator Menu:\n"
                  << " 1) Change aircraft speed (send velocity vector)\n"
                  << " 2) Change aircraft position (teleport)\n"
                  << " 3) Change aircraft altitude (via heading struct altitude field)\n"
                  << " 4) Change collision-check frequency (ComputerSystem)\n"
                  << " 0) Exit\n"
                  << "Choose: ";
        int choice;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(1024, '\n');
            continue;
        }

        if (choice == 0) {
            done = true;
            break;
        }

        Message_inter_process msg {};
        msg.header = true; // inter-process
        msg.dataSize = 0;
        msg.planeID = -1;

        switch (choice) {
            case 1: {
                // Change speed / velocity: use msg_change_heading
                msg.type = MessageType::REQUEST_CHANGE_OF_HEADING;
                std::cout << "Enter Aircraft ID: \n";
                int id; std::cin >> id; msg.planeID = id;

                msg_change_heading ch {};
                std::cout << "Enter new VelocityX: \n"; std::cin >> ch.VelocityX;
                std::cout << "Enter new VelocityY: \n"; std::cin >> ch.VelocityY;
                std::cout << "Enter new VelocityZ: \n"; std::cin >> ch.VelocityZ;
                std::cout << "Enter altitude: \n"; std::cin >> ch.altitude;
                ch.ID = msg.planeID;

                if (sizeof(ch) <= msg.data.size()) {
                    std::memcpy(msg.data.data(), &ch, sizeof(ch));
                    msg.dataSize = sizeof(ch);
                } else {
                    std::cerr << "OperatorConsole: payload too large\n";
                    continue;
                }
                break;
            }
            case 2: {
                // Change position: use msg_change_position
                msg.type = MessageType::REQUEST_CHANGE_POSITION;
                std::cout << "Enter Aircraft ID: \n";
                int id; std::cin >> id; msg.planeID = id;

                msg_change_position cp {};
                std::cout << "Enter X: \n"; std::cin >> cp.x;
                std::cout << "Enter Y: \n"; std::cin >> cp.y;
                std::cout << "Enter Z: \n"; std::cin >> cp.z;

                if (sizeof(cp) <= msg.data.size()) {
                    std::memcpy(msg.data.data(), &cp, sizeof(cp));
                    msg.dataSize = sizeof(cp);
                } else {
                    std::cerr << "OperatorConsole: payload too large\n";
                    continue;
                }
                break;
            }
            case 3: {
                // Change altitude — reuse msg_change_heading.altitude
                msg.type = MessageType::REQUEST_CHANGE_ALTITUDE;
                std::cout << "Enter Aircraft ID: \n";
                int id; std::cin >> id; msg.planeID = id;

                msg_change_heading ch {};
                ch.ID = msg.planeID;
                ch.altitude = 0.0;
                std::cout << "Enter new altitude: \n";
                std::cin >> ch.altitude;

                if (sizeof(ch) <= msg.data.size()) {
                    std::memcpy(msg.data.data(), &ch, sizeof(ch));
                    msg.dataSize = sizeof(ch);
                } else {
                    std::cerr << "OperatorConsole: payload too large\n";
                    continue;
                }
                break;
            }
            case 4: {
                // Change collision-check frequency in ComputerSystem
                msg.type = MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS;
                std::cout << "Enter new collision time constraint (seconds): \n";
                int freq; std::cin >> freq;
                // We'll pack the integer into the data buffer
                if (sizeof(int) <= msg.data.size()) {
                    std::memcpy(msg.data.data(), &freq, sizeof(int));
                    msg.dataSize = sizeof(int);
                } else {
                    std::cerr << "OperatorConsole: payload too large\n";
                    continue;
                }
                break;
            }
            default:
                std::cout << "Unknown choice\n";
                continue;
        }

        // Send the message to ComputerSystem
        int rc = MsgSend(coid, &msg, sizeof(msg), nullptr, 0);
        if (rc == -1) {
            std::cerr << "OperatorConsole: MsgSend failed: " << strerror(errno) << "\n";
        } else {
            std::cout << "Command sent.\n";
        }

        // small pause to allow processing
        usleep(100000);
    }

    name_close(coid);
    std::cout << "Operator Console exiting.\n";
    return;
}

