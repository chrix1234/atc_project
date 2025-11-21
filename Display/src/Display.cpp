#include <sys/dispatch.h>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <algorithm>
#include <cmath>
#include "Msg_structs.h"  // Your shared structs (SharedMemory, msg_plane_info, Message_inter_process)

#define DISPLAY_CHANNEL "chris_display"
#define COLLISION_CHANNEL "chris_collision"
#define SHM_NAME "/radar_shm"
#define SHARED_MEMORY_SIZE sizeof(SharedMemory)

// Grid size
#define GRID_W 40
#define GRID_H 20

// Airspace limits
#define MAX_X 100000.0
#define MAX_Y 100000.0
#define MAX_Z 25000.0

SharedMemory* shared_mem = nullptr;
int clamp(int val, int minVal, int maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}
struct PlanePair {
    int plane1;
    int plane2;
};
// Display planes in text grid
void drawGrid() {
    std::vector<std::vector<std::string>> grid(GRID_H, std::vector<std::string>(GRID_W, " ."));

    for (int i = 0; i < shared_mem->count; ++i) {
        auto& p = shared_mem->plane_data[i];

        // Map world coordinates to grid
        int gx = static_cast<int>(p.PositionX / MAX_X * GRID_W);
        int gy = static_cast<int>(p.PositionY / MAX_Y * GRID_H);

        // Clamp
        if (gx < 0) gx = 0;
        if (gx >= GRID_W) gx = GRID_W - 1;
        if (gy < 0) gy = 0;
        if (gy >= GRID_H) gy = GRID_H - 1;

        // Place plane ID in grid (convert int to string)
        if (grid[GRID_H - 1 - gy][gx] != " .") {
            grid[GRID_H - 1 - gy][gx] = "*"; // mark collision
        } else {
            grid[GRID_H - 1 - gy][gx] = std::to_string(p.id);
        }

    }

    // Print the grid
    for (auto& row : grid) {
        for (auto& cell : row) {
            std::cout << cell << " ";
        }
        std::cout << "\n";
    }
}



// Thread to listen for collision warnings
void listenForCollisions() {
    name_attach_t* attach = name_attach(nullptr, DISPLAY_CHANNEL, 0);
    if (!attach) {
        std::cerr << "Display: name_attach failed\n";
        return;
    }

    Message_inter_process msg;
    while (true) {
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), nullptr);
        if (rcvid < 0) continue;

        if (msg.type == MessageType::COLLISION_DETECTED) {
            std::cout << "\n*** COLLISION WARNING ***\n";
            int numPairs = msg.dataSize / sizeof(std::pair<int,int>);
            auto pairs = reinterpret_cast<std::pair<int,int>*>(msg.data.data());
            for (int i = 0; i < numPairs; i++) {
                std::cout << "Planes " << pairs[i].first
                          << " and " << pairs[i].second
                          << " predicted to collide.\n";
            }
            std::cout << "*************************\n";
        }

        MsgReply(rcvid, EOK, nullptr, 0);
    }

    name_detach(attach, 0);
}

void collisionIPCServer() {
    name_attach_t* attach = name_attach(nullptr, COLLISION_CHANNEL, 0);
    if (!attach) {
        std::cerr << "IPC Server: name_attach failed\n";
        return;
    }

    Message_inter_process msg;
    while (true) {
        int rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), nullptr);
        if (rcvid < 0) continue;

        MsgReply(rcvid, EOK, nullptr, 0);
    }

    name_detach(attach, 0);
}

void sendCollisionAlert(int planeA, int planeB) {
    int coid = name_open(COLLISION_CHANNEL, 0);
    if (coid == -1) {
        std::cerr << "Display: Failed to connect to collision channel\n";
        return;
    }

    PlanePair pair{planeA, planeB};
    Message_inter_process msg;
    msg.type = MessageType::COLLISION_DETECTED;
    msg.dataSize = sizeof(pair);
    memcpy(msg.data.data(), &pair, sizeof(pair));

    MsgSend(coid, &msg, sizeof(msg), nullptr, 0);

    close(coid);
}

void checkAndNotifyCollisions() {
    for (int i = 0; i < shared_mem->count; ++i) {
        auto& p1 = shared_mem->plane_data[i];
        for (int j = i + 1; j < shared_mem->count; ++j) {
            auto& p2 = shared_mem->plane_data[j];

            // Simple proximity check (tweak thresholds as needed)
            if (std::abs(p1.PositionX - p2.PositionX) < 1000 &&
                std::abs(p1.PositionY - p2.PositionY) < 1000 &&
                std::abs(p1.PositionZ - p2.PositionZ) < 500) {

                // Print to console
                std::cout << "\n*** COLLISION ALERT! ***\n";
                std::cout << "Planes " << p1.id << " and " << p2.id
                          << " \nCOLLISION!\n";
                std::cout << "************************\n";

                // Send IPC message
                sendCollisionAlert(p1.id, p2.id);
            }
        }
    }
}

// Thread to periodically read shared memory and draw the grid
void readAndDisplay() {
    while (true) {
        if (shared_mem->count == 0) {
            std::cout << "No planes in airspace.";
        } else {
            drawGrid();

            std::cout << "Planes info:\n";
            for (int i = 0; i < shared_mem->count; i++) {
                auto& p = shared_mem->plane_data[i];
                std::cout << "Plane " << p.id
                          << " Pos(" << p.PositionX << "," << p.PositionY << "," << p.PositionZ << ")"
                          << " Vel(" << p.VelocityX << "," << p.VelocityY << "," << p.VelocityZ << ")\n";
            }
            checkAndNotifyCollisions();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    // Open shared memory
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        std::cerr << "Display: shm_open failed\n";
        return 1;
    }

    shared_mem = (SharedMemory*)mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_mem == MAP_FAILED) {
        std::cerr << "Display: mmap failed\n";
        return 1;
    }

    std::cout << "Display: Shared memory mapped successfully.\n";

    // Start threads
    std::thread t1(readAndDisplay);
    std::thread t2(listenForCollisions);
    std::thread t3(checkAndNotifyCollisions);

    t1.join();
    t2.join();
    t3.join();

    return 0;
}
