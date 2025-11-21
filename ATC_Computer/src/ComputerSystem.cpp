#include "ComputerSystem.h"
#include "ATCTimer.h"
#include <ctime>        // For std::time_t, std::localtime
#include <iomanip>      // For std::put_time
#include <cmath>
#include <sys/dispatch.h>
#include <cstring> // For memcpy
#include <errno.h>

// COEN320 Task 3.1, set the display channel name
#define display_channel_name "chris_display"
static constexpr const char* COMPUTER_SYSTEM_CHANNEL = "computer_system_channel";
static constexpr const char* COMMUNICATIONS_CHANNEL = "communications_channel";
#define COLLISION_CHANNEL "chris_collision"

ComputerSystem::ComputerSystem() : shm_fd(-1), shared_mem(nullptr), running(false) {}

ComputerSystem::~ComputerSystem() {
    joinThread();
    cleanupSharedMemory();
}

bool ComputerSystem::initializeSharedMemory() {
	const char *name = "/radar_shm";

	// Open the shared memory object
	while (true) {
        // COEN320 Task 3.2
		// Attempt to open the shared memory object (You need to use the same name as Task 2 in Radar)
        // In case of error, retry until successful
        // e.g. shm_open("/radar_shared_mem", O_RDONLY, 0666);

		// Open shared memory
	    this->shm_fd = shm_open(name, O_RDWR, 0666);
	    if (this->shm_fd == -1) {
	        fprintf(stderr, "shm_open (write) failed: %s\n", strerror(errno));
	        return false;
	    }
        // COEN320 Task 3.3
		// Map the shared memory object into the process's address space
        // The shared memory should be mapped to "shared_mem" (check for errors)
	    void *ptr = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, this->shm_fd, 0);
	    if (ptr == MAP_FAILED) {
	        fprintf(stderr, "mmap (write) failed: %s\n", strerror(errno));
	        close(this->shm_fd);
	        return false;
	    }
	    this->shared_mem = static_cast<SharedMemory*>(ptr);
        std::cout << "Shared memory initialized successfully." << std::endl;
        return true;
	}
}

void ComputerSystem::cleanupSharedMemory() {
    if (shared_mem && shared_mem != MAP_FAILED) {
        munmap(shared_mem, sizeof(SharedMemory));
    }
    if (shm_fd != -1) {
        close(shm_fd);
    }
}

bool ComputerSystem::startMonitoring() {
    if (initializeSharedMemory()) {
        running.store(true); // will be used in monitorAirspace and operator thread
        std::cout << "Starting monitoring thread." << std::endl;
        monitorThread = std::thread(&ComputerSystem::monitorAirspace, this);

        // start operator input listener thread
        monitorOperatorInput = std::thread(&ComputerSystem::processMessage, this);
        return true;
    } else {
        std::cerr << "Failed to initialize shared memory. Monitoring not started.\n";
        return false;
    }
}
void ComputerSystem::joinThread() {
    if (monitorThread.joinable()) {
        monitorThread.join();
    }
}

void ComputerSystem::monitorAirspace() {
	//std::cout << "Initial is_empty value: " << shared_mem->is_empty.load() << std::endl;
	ATCTimer timer(1,0);
	// Vector to store plane data
	std::vector<msg_plane_info> plane_data_vector;
	uint64_t timestamp;
    // Keep monitoring indefinitely until `stopMonitoring` is called
	while (shared_mem->is_empty.load()) {
		std::cout << "Waiting for planes in airspace...\n";
		timer.waitTimer();
	}

	while (running) {
		if (shared_mem->is_empty.load()) {
			std::cout << "No planes in airspace. Stopping monitoring.\n";
			running = false;
	        break;
        } else {
        	plane_data_vector.clear();
        	// Print separator and timestamp
            //std::cout << "\n================= Shared Memory Update =================\n";

            // Display the timestamp in a readable format
            timestamp = shared_mem->timestamp;
            //std::cout << "Last Update Timestamp: " << timestamp << "\n";
            //std::cout << "Number of planes in shared memory: " << shared_mem->count << "\n";

            for (int i = 0; i < shared_mem->count; ++i) {
            	const msg_plane_info& plane = shared_mem->plane_data[i];
            	// Store the plane info in the vector
            	plane_data_vector.push_back(plane);
            }
        }
		//**************Call Collision Detector*********************
		if (plane_data_vector.size()>1)
            checkCollision(timestamp, plane_data_vector);
		else
            std::cout << "No collision possible with single plane\n";
        // Sleep for a short interval before the next poll
       timer.waitTimer();
    }
	std::cout << "Exiting monitoring loop." << std::endl;
}

void ComputerSystem::checkCollision(uint64_t currentTime, std::vector<msg_plane_info> planes) {
    std::cout << "Checking for collisions at time: " << currentTime << std::endl;
    // COEN320 Task 3.4
    // detect collisions between planes in the airspace within the time constraint
    // You need to Iterate through each pair of planes and in case of collision,
    // store the pair of plane IDs that are predicted to collide
    // You can use the function checkAxes provided below to check if two planes will collide
    // COEN320 Task 3.5
    // in case of collision, send message to Display system
    /*
    HINT:
    In case of collision a Message (type Message_inter_process) should be sent to the Display system
    The data field of the message should contain the list of pairs of plane IDs that are predicted to collide
    Make sure to fill dataSize field of the message appropriately
    e.g. (here a std::pair<int,int> is used to represent a pair of colliding planes)
    // Prepare the message
    Message_inter_process msg_to_send;
    std::vector<std::pair<int, int>> collisionPairs;
    // Store the collision pair plane ID 0 and 1
    collisionPairs.emplace_back(<plane 0>, <plane 1>);
    // Serialize collisionPairs
    size_t numPairs = collisionPairs.size();
    size_t dataSize = numPairs * sizeof(std::pair<int, int>);

    msg_to_send.planeID = -1;
    msg_to_send.type = MessageType::COLLISION_DETECTED;
    msg_to_send.dataSize = dataSize;
    std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
    sendCollisionToDisplay(msg_to_send);

    */
    name_attach_t* attach = name_attach(nullptr, COLLISION_CHANNEL, 0);
    std::cout << "Checking for collisions at time: " << currentTime << " with " << planes.size() << " planes\n";

    std::vector<std::pair<int,int>> collisionPairs;
    size_t n = planes.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (checkAxes(planes[i], planes[j])) {
                collisionPairs.emplace_back(planes[i].id, planes[j].id);
                std::cout << "Predicted collision: " << planes[i].id << " <-> " << planes[j].id << "\n";
            }
        }
    }

    if (collisionPairs.empty()) {
        std::cout << "No collisions predicted in this update.\n";
        return;
    }

    // Prepare inter-process message
    Message_inter_process msg_to_send;

    msg_to_send.planeID = -1;
    msg_to_send.type = MessageType::COLLISION_DETECTED;

    size_t numPairs = collisionPairs.size();
    size_t dataSize = numPairs * sizeof(std::pair<int,int>);

    if (dataSize > msg_to_send.data.size()) {
        std::cerr << "Collision data too large to send (" << dataSize << " bytes). Truncating.\n";
        // truncate
        size_t fitPairs = msg_to_send.data.size() / sizeof(std::pair<int,int>);
        dataSize = fitPairs * sizeof(std::pair<int,int>);
    }

    // Copy memory
    std::memcpy(msg_to_send.data.data(), collisionPairs.data(), dataSize);
    msg_to_send.dataSize = static_cast<int>(dataSize);

    // Send
    try {
        sendCollisionToDisplay(msg_to_send);
    } catch (const std::exception& ex) {
        std::cerr << "Failed to send collision message: " << ex.what() << "\n";
    }
    
}

bool ComputerSystem::checkAxes(msg_plane_info p1, msg_plane_info p2) {
    // COEN320 Task 3.4
    // A collision is defined as two planes entering the defined airspace constraints within the time constraint
    // You need to implement the logic to check if plane1 and plane2 will collide within the time constraint
    // Return true if they will collide, false otherwise
    // A simple approach is to just check if their positions will be within the defined constraints (e.g., CONSTRAINT_X, CONSTRAINT_Y, CONSTRAINT_Z)
    // A more accurate approach would involve calculating their future positions based on their velocities
    // and checking if those future positions will be within the defined constraints within the time constraint
    // p1/p2 have PositionX/Y/Z and VelocityX/Y/Z in same units.
    // Choose parameters:
    const double timeHorizon = 30.0;     // seconds to look ahead
    const double sepThreshold = 500.0;   // meters separation threshold
    // Collision using threshold and time (default set: alert when they are 500 meters within the next 30 seconds)

    // Relative position and velocity
    double rx = p2.PositionX - p1.PositionX;
    double ry = p2.PositionY - p1.PositionY;
    double rz = p2.PositionZ - p1.PositionZ;

    double vx = p2.VelocityX - p1.VelocityX;
    double vy = p2.VelocityY - p1.VelocityY;
    double vz = p2.VelocityZ - p1.VelocityZ;

    double v2 = vx*vx + vy*vy + vz*vz;
    double rdotv = rx*vx + ry*vy + rz*vz;

    double tca;
    if (v2 < 1e-6) {
        // Relative velocity near zero, check current distance
        tca = 0.0;
    } else {
        tca = - rdotv / v2;
        if (tca < 0.0) tca = 0.0;
        if (tca > timeHorizon) tca = timeHorizon;
    }

    // position difference at closest approach
    double cx = rx + vx * tca;
    double cy = ry + vy * tca;
    double cz = rz + vz * tca;

    double dist2 = cx*cx + cy*cy + cz*cz;
    bool collision = dist2 <= (sepThreshold * sepThreshold);

    return collision;
}


void ComputerSystem::sendCollisionToDisplay(const Message_inter_process& msg){
	int display_channel = name_open(display_channel_name, 0);
	if (display_channel == -1) {
		throw std::runtime_error("Computer system: Error occurred while attaching to display");
	}
	int reply;

	int status = MsgSend(display_channel, &msg, sizeof(msg), &reply, sizeof(reply));
	if (status == -1) {
		perror("Computer system: Error occurred while sending message to display channel");
	}
}

// Message processor
void ComputerSystem::processMessage() {
    // Create a named channel
    name_attach_t* attach = name_attach(nullptr, COMPUTER_SYSTEM_CHANNEL, 0);
    if (attach == nullptr) {
        std::cerr << "ComputerSystem: name_attach failed: " << strerror(errno) << "\n";
        return;
    }

    std::cout << "ComputerSystem: operator channel attached as '" << COMPUTER_SYSTEM_CHANNEL << "'.\n";

    while (running.load()) {
        Message_inter_process incoming {};
        int rcvid = MsgReceive(attach->chid, &incoming, sizeof(incoming), nullptr);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            std::cerr << "ComputerSystem: MsgReceive error: " << strerror(errno) << "\n";
            break;
        }

        // handle message
        try {
            switch (incoming.type) {
                case MessageType::REQUEST_CHANGE_OF_HEADING:
                case MessageType::REQUEST_CHANGE_POSITION:
                case MessageType::REQUEST_CHANGE_ALTITUDE:
                    // Forward to Communications system to reach aircraft
                    applyOperatorCommand(incoming);
                    sendMessagesToComms(reinterpret_cast<const Message&>(incoming));
                    break;

                case MessageType::CHANGE_TIME_CONSTRAINT_COLLISIONS:
                    handleTimeConstraintChange(reinterpret_cast<const Message&>(incoming));
                    break;

                default:
                    // For other message types, log and ignore or handle as needed.
                    std::cout << "ComputerSystem: Received unsupported message type: "
                              << static_cast<int>(incoming.type) << "\n";
                    break;
            }
        } catch (const std::exception& ex) {
            std::cerr << "ComputerSystem: exception while processing message: " << ex.what() << "\n";
        }

        // Reply to the sender if necessary. If MsgReceive was used with MsgSend (coid/MsgSend),
        // a reply may be required. We'll use MsgReply with 0 status.
        MsgReply(rcvid, EOK, nullptr, 0);
    }

    name_detach(attach, 0);
    std::cout << "ComputerSystem: operator message loop exiting.\n";
}

void ComputerSystem::sendMessagesToComms(const Message& msg) {
    // Attempt to open communications channel and forward the raw Message_inter_process bytes.
    // Since Message_inter_process and Message may differ, prefer sending Message_inter_process if available.
    Message_inter_process mip {};
    // If msg is actually a Message_inter_process in disguise, copy bytes.
    std::memcpy(&mip, &msg, std::min(sizeof(mip), sizeof(msg)));

    int comm_coid = name_open(COMMUNICATIONS_CHANNEL, 0);
    if (comm_coid == -1) {
        std::cerr << "ComputerSystem: name_open failed for '" << COMMUNICATIONS_CHANNEL
                  << "': " << strerror(errno) << ". Command not forwarded.\n";
        return;
    }

    int rc = MsgSend(comm_coid, &mip, sizeof(mip), nullptr, 0);
    if (rc == -1) {
        std::cerr << "ComputerSystem: MsgSend to Communications system failed: "
                  << strerror(errno) << "\n";
    } else {
        std::cout << "ComputerSystem: forwarded command to CommunicationsSystem.\n";
    }
    name_close(comm_coid);
}

void ComputerSystem::handleTimeConstraintChange(const Message& msg) {
    // The operator packed an int into data; ensure sizes are valid
    // We'll interpret msg as Message_inter_process layout (fixed array), so cast:
    const Message_inter_process* mip = reinterpret_cast<const Message_inter_process*>(&msg);
    if (!mip) return;

    if (mip->dataSize < sizeof(int)) {
        std::cerr << "handleTimeConstraintChange: invalid payload size\n";
        return;
    }

    int newFreq = 0;
    std::memcpy(&newFreq, mip->data.data(), sizeof(int));
    if (newFreq <= 0) {
        std::cerr << "handleTimeConstraintChange: invalid frequency " << newFreq << "\n";
        return;
    }

    timeConstraintCollisionFreq = newFreq;
    std::cout << "ComputerSystem: collision time constraint updated to "
              << timeConstraintCollisionFreq << " seconds.\n";
}

void ComputerSystem::applyOperatorCommand(const Message_inter_process& msg) {
    // Each aircraft has its own channel named "chris<planeID>"
    std::string channelName = "chris" + std::to_string(msg.planeID);
    int plane_coid = name_open(channelName.c_str(), 0);
    if (plane_coid == -1) {
        std::cerr << "Failed to open channel for plane " << msg.planeID
                  << ": " << strerror(errno) << "\n";
        return;
    }

    // Forward the message directly to the aircraft
    int rc = MsgSend(plane_coid, &msg, sizeof(msg), nullptr, 0);
    if (rc == -1) {
        std::cerr << "Failed to send operator command to plane " << msg.planeID
                  << ": " << strerror(errno) << "\n";
    } else {
        std::cout << "Operator command sent to plane " << msg.planeID << "\n";
    }

    name_close(plane_coid);
}

