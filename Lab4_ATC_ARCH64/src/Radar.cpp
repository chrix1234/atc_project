#include "Radar.h"
#include <sys/dispatch.h>


Radar::Radar(uint64_t& tick_counter) : tick_counter_ref(tick_counter), activeBufferIndex(0), timer(1,0), stopThreads(false) {
	// Start threads for listening to airspace events
    Arrival_Departure = std::thread(&Radar::ListenAirspaceArrivalAndDeparture, this);
    UpdatePosition = std::thread(&Radar::ListenUpdatePosition, this);
    Radar_channel = NULL;
    clearSharedMemory();  //For future Use

}

Radar::~Radar() {
    // Join threads to ensure proper cleanup
    shutdown();
    clearSharedMemory();//For future Use */
}

void Radar::shutdown() {
    // Set stop flag and wait for threads to complete
    stopThreads.store(true);

    // If the channel exists, close it properly
    if (Radar_channel) {
        name_detach(Radar_channel, 0);
    }

    if (Arrival_Departure.joinable()) {
        Arrival_Departure.join();
    }
    if (UpdatePosition.joinable()) {
        UpdatePosition.join();
    }
}


// Method to get the current active buffer
std::vector<msg_plane_info>& Radar::getActiveBuffer() {
    return planesInAirspaceData[activeBufferIndex];
}
//Coen320_Lab (Task0): Create channel to be reachable by radar that wants to poll the Airplane
//Radar Channel name should contain your group name
//To choose the channel with concatenating your group name with "Radar"
//Note: It is critical to not interfere other groups
void Radar::ListenAirspaceArrivalAndDeparture() {
	Radar_channel = name_attach(NULL, "chris_Radar", 0);
	if (Radar_channel == NULL) {
		std::cerr << "Failed to create channel for Radar" << std::endl;
		exit(EXIT_FAILURE);
	}
	// Simulated listening for aircraft arrivals and departures
    while (!stopThreads.load()) {
        // Replace with IPC
        Message msg;
        int rcvid = MsgReceive(Radar_channel->chid, &msg, sizeof(msg), nullptr); // Replace with actual channel ID
        if (rcvid == -1) {
        	// Silently skip if MsgReceive fails, but no crash happens
        	// std::cerr << "Error receiving airspace message:" << strerror(errno) << std::endl;
        	continue;
        }

        // Reply back to the client
        int msg_ret = msg.planeID;
        MsgReply(rcvid, 0, &msg_ret, sizeof(msg_ret)); // Send plane's ID back to airplane

        switch (msg.type) {
        case MessageType::ENTER_AIRSPACE:
            addPlaneToAirspace(msg);
            break;
        case MessageType::EXIT_AIRSPACE:
            removePlaneFromAirspace(msg.planeID);
            break;
        default:
        	//All other messages dropped
            //std::cerr << "Unknown airspace message type" << std::endl;
        	break;
        }

    }
}

void Radar::ListenUpdatePosition() {

    while (!stopThreads.load()) {
    	timer.waitTimer(); // Wait for the next timer interval before polling again
    	// Only poll airspace if there are planes
        if (!planesInAirspace.empty()) {
            pollAirspace();  // Call pollAirspace() to gather position data
            writeToSharedMemory();  // Write active buffer to shared memory //For future Use
            wasAirspaceEmpty = false;
        } else if (!wasAirspaceEmpty){
        	// Only write empty buffer once after transition to empty
        	writeToSharedMemory();  // Write to shared mem when all planes have left the airspace //For future Use
        	wasAirspaceEmpty = true;  // Set flag to indicate airspace is empty
        } else{
        	//std::cout << "Airspace is empty\n";
        }

    }
}

void Radar::pollAirspace(){

	airspaceMutex.lock();
	// Make a copy of the current planes in airspace to avoid modification during iteration
	std::unordered_set<int> planesToPoll = planesInAirspace;
	airspaceMutex.unlock();

	int inactiveBufferIndex = (activeBufferIndex + 1) % 2;
	std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[inactiveBufferIndex];
	inactiveBuffer.clear();


	//make channel to aircraft
	for (int planeID: planesToPoll){

		airspaceMutex.lock();
		bool isPlaneInAirspace = planesInAirspace.find(planeID) != planesInAirspace.end();
		airspaceMutex.unlock();
		if (isPlaneInAirspace){
			try {
			// Confirm that the plane is still in airspace
				msg_plane_info plane_info = getAircraftData(planeID);
				inactiveBuffer.emplace_back(plane_info);
			} catch (const std::exception& e) {
				// if error to process plane get next id and exception description
				//std::cerr << "Radar: Failed to get plane data " << planeID << ": " << e.what() << "\n";
				continue;
			}
		}


		{
			std::lock_guard<std::mutex> lock(bufferSwitchMutex);
		    activeBufferIndex = inactiveBufferIndex;
		}
	}
}

msg_plane_info Radar::getAircraftData(int id) {
	//Coen320_Lab (Task0): You need to correct the channel name
	//It is your group name + plane id

	std::string id_str = "chris"+std::to_string(id);  // Convert integer id to string
	const char* ID = id_str.c_str();         // Convert string to const char*
	int plane_channel = name_open(ID, 0);

	if (plane_channel == -1) {
		throw std::runtime_error("Radar: Error occurred while attaching to channel");
	}

	// Prepare a message to request position data
	Message requestMsg;
	requestMsg.type = MessageType::REQUEST_POSITION;
	requestMsg.planeID = id;
	requestMsg.data = NULL;

	// Structure to hold the received position data
	Message receiveMessage;

	// Send the position request to the aircraft and receive the response
	if (MsgSend(plane_channel, &requestMsg, sizeof(requestMsg), &receiveMessage, sizeof(receiveMessage)) == -1) {
		name_close(plane_channel);
		throw std::runtime_error("Radar: Error occurred while sending request message to aircraft");
	}

	msg_plane_info received_info = *static_cast<msg_plane_info*>(receiveMessage.data);

	// Close the communication channel with the aircraft
	name_close(plane_channel);

	return received_info;
}

void Radar::addPlaneToAirspace(Message msg) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	int plane_data = msg.planeID;
    planesInAirspace.insert(plane_data);
    std::cout << "Plane " << msg.planeID << " added to airspace" << std::endl;
}

void Radar::removePlaneFromAirspace(int planeID) {
	std::lock_guard<std::mutex> lock(airspaceMutex);
	planesInAirspace.erase(planeID);  // Directly remove the integer from the list
	std::cout << "Plane " << planeID << " removed from airspace" << std::endl;
}

void Radar::writeToSharedMemory() {
	const char *name = "/radar_shm";

	// Open shared memory
    int shm_fd = shm_open(name, O_RDWR, 0666);
    if (shm_fd == -1) {
        fprintf(stderr, "shm_open (write) failed: %s\n", strerror(errno));
        return;
    }

    // Map the shared memory segment in the address space of the process
    void *shared_mem = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        fprintf(stderr, "mmap (write) failed: %s\n", strerror(errno));
        close(shm_fd);
        return;
    }

    // Cast
	SharedMemory* ptr = static_cast<SharedMemory*>(shared_mem);

    // Determine capacity of plane_data safely:
    size_t capacity = sizeof(ptr->plane_data) / sizeof(ptr->plane_data[0]);

    // Write to shared memory
    // Get the active buffer based on the current active index
    std::vector<msg_plane_info>& activeBuffer = getActiveBuffer();

    // Get the current timestamp
    ptr->timestamp = tick_counter_ref;

	// Check if activeBuffer is empty and set the flag accordingly
    if (activeBuffer.empty()) {
        std::vector<msg_plane_info>& inactiveBuffer = planesInAirspaceData[(activeBufferIndex + 1) % 2];
        if (!inactiveBuffer.empty()) {
            // copy at most capacity elements
            size_t to_copy = std::min(inactiveBuffer.size(), capacity);
            ptr->is_empty.store(false);
            ptr->count = to_copy;
            std::memcpy(ptr->plane_data, inactiveBuffer.data(), to_copy * sizeof(msg_plane_info));
            inactiveBuffer.clear();
        } else {
            // no data at all
            ptr->is_empty.store(true);
            ptr->count = 0;
        }
    } else {
        size_t to_copy = std::min(activeBuffer.size(), capacity);
        ptr->is_empty.store(false);
        ptr->count = to_copy;
        std::memcpy(ptr->plane_data, activeBuffer.data(), to_copy * sizeof(msg_plane_info));
        activeBuffer.clear();
    }

    // cleanup
    munmap(shared_mem, SHARED_MEMORY_SIZE);
    close(shm_fd);

}

void Radar::clearSharedMemory() {
	const char *name = "/radar_shm";

	// Create shared memory
    int shm_fd = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        fprintf(stderr, "shm_open (create) failed: %s\n", strerror(errno));
        return;
    }

	// Configure size of shared memory
    // Ensure the shared memory is the required size
    if (ftruncate(shm_fd, SHARED_MEMORY_SIZE) == -1) {
        fprintf(stderr, "ftruncate failed: %s\n", strerror(errno));
        close(shm_fd);
        return;
    }

    void *shared_mem = mmap(nullptr, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shared_mem == MAP_FAILED) {
        fprintf(stderr, "mmap (create) failed: %s\n", strerror(errno));
        close(shm_fd);
        return;
    }

    SharedMemory* ptr = static_cast<SharedMemory*>(shared_mem);
    std::memset(ptr, 0, sizeof(SharedMemory));
    ptr->is_empty = 1;     // mark empty
    ptr->count = 0;
    ptr->timestamp = 0;
	// Finally unmap the shared memory and close the file descriptor
	munmap(shared_mem, SHARED_MEMORY_SIZE);
//	shm_unlink(name);
	close(shm_fd);
    }
