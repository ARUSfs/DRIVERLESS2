<p align="center" style="margin-top: 50px; margin-bottom: 50px;">
    <a href="https://www.arusteam.com/">
        <img src="https://images.squarespace-cdn.com/content/v1/5e45d3a8e509f61738454469/5ac8e64c-a831-40c4-b413-81f0c7a44bf7/2.+NEGATIVO.png" alt="ARUS Team">
    </a>
</p>

# CAN Interface

The CAN interface is the node that establishes a connection between the sensors and integrates the circuit boards that send information through CAN messages and the entire ROS2 pipeline of **DRIVERLESS2**. 

This year's CAN node code focuses mainly on clarity and ease of adding new messages to be sent by ROS2, which has led us to implement csv where these modifications are made without touching the code.

This file contains the necessary information to install the package on a computer. This guide includes the folowing sections:
- [**Dependencies**](#dependencies). Dependencies used in the node.
- [**Connections**](#connections). All the subscribers it needs and the publishers it generates.
- [**Complete documentation**](#complete-documentation). Complete detailed documentation of the node, covering the functions in the code and a step-by-step guide to how the code works.


## Dependencies

### System Libraries

- **`linux/can.h`** Defines structures and macros for working with the CAN (Controller Area Network) protocol in Linux.

- **`linux/can/raw.h`**	Provides support for raw CAN sockets in Linux.

- **`linux/can/error.h`**	Contains definitions for CAN error handling.

- **`net/if.h`**	Provides network interface-related functions for socket communication.

- **`sys/ioctl.h`**	Allows manipulation of device parameters using I/O control operations.

- **`sys/socket.h`**	Defines socket interface for network communication.
- **`cstring`** Standard C++ string manipulation functions (e.g., memcpy, strlen).
- **`iostream`**	Standard C++ I/O stream handling.
- **`cstdlib`**	General-purpose functions for memory allocation, random number generation, etc.
- **`unistd.h`**	Provides access to POSIX operating system API (e.g., file handling, sleep).
- **`stdio.h`**	Standard C I/O library.
- **`thread`**	C++ standard library for multithreading support.
- **`fstream`**	File stream handling for reading and writing files.
- **`sstream`**	String stream handling for formatted I/O operations.
- **`string`**	C++ string class.
- **`map`**	C++ Standard Template Library (STL) map container.
- **`vector`**	C++ STL vector container.
- **`fcntl.h`**	File control options, such as setting non-blocking mode for file descriptors.
- **`stdexcept`**	Provides standard exceptions like runtime_error and invalid_argument.
- **`cerrno`**	Defines error codes used by system calls (e.g., errno).


### ROS 2 Dependencies

- **`rclcpp/rclcpp.hpp`** The main C++ API for writing ROS 2 nodes.
- **`std_msgs/msg/int16.hpp`** Standard message type for 16-bit integers.
- **`std_msgs/msg/float32.hpp`** Standard message type for 32-bit floating point numbers.
- **`std_msgs/msg/float32_multi_array.hpp`** Standard message type for an array of 32-bit floats.
- **`std_msgs/msg/bool.hpp`** Standard message type for boolean values.
- **`geometry_msgs/msg/vector3.hpp`** Message type for 3D vectors.
- **`common_msgs/msg/car_info.hpp`** Custom message from car_state with all car data.
- **`common_msgs/msg/cmd.hpp`** Custom command message for vehicle control of the inversors.

- **`ament_index_cpp/get_package_share_directory.hpp`** Helper functions for locating package resources in ROS 2.


## Connections

### Subscribers
This section describes the ROS 2 subscribers used in this package. Each subscriber listens to a specific topic and triggers a corresponding callback function to handle the received messages.

#### 1. control_sub_	
- **Topic:** /controller/cmd
- **Message Type:** common_msgs::msg::Cmd
- **Callback Function:** control_callback
- **Purpose:** Receive from the Controller node all the commands to be sent to the engine inverter and then send them via CAN.

#### 2. car_info_sub_	
- **Topic:** /car_state/car_info
- **Message Type:** common_msgs::msg::CarInfo
- **Callback Function:** car_info_callback
- **Purpose:** Receive from the car_state node all the car information needed to build the data logger messages.

#### 3. run_check_sub_
- **Topic:** /car_state/run_check
- **Message Type:** std_msgs::msg::Bool
- **Callback Function:** run_check_callback
- **Purpose:** Receive from the car_state node if the check is present that the car can start running and send commands to the inverter.

### Topics
The ROS2 topics by which information is sent and received through the CAN are given by the CSV, for more information on these, go to the include folder and in can.csv you will find them.

## Complete documentation

This section gives detailed information on each part of the code:

### Table of Contents
1. [CSV structure](#1-CSV-structure)
2. [Functions](#2-Functions)
3. [Message processing pipeline](#3-Message-processing-pipeline)

### 1. CSV structure

The code uses two csv:


#### 1.1 can_aux.csv
This one has three columns:

- **`ID`** The id of the message to be read by CAN.

- **`subID`** Here you specify if the ID you want to read has a subID, if yes you specify it and if no you specify `no`.

- **`nMssg`** The number of data you want to read from the ID.

#### 1.2 can.csv 
This one has eight columns: 

- **`ID`** The dynamic ID which is constructed based on the ID and the number of messages, if the ID contains a single message the ID is set as is (e.g., 0x300). If there are several messages, the ID is concatenated with the number of messages (e.g., 0x3001, 0x3002).

- **`bitIn`** The number of the bit on which the message begins.

- **`bitFin`** The number of the bit on which the message ends.

- **`Signed`** Whether the data transmitted by car is signed or unsigned.

- **`Power`** The power to which it raises the data transmitted by CAN.

- **`Scale`** The scale to be multiplied to the transmitted data once it has been raised.

- **`Offset`** The offset that is added to the data after raising and scaling.

- **`Publisher`** The topic by which the data is transmitted by ROS2

### 2. Functions

#### read_csv(filepath)
- **Purpose:** Loads CSV files and stores them in a map for reference.
- **Returns:** std::map<std::string,std::vector<std::string>>
- **Process:**
	1.	Opens the CSV file.
	2.	Reads the header.
	3.	Parses each row and stores it in a map (ID → corresponding data).
	4.	Returns the map.

#### read_CAN(socketCan)
- **Purpose:** Reads incoming CAN messages and processes them based on CSV configurations.
- **Process:**
	1.	Reads a CAN frame.
	2.	Checks if frame_id exists in csvdata_aux.
	3.	If it does, processes its auxiliary vector and applies filter_subID().
	4.	Generates a dynamic key and searches in csvdata_main.
	5.	Extracts message data and calls parse_msg().
	6.	Publishes extracted data.

#### parse_msg(frame, config)
- **Purpose:** Converts CAN message bytes into meaningful numerical values.
- **Process:**
	1.	Extracts data from startByte to endByte.
	2.	Converts it to an integer (rawValue).
	3.	Applies transformations (power, scale, offset).
	4.	Publishes the result to the appropriate ROS 2 topic.

#### filter_subID(frame, aux_vector_subID)
- **Purpose:**  Determines if a received CAN frame should be processed based on its subID.
- **Returns:** true if the frame matches the expected subID, false otherwise.

#### control_callback(msg)
- **Purpose:** Converts a received acceleration command to a CAN frame and sends it to the inverter.
- **Process:**
	1.	Checks if run_check_ is true.
	2.	Converts acceleration to a 16-bit integer.
	3.	Sends a CAN frame (0x201) with the converted command.

#### car_info_callback(msg)
- **Purpose:** Updates vehicle status and sends appropriate messages if needed.
- **Process:**
	1.	Updates local state variables.
	2.	If as_status_ == 4, sends a CAN frame (0x202) indicating an emergency or completion state.

#### run_check_callback(msg)
- **Purpose:** Updates the system’s run-check status.
- **Process:**
	1.  Sets run_check_ to the value given.

#### heart_beat_callback()
- **Purpose:** Sends periodic heartbeat messages over CAN.
- **Process:**
	1.	Sends a CAN frame (0x140) every 0.1 seconds.

#### dl_timer_callback()
- **Purpose:** Sends the data logger messages periodically.
- **Process:**
	1.	Calls send_dl500(), send_dl501(), and send_dl502() to send the different messages.

#### send_dl500()
- **Purpose:** Sends the data logger message with ID 0x500.
- **Process:**
	1.	Converts and clamps values to their respective ranges.
	2.	Packs data into an 8-byte CAN frame.
	3.	Sends the frame via socketCan1.

#### send_dl501()
- **Purpose:** Sends the data logger message with ID 0x501.
- **Process:**
	1.	Converts acceleration and yaw rate to signed 16-bit integer values.
	2.	Stores values in Little-Endian format.
	3.	Sends the frame via socketCan1.

#### send_dl502()
- **Purpose:** Sends the data logger message with ID 0x502.
- **Process:**
	1.	Packs multiple status variables into a 5-byte bitfield.
	2.	Uses bitwise operations for efficient storage.
	3.	Sends the frame via socketCan1.

### 3. Message processing pipeline

This section outlines the flow of data from incoming CAN messages to ROS 2 topics. The system ensures that raw CAN frames are properly parsed, processed, and published in real-time.

#### 3.1. Reading Data
The system continuously listens for incoming CAN messages and processes them in real time.
1.	**CAN Frame Reception:**
	- The function read_CAN(int socketCan) runs in a loop, constantly reading messages from the assigned CAN socket (socketCan0 or socketCan1).

	- Each incoming message contains an ID, DLC (Data Length Code), and data payload.

2.	**Validation Against csvdata_aux**
	- The received message ID (frame_id) is checked against csvdata_aux, which contains auxiliary metadata about CAN messages.

	- If the ID is found:
	  - The system retrieves the number of expected messages (nMssg).

	  - It determines whether a subID filter applies using filter_subID().

3.	**Dynamic Key Construction:**
	- If the message has multiple sub-messages (nMssg > 1), a dynamic key is constructed (frame_id + sub-message index).

	- The dynamic key is used to match the corresponding parsing configuration in csvdata_main.

#### 3.2. Processing Data
If a match is found in csvdata_main, the raw CAN frame undergoes several transformations:

1.	**Extracting Bytes from the Frame:**

	- The system extracts the relevant bytes from the CAN payload using startByte and endByte specified in csvdata_main.
2.	**Converting Raw Bytes to Integer (parse_msg())**
	- Depending on the byte length, the system interprets the data as:

	  - Unsigned Integer (uint8_t, uint16_t, uint32_t)

	  - Signed Integer (int8_t, int16_t, int32_t)

	- If signed data is specified (isSigned == "yes"), sign extension is performed.
3.	**Applying Transformations:**
	-	The raw integer value is transformed using the formula:
  
  $$
  \text{scaledValue} = (\text{rawValue}^\text{Power} \times \text{Scale}) + \text{Offset}
  $$

#### 3.3. Publishing Data
Once the processed data is obtained, it is published to the corresponding ROS 2 topic:

1.	**Finding the Correct ROS 2 Publisher:**

	-	The system looks up the publisher associated with the processed dynamic key in the publishers map.

2.	**Publishing the Data:**

	-	A std_msgs::msg::Float32 message is created.

	-	The computed scaledValue is assigned to msg.data.

	-	The message is published to its ROS 2 topic.

3.	**Debug Logging (if DEBUG == true)**

  - The system prints the processed CAN frame details, including:

    -	Frame ID

    -	Extracted data

	  -	Scaled and converted values

	  -	Associated ROS topic

This ensures that the extracted CAN data is made available to other ROS 2 nodes in a structured and meaningful format.