# Kinect 2 Client
This ROS package is part of a two part software package which allows data to be streamed from a Microsoft Kinect v2 running on a Windows machine over the network to ROS. The corresponding Kinect 2 Server program reads data from the Kinect and dumps it over the network. The k2_client package recieves the data and publishes it as a series of ROS topic.

Both the server and client components were developed by [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu/). 

## Installation - Server side
On the Windows machine:
The server software can be installed and run in two ways: either by downloading the source code and manually building, or by using pre-built binaries. For the robotic surgery course we used the source code for greater flexibility.

### Prequisites
1. Windows 8.1 or higher
2. Microsoft Visual Studio Express 2013 for Windows Desktop, which can be found [here](http://www.microsoft.com/en-gb/download/details.aspx?id=43733)
3. Kinect v2.0 SDK, which can be found [here](http://www.microsoft.com/en-us/download/details.aspx?id=43661)

### Building from source
1. Download the source code from Github [https://github.com/personalrobotics/k2_server]. The latest stable release can be found at [https://github.com/personalrobotics/k2_server/releases].

2. Connnect the Kinect 2.0 to a USB 3.0 port and check that it is working using the SDK sample applications. Once done, close all the Kinect applications.

3. Disable IPv6 on the machine by following the instructions on following page. [http://www.techunboxed.com/2012/08/how-to-disable-ipv6-in-windows-8.html]

4. Open the k2_server project in Visual Studio Express 2013, and click build to compile the solution.

5. Run the program by running the kinect2server.exe executable in the \bin folder of the Visual Studio project. A Kinect icon in the icon tray indicates that the software is running.

### Prebuilt binaries
1. Download the binaries from Github: 
[https://github.com/personalrobotics/k2_server/releases]

2. Disable IPv6 on the machine by following the instructions on following page. [http://www.techunboxed.com/2012/08/how-to-disable-ipv6-in-windows-8.html] 

5. Run the program by running the kinect2server.exe executable. A Kinect icon in the icon tray indicates that the software is running.

## Client side
### Prerequisites
The k2_client package requires libjsoncpp-dev to be installed. Use the following commands:
`sudo apt-get update`

`sudo apt-get install libjsoncpp-dev`

For the robotic_surgery project these files are automatically installed inside the docker file.

### Installing k2_client
1. Download the k2_client package from Github. The original Personal Robotics version can be found at [https://github.com/personalrobotics/k2_client]. 

2. Make sure that either k2_client is in the current ROS workspace or is referenced in the assemby by adding the following line to the end of ~/.bashrc
`export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/TO/PACKAGE/THE/PACKAGE`

3. In the k2_client.launch launch file, edit the value of the `serverNameOrIP` parameter to the IP address of the server computer.

4. Build the package using `rosmake`

5. Run the package using 
`roslaunch k2_client kinect2Client.launch` 

### Using K2 Client
To run the software and stream Kinect data, you may wish to follow the following workflow:
1. Check that the Kinect is working correctly on the Windows machine

2. Run the Kinect2Server.exe executable. When the program is running, a Kinect icon will appear in the icon tray in the bottom-right of the screen

3. When you first start Kinect2Server, the Kinect icon will appear in red. Once a connection can be established with the Kinect and the network

On the Linux machine:
4. Launch the required k2_client nodes, either using the k2_client.launch file or a custom launch file.
>> Make sure to set the IP address of the server machine by adding
`<param name="serverNameOrIP" value="my-ip-address" />` to your launch file.


## Nodes
The k2_client package contains 5 nodes, each of which exposes a number of different topics. All topics are prefixed by /head/kinect2.

* **startRGB**
  - rgb/camera_info
  - rgb/image_color
  - rgb/image_color/compressed
  - rgb/image_color/compressed/parameter_descriptions
  - rgb/image_color/compressed/parameter_updates
  - rgb/image_color/compressedDepth
  - rgb/image_color/compressedDepth/parameter_descriptions
  - rgb/image_color/compressedDepth/parameter_updates
  - rgb/image_color/theora
  - rgb/image_color/theora/parameter_descriptions
  - rgb/image_color/theora/parameter_updates|
* **startDepth**
  - depth/camera_info
  - depth/image_raw
  - depth/image_raw_compressed
  - depth/image_raw/compressed/parameter_descriptions
  - depth/image_raw/compressed/parameter_updates
  - depth/image_raw/compressedDepth
  - depth/image_raw/compressedDepth/parameter_descriptions
  - depth/image_raw/theora
  - depth/image_raw/theora/parameter_descriptions
  - depth/image_raw/theora/parameter_updates
* **startIR**
  - ir/camera_info
  - ir/image_ir
  - ir/image_ir/compressed
  - ir/image_ir/compressed/parameter_descriptions
  - ir/image_ir/compressed/parameter_updates
  - ir/image_ir/compressedDepth
  - ir/image_ir/compressedDepth/parameter_descriptions
  - ir/image_ir/compressedDepth/parameter_updates
  - ir/image_ir/theora
  - ir/image_ir/theora/parameter_descriptions
  - ir/image_ir/theora/parameter_updates
* **startAudio**
  - audio
* **startBody**
  - bodyArray



