# Manual for Agricultural Mobile Mapping Device

This Readme file will instruct you on how to successfully operate this machine.
Note that the order in which the commands are executed does matter. 
Do not skip any point on the list.

## Connecting to the device

The connection is provided by SSH.
Set up a static IP address on the guest machine using 10.42.0.0.
The host device will be at 10.42.0.1.
Power up the device, wait, and connect to the following WiFi: 

	- WIFI: blubb-NUC

	- Password: blubb123 

Many scripts on the device use X forwarding for gnome-terminal tabs. Thus, to connect use the following command:

```bash
ssh -X blubb@10.42.0.1 gnome-terminal --disable-factory
```

The credentials are:

	-Username: blubb
	-Password: blubb

Once you are logged in, you may want to check on the guest machine wether the data looks OK.
Therefore, set your ROS_IP and ROS_MASTER_URI flag accordingly:

```bash
$ export ROS_IP=10.42.0.1
$ export ROS_MASTER_URI=http://10.42.0.1:11311
``` 

Once this is done, *do not leave the tab*, but execute the following commands into the same terminal tab. 

## Starting sensors

### 1. Network clock

First, we start by setting up the network clock, which is used to timestamp the LiDAR data.
This is one of the *most important* steps for precise reconstruction. 
The stamping is done by the Ethernet port of the NUC, where the LiDARs are plugged. 
Usually, the ptpd clock client should start after logging in.
Just to be sure: To start the clock, use the script which is in the home directory:

```bash
$ ./launch_network_clock.sh
```

Enter the password. 
If the command exits immediatly, the clock is likely running already.
Check if the LiDAR data has timestamps using:

```bash
$ rostopic echo /livox/lidar_3GGDJ7U00100171/header/stamp
```  

Which should display rather huge numbers, e.g. larger than "secs: 1696423320".
If the number appears to be significantly smaller than this, the network clock is not working (bad). 

### 2. Inertial Measurement Unit (IMU for Orientation)

The IMU measures orientation and is started straightforward using a script in the home directory:
DO NOT MOVE the whole setup for 2 seconds after launching the script, as the IMU will self-calibrate. 

```bash
$ ./launch_imu.sh
```

### 3. GPS Antennas (Position Measurement) 

The GPS antennas are plugged into the serial port of the NUC.
They continuously stream the data.
To record the GPS antennas, use the following script in the home directory:

```bash
$ ./launch_gps.sh
```

### 4. LiDAR sensors

For the two onboard LiDAR sensors it is crucial to know the extrinsic parameters (Position / Orientation calibration) between the two LiDARs itself, but also from the LiDARs to the GPS antennas. 
The calibration has been already performed (contact: fabian.arzberger@uni-wuerzburg.de) and is loaded together with the LiDAR sensors using the following script, located in the home directory:

```bash
$ ./launch_scanners.sh
```

## Recording the data

Note that every sensor (except for the GPS antennas) uses ROS.
Thus, once the previous steps have been executed (make sure that GPS is recording by watching the bitrate of the file-stream), you may record all the data at once:

```bash 
$ rosbag record -a 
```
Stop the recording using Ctrl+C.
This will create a new ".bag"-file in the current directory.

## Technical details

If you need to debug something, or build new software on the device, the following information will be valuable: 

- LiDAR:
	- Topic: /livox/lidar_3GGDJ7U00100171
	- Frame: livox_frame_1
	- Location: Towards the ON/OFF button box

	- Topic: /livox/lidar_3GGDJ7W00100801
	- Frame: livox_frame_2
	- Location: Towards the NUC box

	- Launchfile location: ~/ws_livox/src/livox_ros_driver/launch

	- RVIZ Visualization: Load "kaveh.rviz" config file

 - TF tree for convenience:
   ![image](https://github.com/fallow24/Soil3D/blob/master/frames.png)

 - Lidar frames and data vizualized in the lab:
   
