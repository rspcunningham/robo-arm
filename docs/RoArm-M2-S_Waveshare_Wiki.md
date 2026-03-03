# RoArm-M2-S

**From Waveshare Wiki**

[Product Page](https://www.waveshare.com/roarm-m2-s.htm)

---

## Overview

RoArm-M2-S is a 4DOF smart robotic arm designed for innovative applications. Adopts lightweight structure design with a total weight of less than 850g and an effective payload of 0.5kg@0.5m, it can be flexibly mounted on various mobile platforms. Adopts a 360° omnidirectional base combined with three flexible joints to create a workspace with a 1-meter diameter.

The joint direct-drive design enhances repositioning precision and also improves structural reliability, with innovative dual-drive technology doubling the shoulder joint torque. The onboard ESP32 MCU main control module supports multiple wireless control modes and provides control interfaces and rich communication protocols for easily connecting to various devices. Compatible with ROS2 and various host computers, supports various wireless and wired communication modes.

**Interfaces:** I2C, UART, TTL Serial Bus Servo Control Interface

Comes with an expansion plate, and supports customizing the EoAT (End of Arm Tooling) to meet innovative application requirements. Provides a user-friendly and cross-platform WEB application that integrates a simple and visualized coordinate control mode, making it easier to get started. Comes with rich graphic and video tutorials to help you learn and use it quickly.

RoArm-M2-S achieves an excellent balance between lightweight, user-friendliness, expandability, and open innovation, it is a multifunctional robotic arm that integrates intelligent control, human-machine interaction, and customizable development. Ideal for applications that require a combination of flexibility, expandability, and user-friendliness.

---

## Features

- **Expandability:** Comes with various expandable components, and supports multifunctional EoAT (End of Arm Toolings) customization to meet different innovative application needs.
- **Omnidirectional Workspace:** The base, with 360° rotation, combined with flexible joint movements, creates a workspace with a diameter of up to 1 meter, enabling versatile movement in all directions.
- **Easy to use:** Offers cross-platform web applications and coordinate control modes to reduce user difficulties, making it easy for users to operate the robotic arm.
- **Open-source Code:** The control code and communication interface documentation for RoArm-M2-S are open-source, facilitating secondary development for users.
- **Lightweight Structure:** Utilizes carbon fiber and 5052 aluminum alloy, resulting in a body weighing less than 850g, making it easy to install on various mobile platforms.
- **Joint Direct-Drive Design:** Uses 12-bit high-precision magnetic encoders to obtain joint angles, ensuring a repeat positioning accuracy of 0.088°, thereby enhancing structural reliability.
- **Dual-Drive Technology:** Innovative dual-drive technology increases the torque of the robotic arm's shoulder joint, improving the overall load capacity of the robotic arm.
- **Powerful Main Control:** Utilizes the ESP32 main control MCU, supporting various control interfaces and wireless communication protocols.
- **Multi-platform Support:** Compatible with ROS2 and various host computers, supporting multiple wireless and wired communication methods, providing flexibility for different development and control requirements.
- **Rich Tutorials:** Offers rich graphic and video tutorials covering various functionalities to help users quickly get started and use the robotic arm, thereby accelerating the realization of innovative applications.

---

## Product Video

[RoArm-M2-S video](https://www.youtube.com/watch?v=PstPVWVywZc)

---

## How to Use

### Precautions

Before use, please learn the following:

1. RoArm-M2-S is pre-assembled before shipment, but it is not recommended to disassemble it due to a significant number of servos.
2. The working voltage range of RoArm-M2-S is 7-12.6V. It is recommended to use the 12V 5A power supply, or you can use a 3S lithium battery. It is strictly forbidden to use a power supply that exceeds this working voltage range.
3. It may bring you potential risks that the torque of RoArm-M2-S is large. When using the product, avoid having sensitive areas such as eyes and head within the range of servo movement.
4. Keep this product away from children to prevent injury. And the product can not be subjected to violent impact.
5. For safety reasons, the default demos have a relatively slow speed for the robotic arm's operation. You can refer to subsequent tutorials to change this speed, but excessively low speed may cause shaking when the robotic arm reaches certain positions.

### Introduction

The robotic arm has the following key parts and interfaces:

- **Elbow joint**
- **Shoulder joint**
- **Base joint**
- **EoAT** (End of Arm Tooling)
- **WiFi** antenna
- **Power jack**
- **OLED** display
- **LED**

#### Getting Started

1. The two images in the wiki show the markings for various parts of the robotic arm and the commonly used interfaces.
2. To use, connect the provided 12V 5A power cable to the power interface on the robotic arm, turn on the power switch, and the product's joints will automatically move to the center position.
3. After powering on, the OLED screen will display the following information:
   - The first line indicates that the WiFi is in AP mode, and the WiFi hotspot is named **RoArm-M2**.
   - The second line indicates that the STA mode is turned off. When WiFi is in STA mode, the router will assign an IP address, which will be displayed.
   - The content of the third line is the MAC address of the device, which is unique and used for ESP-NOW communication. Please refer to [RoArm-M2-S ESP-NOW Control](https://www.waveshare.com/wiki/RoArm-M2-S_ESP-NOW_Control) for specific usage.
4. After powering on, use a smartphone or computer to connect to RoArm-M2-S's WiFi: **RoArm-M2**, with the password **12345678**. Once connected to the WiFi, open the Google Chrome browser and enter **192.168.4.1** in the address bar to access the web-based control interface. From there, you can use the web interface features to control the robotic arm.

---

## Web Usage

### Key Function

#### AngleCtrl: Servo Angle Control

The number displayed above each column of buttons represent the angles to which each joint has rotated, displayed in radians. The labels for each joint can be found on the image in the Introduction section. All displayed numbers are updated only after the button is released.

1. **"B L" and "B R"** (first column) control the rotation of the base joint, with a rotation range of 360°. When the product is powered on, the base joint will automatically rotate to the middle position, and the number on the button is 0.
   - Keep clicking on "B L", the base joint turns left at 180°, and the value updates from 0 to 3.14.
   - Then, keep clicking on "B R", the base joint turns right at 360°, and the value updates from 3.14 to -3.14.

2. **"S D" and "S U"** (second column) control the rotation of the shoulder joint, with a rotation range of 180°. When the product is powered on, the shoulder joint will automatically rotate to the middle position, and the number on the button is 0.
   - Keep clicking on "S D", the shoulder joint rotates forward at 90°, and the value updates from 0 to -1.57.
   - Then, keep clicking on "S U", the shoulder joint rotates reversely at 180°, and the value updates from -1.57 to 1.57.

3. **"E D" and "E U"** (third column) control the rotation of the elbow joint, with a rotation range of 225°. When the product is powered on, the elbow joint will automatically rotate to the middle position, and the number on the button is 1.57.
   - Keep clicking on "E D", the elbow joint will rotate downward at 90°, and the value will change from 1.57 to 3.14.
   - Then keep clicking on "E U", the elbow joint will rotate reversely at 225°, and the value will change from 3.14 to -1.11.

4. **"H+ DG" and "H- UG"** (fourth column) control the rotation of the end joint. The end joint is divided into "clamp" and "wrist" joints, depending on the end effector's configuration. When the product is powered on, the end joint will automatically rotate to the middle position, and the number on the button is 3.14.
   - **When in "gripper" joint mode**, the rotation range is 135°.
     - Pressing "H+ DG" will make the "gripper" joint grab, which is the default closed state.
     - Pressing "H- DG" will make the "gripper" joint open, with a maximum opening angle of 135°, and the value will change from 3.14 to 1.08.
   - **When in "wrist" joint mode**, the rotation range is 270°.
     - Keep clicking on "H+ DG", the "wrist" joint will rotate downward at 135°, and the value will change from 3.14 to 5.20.
     - Then, keep clicking on "H- DG", the "wrist" joint will rotate upward at 270°, and the value will change from 5.20 to 1.08.

**INIT:** Reset all joints to the middle position they were in when the product was powered on.

#### Torque: Torque Lock Control

- Clicking **"Torque OFF"** means the torque lock is off, then you can manually rotate the joints when the robotic arm is powered on.
- Clicking **"Torque ON"** means the torque lock is on, then you can't manually rotate the joints after the robotic arm is powered on.

> **Note:** If one of the joints or all the joints of the robotic arm receives other rotation commands after the torque lock is turned off, the torque lock will be turned on automatically.

#### DEFA: Dynamic External Force Adaptation Control

- Clicking **"DEFA ON"** means that the Dynamic External Force Adaptation (DEFA) function is enabled. When DEFA is turned on, if external forces cause the robotic arm to move, it will automatically return to its original position before the external force is applied.
- Clicking **"DEFA OFF"** to disable this function, means that the robotic arm will not automatically return to its original position when external forces are applied.

#### LED: LED Control

Clicking **"LED OFF"** to turn off LED lights, and clicking **"LED ON"** to turn on LEDs.

#### HORIZONTAL DRAG

Please refer to [Horizontal Drag Instructions](https://www.waveshare.com/wiki/Horizontal_Drag_Instructions) for more details.

#### VERTICAL DRAG

Please refer to [Vertical Drag Instructions](https://www.waveshare.com/wiki/Vertical_Drag_Instructions) for more details.

#### COORDCTRL: Robotic Arm End Joint End Point Coordinate Control

X, Y, and Z respectively represent the X axis, Y axis, and Z axis of the "Clamp/Wrist" EoAT (End of Arm Tooling) of the robotic arm. T represents the rotation angle of the EoAT (Different joints have different rotation angles, as you refer to "ANGLECTRL H+DG"), and all of these four parameters are adjusted by "+ -".

**INIT:** Reset all EoAT to the middle position they were in when the product was powered on.

#### FEEDBACK INFORMATION

Input JSON command to communicate with the robotic arm and the feedback of JSON command will be displayed here. The following is the commonly used JSON commands shortcut input of the robotic arm. Click the "INPUT" next to the command, it will be automatically input into the input box. For details about the control meaning of the JSON command, see [RoArm-M2-S JSON Command Meaning](https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning).

---

## Product Initialization

Users who need to quickly restore their product to the factory program can use the ESP32 download tool for RoArm-M2-S that we provide.

1. Click [RoArm-M2-S ESP32 download tool](https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2_FACTORY-260115.zip) to download, unzip it and double-click "flash_download_tool_3.9.5.exe" to open. Then, two windows pop up. The UI interface of the download tool is for operation, and the other window is the terminal to display the working status of the download tool.

2. In the "DOWNLOAD TOOL MODE" interface, select "Chip Type" as **ESP32**, and "WorkMode" as **Factory**, and the relative path will be used when calling the binary file, so you don't need to manually enter the binary file path, click OK.

3. Enter the "ESP32 FLASH DOWNLOAD TOOL", you can upload the demo for the eight robotic arms at the same time on the right. Connect the driver board on the robotic arm to the PC with a USB cable, and click on "COM" to select the new COM (here is "COM3"). "BAUD" is for setting the download speed, the higher the value, the faster the speed, and ESP32 can use up to 921600.

4. After the selection, click on "START" to start uploading the demo, after the upload is completed, "IDLE" will change to "FINISH". Then, the driver board can be disconnected from the USB connection with the computer. Insert the 12V 5A power cable into the 12V DC port of the robotic arm driver board, and then you can control RoArm-M2-S after powering on.

---

## Onboard Interfaces on General Driver for Robots

| No. | Resource Name | Description |
|-----|---------------|-------------|
| 1 | ESP32-WROOM-32 main control module | Can be developed using Arduino IDE |
| 2 | IPEX Gen 1 WIFI interface | Used for connecting the antenna with IPEX1 outer screw inner hole |
| 3 | Lidar Interface | Integrated the radar adapter board functionality |
| 4 | IIC peripheral expansion interface | Can be used to connect OLED screens or other IIC sensors |
| 5 | Reset button | Press and release to reboot the ESP32 |
| 6 | Download button (BOOT) | When pressed, it boots the ESP32 into download mode |
| 7 | DC-DC 5V Regulator circuit | Can power a host device such as a Raspberry Pi or Jetson Nano, etc. |
| 8 | Type-C port (LADAR) | LiDAR data interface |
| 9 | Type-C port (USB) | ESP32 serial communication interface, which can upload programs for ESP32 |
| 10 | XH2.54 Power connector | Inputs DC7~12.6V, can directly power the serial bus servos and motors |
| 11 | INA219 | Voltage/current monitoring chip |
| 12 | Power switch | Control external power supply ON/OFF |
| 13 | ST3215 Bus Servo Interface | Used to connect ST3215 serial bus servo |
| 14 | Motor interface PH2.0 6P | Group B interface for motor with encoder |
| 15 | Motor interface PH2.0 6P | Group A interface for motor with encoder |
| 16 | Motor interface PH2.0 2P | Group A interface for motor without encoder (LED lamp interface in this product) |
| 17 | Motor interface PH2.0 2P | Group B interface for motor without encoder |
| 18 | AK09918C | 3-axis electronic compass |
| 19 | QMI8658 | 6-axis motion sensor |
| 20 | TB6612FNG | Motor control chip |
| 21 | Serial bus servo control circuit | Connect multiple ST3215 bus servos and get servo feedback |
| 22 | TF card slot | Can be used to store logs or WIFI configurations |
| 23 | 40PIN expansion interface | Easy access to Raspberry Pi 4B, Raspberry Pi Zero or Jetson Orin Nano |
| 24 | 40PIN expansion interface | Convenient for using the pins on the host computer installed on the driver board |
| 25 | CP2102 chip | Serial port to USB, used for radar data transmission |
| 26 | CP2102 chip | Serial to USB, used for ESP32 serial communication |
| 27 | Automatic download circuit | Does not require pressing the EN and BOOT buttons when uploading code to ESP32 |

---

## RoArm-M2-S Tutorial Directory

### RoArm-M2-S ROS2 Humble + Moveit2 Tutorial

- [RoArm-M2-S ROS2 Humble + Moveit2 Tutorial](https://www.waveshare.com/wiki/RoArm-M2-S_ROS2_Humble_%2B_Moveit2_Tutorial)

### RoArm-M2-S User Tutorial

- RoArm-M2-S Web Usage
- [RoArm-M2-S Secondary Development Tool Usage](https://www.waveshare.com/wiki/RoArm-M2-S_Secondary_Development_Tool_Usage)
- [RoArm-M2-S JSON Command Meaning](https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning)
- [RoArm-M2-S WIFI Configuration](https://www.waveshare.com/wiki/RoArm-M2-S_WIFI_Configuration)
- [RoArm-M2-S Robotic Arm Control](https://www.waveshare.com/wiki/RoArm-M2-S_Robotic_Arm_Control)
- [RoArm-M2-S EoAT Setting](https://www.waveshare.com/wiki/RoArm-M2-S_EoAT_Setting)
- [RoArm-M2-S FLASH File System Operation](https://www.waveshare.com/wiki/RoArm-M2-S_FLASH_File_System_Operation)
- [RoArm-M2-S Step Recording and Reproduction](https://www.waveshare.com/wiki/RoArm-M2-S_Step_Recording_and_Reproduction)
- [RoArm-M2-S ESP-NOW Control](https://www.waveshare.com/wiki/RoArm-M2-S_ESP-NOW_Control)
- [RoArm-M2-S Python UART Communication](https://www.waveshare.com/wiki/RoArm-M2-S_Python_UART_Communication)
- [RoArm-M2-S Python HTTP Request Communication](https://www.waveshare.com/wiki/RoArm-M2-S_Python_HTTP_Request_Communication)

---

## Resources

### Robotic Arm Drawing

- [RoArm-M2-S related drawing](https://drive.google.com/file/d/1W3uj9fRvepdHmKwW1BALIXXCU5HevkVn/view)

### Robotic Arm STEP Model

- [RoArm-M2-S STEP Model](https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2-S_STEP.zip)

### Open-source Demo

- [RoArm-M2-S slave example](https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2_example20260115.zip)
- [RoArm-M2-S Python demo](https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2-S_python.zip)
- [RoArm-M2 series Github open source project](https://github.com/waveshareteam/roarm_m2)

### Software

- [ESP32 download Tool](https://files.waveshare.com/wiki/RoArm-M2-S/RoArm-M2_FACTORY-260115.zip)
- [Vertical and horizontal control tools](https://files.waveshare.com/wiki/common/Vertical%20and%20horizontal%20plane%20control%20tools.zip)

### Serial Port Driver

- [ESP32 CP2102 Serial Port Driver](https://files.waveshare.com/wiki/RoArm-M2-S/CP210x_USB_TO_UART.zip)

### Driver Board Schematic

- [Schematic](https://files.waveshare.com/wiki/RoArm-M2-S/General_Driver_for_Robots.pdf)

### VirtualBox ROS2 Image

- [VirtualBox ROS2 Image](https://drive.google.com/drive/folders/1ro-0LlyY9Z8aLXa7fL2Spb7qX5_6lYAP?usp=sharing) (System default password: ws)

### RoArm-M2-S ROS2 Package

- [RoArm-M2-S gripper ROS2 package](https://files.waveshare.com/wiki/RoArm-M2-S/Roarm_ws_em0.zip)
- [RoArm-M2-S wrist ROS2 package](https://files.waveshare.com/wiki/RoArm-M2-S/Roarm_ws_em1.zip)

### Project Resource

This section features third-party project resources. We merely provide links and bear no responsibility for content updates or maintenance.

**Kevin McAleer - 2 Robot Arms - what should I do?**
- [YouTube](https://www.youtube.com/watch?v=DQPC1Ev3jng)

---

## FAQ

### Question 1: When using serial communication, the robot arm does not respond to JSON commands or ROS2 control?

**Answer:**
First, check if the robotic arm is powered on. Refer to the power interface markings in the Product Introduction section for guidance. After plugging in the power interface, ensure that the power switch on the driver board is turned on. Next, check the onboard interfaces part of the General Driver for Robots to ensure that the robot arm is inserted into USB port number ⑨. Finally, verify that the serial port communication's port number is correct. If the control arm does not respond after the above checks, please consult technical support.

### Question 2: Why does the new COM port not appear when I upload the program to the General Driver for Robots using the download tool or Arduino IDE?

**Answer:**
First of all, check whether it is inserted into the USB port of the serial number ⑨ according to the onboard interfaces part of the General Driver for Robots, if yes, then look at the device manager of the computer whether there is a new COM port appearing; if not, check the other devices part if there is an unrecognized device named CP2102. If such a device is found, it's likely due to the lack of a driver. In that case, you can click to install the CP2102 serial port driver. If there is no unrecognized device with CP2102 in the name in other devices part, please contact the store's customer service to return it to the factory for repair.

### Question 3: When uploading a program to the General Driver for Robots using a download tool or the Arduino IDE, the terminal always shows the "Connecting..." interface, what should I do?

**Answer:**
In general, this situation occurs when the automatic download circuit of the General Driver for Robots can not work. At this time, you need to unplug the USB cable to upload again, when "...." appears during unload, press and hold the BOOT button on the General Driver for Robots, then press the EN key for one second and release the EN key, and finally release the BOOT key. If the upload is unsuccessful after multiple attempts, contact the store's customer service to return to the factory for repair.

### Question 4: After re-uploading the program to the product, I encounter password errors or other connection issues with Wi-Fi, how to solve the problem?

**Answer:**
After re-flashing the program, you need to clear the NVS area with `{"T":604}` command via serial communication, then power the robotic arm again, and reconnect to the WIFI.

---

## Support

**Technical Support**

If you need technical support or have any feedback/review, please click the [Submit Now](https://service.waveshare.com/) button to submit a ticket. Our support team will check and reply to you within 1 to 2 working days.

**Working Time:** 9 AM - 6 PM GMT+8 (Monday to Friday)
