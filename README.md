# common_arduino_application

```
.
├── CMakeLists.txt
├── ino
│   ├── accelerometer-_gyroscope_GY-521_node
│   │   └── accelerometer-_gyroscope_GY-521_node.ino
│   ├── Arduino_Multiple_Read_Write_I2C
│   │   └── Arduino_Multiple_Read_Write_I2C.ino
│   ├── ax12_blink
│   │   └── ax12_blink.ino
│   ├── dc_motor_control_mdd10
│   │   └── dc_motor_control_mdd10.ino
│   ├── dc_motor_control_mdd10_encoder_ax12_node
│   │   └── dc_motor_control_mdd10_encoder_ax12_node.ino
│   ├── dc_motor_control_mdd10_encoder_node
│   │   └── dc_motor_control_mdd10_encoder_node.ino
│   ├── dc_motor_control_mdd10_node
│   │   └── dc_motor_control_mdd10_node.ino
│   ├── encoder_spg30e
│   │   └── encoder_spg30e.ino
│   ├── ROS-node-control-LED
│   │   └── ROS-node-control-LED.ino
│   ├── ROS_node_for_self_collect_machine
│   │   └── ROS_node_for_self_collect_machine.ino
│   ├── ROS_node_for_self_collect_machine_rev1
│   │   └── ROS_node_for_self_collect_machine_rev1.ino
│   ├── ROS_node_for_self_collect_machine_rev2
│   │   └── ROS_node_for_self_collect_machine_rev2.ino
│   ├── ROS-node-generate-random-number
│   │   └── ROS-node-generate-random-number.ino
│   ├── ROS_node_that_controls_the_LED
│   │   └── ROS_node_that_controls_the_LED.ino
│   ├── ROS_node_to_control_boxID
│   │   └── ROS_node_to_control_boxID.ino
│   ├── ROS_node_to_generate_a_random_number
│   │   └── ROS_node_to_generate_a_random_number.ino
│   ├── ROS_node_to_push_sensor_state
│   │   └── ROS_node_to_push_sensor_state.ino
│   ├── servo_motor_control_ax12_node
│   │   └── servo_motor_control_ax12_node.ino
│   └── servo_motor_control_ax12_with_encoder_node
│       └── servo_motor_control_ax12_with_encoder_node.ino
├── installArduinoIDE.sh
├── launch
│   ├── dc_motor_control.launch
│   ├── dc_motor_robot1_control.launch
│   ├── dc_motor_robot2_control.launch
│   ├── multiple_arduinoSBC.launch
│   ├── robot1_motor_control.launch
│   ├── robot2_motor_control.launch
│   ├── roll_pitch_yaw.launch
│   ├── servo_motor_control.launch
│   ├── servo_motor_robot1_control.launch
│   ├── servo_motor_robot2_control.launch
│   └── twoArduino_serial.launch
├── package.xml
├── README.md
└── script
    ├── display_calculate_LED.py
    ├── display_calculate_LED_stage.py
    └── rpy_reading.py

```

**Using an Arduino to communicate through ROS; *rosserial*. Each of the Arduino files (ino) required to be downloaded to the Arduino before rosserial can be used**

**Each of ino file(s) serve different purpose**

**NOTES**

Please refer to these pages:
1. http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
2. http://wiki.ros.org/rosserial
