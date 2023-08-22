# raceteam_softwareHW_2023
In the race team, many scenarios require the communication of different system parts, that is when the ROS2 nodes come into play. For instance, our system needs to detect how much the accelerator pedal is pressed and convert it into a signal which will determine the motor output power.
## Scenario Description
- A stroke sensor installed on the accelerator will detect how much the pedal is pressed and return data within 0~4095 according to the pressed depth (larger is deeper).
    A ROS2 bag file containing pedal stroke data within a period of time is provided in ***/rosbag2_2023_08_22-09_00_33_0*** inside the given Github repository.
- A Node called **motor_node** which represents the motor will:
    - Read the PWM signal within 0~255 published under the **pwm_signal** topic.
        > - When the sent signal is 0, there will be 0 volts going through the motor.
        > - When the sent signal is 255, the motor will run under its maximum voltage (500V for example). 
    - Spit out the rotational speed with the unit of revolution per second (revs/s) by publishing it under the ***motor_revs*** topic.

    The ***motor_node*** is provided at ***/src/motor_node.cpp*** in the Github repository.
    Note that there are *no direct relations* between the given voltage and motor rotational speed.

- The principle of feeding PWM signal to the motor according to the pedal stroke value is linear. For example, if the pedal stroke is 2047, you should send a PWM signal with the value of 127 to the motor.

## Your mission
- Write a ROS2 Cpp node called ***stroke_to_pwm_node*** listening to the ***pedal_stroke*** topic and send the corresponding PWM signal to the motor by publishing it under the ***pwm_signal*** topic.

- Use ROS2 bag to record the motor rotational speed published under the ***motor_revs*** topic. And place the file under **/bag_files** directory in your GitHub repository.
    
- (Bonus) Make a screenshot of the ***rqt graph*** while running this project, which will help you understand the node structure better. And append the **.png** image file in your GitHub repository.

Your Github repository should look like this:
```bash
a_directory_in_your_raceteam_HW_repository
├── README (your answer to Part 1)
├── CMakeLists.txt
├── package.xml
├── bag_files
│   └── rosbag2_2023_08_22-09_00_33 (the provided one containing pedal stroke data)
│   └── rosbag2_******** (the bag file you record)
├── src
│   └── motor_node.cpp (the provided node)
│   └── stroke_to_pwm_node.cpp (the node you need to write)
│   └── doxyconfig (doxygen related things)
└── image.png (the rqt graph bonus image)
```
