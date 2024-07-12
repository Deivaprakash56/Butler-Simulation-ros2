 BUTLER ROBOT SIMULATION
 
![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/c52e89b4-22a2-47bc-84a0-98c9d96bf88f)

## Description :
  * This repository contains the code for a butler simulation robot in ROS2. The project aims to simulate the butler robot to receive and deliver orders from the table autonomously. The simulation is built using various tools such as Gazebo and RViz.

    

**Problem Statement**:
* To design and simulate a butler robot that navigates through the restaurant to receive orders and to deliver the food autonomously.

**Solution approached:**
*  To select a suitable simulation platform,
*  To create the environment which ressembles to a restaurant/hotel for this problem statement,
*  Selection of robot for replacing the human Butler,
*  Loading the selected butler robot into the created custom environment,
*  To map the entire custom environment using the butler robot,
*  To navigate the robot in our environment ,
*  Finally to implement the logic
*  The logic for the butler robot       
* To deliver the food from kitchen to the specific ordered table in the preference in which the orders are received.


**Selection of simulator**: 

**Gazebo:** 

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/c10ed870-2f90-4e8f-ab28-e532eec68dfd)

- Gazebo is an open-source 3D robotics simulator.
- With Gazebo, we can create a virtual “world”, and load simulated versions of our robots into it.

**Rviz:(ROS visualisation**)

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/52461cb0-22cc-4a59-8867-322bf228b912)

- Rviz is a 3D visualisation software tool for robots.
- The purpose of Rviz is to enable you to visualise the state of a robot.

**Environment Creation(World):**

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/43ff074f-3392-4065-a000-46f77c0e9b8e)

- For creating an environment, I just imagined a restaurant setup in my mind and roughly sketched on a piece of paper.

**Our customised environment (world file) :**

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/627b1bf6-727e-4bf3-af10-f7bfdde5c154)

- Created a basic Layout structure in **Gazebo simulator** with the reference of my basic sketch.

**Selection of a robot for Butler purpose:**

- As per my assigned task,I have selected Turtlebot 3 Robot.
- Since it has the largest community service for better troubleshooting purposes.

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/1f1b4d09-2735-4b92-8f28-f1bdc1a228a4)


**Loading the robot into our environment :**

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/79b328b0-4e4b-4be0-baa7-db4b4c3cef7d)


- For Loading the turtlebot 3 robot , we can clone the necessary executables from the official github page of turtlebot.
- Then we can choose the model type of turtlebot and load them into our gazebo world.

**Mapping the environment :** 

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/cd007310-695f-4ebe-a1bd-e3f3b7d20f68)

- For mapping an environment , there are several tools such as gmapping ,SLAM toolbox,etc…
- I have chosen the **CARTOGRAPHER** tool for mapping the world. Which provides real-time **Simultaneous localization and mapping (SLAM)** in 2D and 3D across.

**Navigation of the robot in our environment :**

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/7bfffd19-a8aa-4604-af4c-7a608dffd6ff)

- For navigating the loaded robot in our environment , we can use the navigation 2 package by loading our map into it.
- For this purpose , **Rviz (Ros/Robot Visualizer)** is used.

**Implementation as per the requirement :** 

- In Rviz , we can provide the destination point of the robot to navigate autonomously that is **Nav2 goal.**
- Even for complete autonomous navigation of several destination points , we can use **waypoint/nav through poses mode.**
- We have utilized Simple commmander for programmatically implementing this task.


**Implementing logics for each criteria:**

**Criteria 1 (Single order) and criteria 5(Multiple orders) :**

- When orders received with table number, 
- Robot\_Homeposition → kitchen → Table for food delivery. (Requires no confirmation)

  ![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/0482997d-1f3e-4563-a662-1dc1c38514ea)


**Criteria 2(Single order):**

- When orders received with table number, 
- Robot\_Homeposition → kitchen → Table for food delivery. (Requires confirmation)

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/22e0ec9d-58cb-4437-b4d5-adc475593cd8)


**Criteria 3(Single order) and criteria 6(Multiple orders) :**

- Robo\_hp → kitchen (no confirmation) → Return back to robo\_hp.
- Robo\_hp → kitchen (confirmation) →Table (confirmed) → robo\_hp.
- Robo\_hp → kitchen (confirmation) →Table (no confirmed) → kitchen →robo\_hp.

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/ab56a469-0354-40a6-bca1-f4a1886e1f51)


**Criteria 4(Single order) and 7(multiple orders) :**

- Order placed →Robo\_hp → kitchen→**Cancelled** →return to robo\_hp.
- Order placed →Robo\_hp →kitchen →Table of order →**Cancelled** →kitchen →robo\_hp.

![image](https://github.com/Deivaprakash56/ROS2_Project/assets/110375140/83ed74ca-0d91-4fc8-84e3-b4e57d5986d4)







