# Tom and Jerry: The Cheese Napping

---

## Tom and Jerry: The Cheese Napping  
**FINAL REPORT**  

Vedanshi Shah and Parthiv Ganguly  

---

### Introduction  

**Problem statement, including original objectives**  

**Problem Statement:** Develop a system where two robots, Tom and Jerry, engage in a game of tag, with Jerry navigating obstacles and retrieving a colored block while Tom chases and attempts to tag Jerry.  

Our project centers on creating an interactive robotics game inspired by the classic dynamics of Tom and Jerry. In this game, the "Jerry" robot must navigate through an obstacle-filled environment to reach a preselected colored block at the goal. Meanwhile, the "Tom" robot is tasked with chasing and attempting to "tag" Jerry by either colliding or getting within close proximity. The key challenges include obstacle avoidance, multi-robot communication, and efficient target tracking.  

The original objectives were:  

1. Block detection  
2. Block pickup  
3. Navigating to the end wall while avoiding obstacles  
4. Programming the Tom robot for the chasing functionality  
5. Implementing obstacles  
6. Robot communication  

---

### Relevant Literature  

The following resources guided our design and implementation:  

- **Model creation:**  
  Techniques for setting up simulation environments and robot configurations  
  - [https://www.youtube.com/watch?v=YV8hlpBOhtw](https://www.youtube.com/watch?v=YV8hlpBOhtw)  
  - [https://www.youtube.com/watch?v=wUZO4wTvKCY](https://www.youtube.com/watch?v=wUZO4wTvKCY)  

- **Action lib:**  
  Implementing action clients for autonomous behaviors  
  - [http://wiki.ros.org/actionlib](http://wiki.ros.org/actionlib)  

- **HSV Color Detection:**  
  Understanding hue, saturation, and value for color-based object detection  
  - [https://learn.leighcotnoir.com/artspeak/elements-color/hue-value-saturation/](https://learn.leighcotnoir.com/artspeak/elements-color/hue-value-saturation/)  

- **Object and Color Detection:**  
  Real-time color recognition and object localization  
  - [https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/](https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/)  
  - [https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/](https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/)  

- **Follow-the-Gap Algorithm:**  
  - [https://www.sciencedirect.com/science/article/pii/S0921889012000838#](https://www.sciencedirect.com/science/article/pii/S0921889012000838#)  

- **Anti-Gravity Obstacle Avoidance:**  
  - [https://www.cse.chalmers.se/~bergert/robowiki-mirror/RoboWiki/robowiki.net/wiki/Anti-Gravity_Movement.html](https://www.cse.chalmers.se/~bergert/robowiki-mirror/RoboWiki/robowiki.net/wiki/Anti-Gravity_Movement.html)  

---

### What Was Created  

**Technical descriptions, illustrations**  

![My Image](https://drive.google.com/uc?export=view&id=11gb9T7If4yfC8p25sbf_QMP6bPsccACf)

There are different goal states/milestones. The robots are started at different set distances, there is around a 1m gap between the two robots. There is an obstacle section with the red brick walls as the obstacles. Finally, there is an end stage with the goal of reaching the pre-chosen colored block.  

![My Image](https://drive.google.com/uc?export=view&id=1Q4wLxMFRofTLzB84H0I3M42ZBmKquAO3)

We worked on getting the robots on the same `roscore` and then trying to figure out how to use the information for one to chase the other → what would be involved with that and decided to create the following functionality using the odoms of the robots.  

---

### Discussion of Interesting Algorithms, Modules, Techniques  

**Follow-the-Gap Method**  
Our first order of business for both the Tom and Jerry robots was writing code that allowed them to navigate to their goal coordinate while avoiding obstacles. One method we found online to do this was the Follow-the-Gap Method (FGM). This algorithm uses LIDAR data to find the widest gap between obstacles, and then follows that gap. This code worked well with point obstacles but did poorly with longer obstacles. So, we switched to a different algorithm which used gravity-based obstacle avoidance.  

**Gravity Obstacle Avoidance Algorithm**  
Our next obstacle avoidance algorithm was based on the “gravity/anti-gravity” method. The gravity algorithm makes the target coordinate have an attractive gravitational force, and it makes obstacles detected by LIDAR have a repulsive gravitational force. The robot then adds the gravity force vectors to get the movement vector of the robot. We started using this algorithm, but it didn’t do well in narrow corridors or when confronted with large surface obstacles. So, we decided to write our own algorithm for avoiding obstacles.  

**Custom Obstacle-Avoidance Algorithm**  
Finally, we created our own version of an obstacle avoidance method by combining the previous two algorithms. The robot divides the region around it into 7 different zones, each with an associated cost based on how far it is (in terms of angular difference) from the front zone. It then stores all the LIDAR values in each zone that are below a threshold. It then finds the most “obstacle-free” zone (i.e., the zone with no LIDAR values below the threshold or the farthest value) with the lowest cost. This is the zone that the robot should point towards to avoid the obstacle. To do so, the robot slightly rotates backwards to point in this new direction, and then continues following its goal.  

---

**Chasing Algorithm**  
For the chasing task, the class implements a chasing algorithm where the Tom robot continuously updates its position and yaw (orientation) and uses this information to calculate the distance and angle to the Jerry robot (the target). The `compute_angle_and_distance` method calculates these values, and Tom adjusts its velocity accordingly. If Tom is close enough to Jerry, it stops, simulating the capture of Jerry. This approach uses basic distance and angle calculations, ensuring that Tom efficiently chases Jerry while keeping its movement smooth by adjusting both linear and angular velocities.  

**Object Detection**  
For object detection, the robot uses both camera and LIDAR data to detect and approach objects of a specific color (like blocks - in red, yellow, blue, and green). The `detect_block` method processes images using OpenCV to find color-specific objects by segmenting the image in the HSV color space. Once an object is detected, the robot uses its LIDAR data to avoid obstacles while moving towards the object. The `smooth_center` method smooths the position of the detected block to reduce erratic movements, and the robot adjusts its speed and direction based on the block’s position relative to the camera’s center. If obstacles are detected via LIDAR, the robot halts or adjusts its path to avoid collisions. This combination of object detection and obstacle avoidance creates a robust system for navigating towards a target while ensuring safety.  

---

### Guide on How to Use the Code Written  

**RUN THE CODE (ASSUMES SETUP IS COMPLETE)**  

1. `rosrun tom_and_jerry tom_odom.py`  
2. `rosrun tom_and_jerry jerry_odom.py`  
3. `rosrun tom_and_jerry tom_chases_jerry.py`  

**SETUP - HOW TO ADD 2 ROBOTS ON THE SAME ROSCORE (IN REAL)**  

- **Access the VNC main window:**  
  Open the VNC main window for the simulation environment (`sim:sim`).  

- **Edit the `.bashrc` file:**  
  - In the terminal, type `nano .bashrc` to open the `.bashrc` file.  
  - Add the master robot VPN (e.g., bran2) in the real section:  
    `# $(bru name bran2 -m 100.100.231.32)`  
  - Save and exit the file (press `Ctrl+X`, then `Y`, and `Enter` to save).  

- **SSH into the Master Robot (bran2):**  
  - In a new terminal, SSH into the master robot:  
    `ssh ubuntu@100.100.231.32`  
  - Launch the ROS nodes:  
    `roslaunch sodacan bu.launch`  

- **SSH into the follower Robot (rafael):**  
  - In another terminal, SSH into the follower robot:  
    `ssh ubuntu@rafael_vpn`  
  - Update the `.bashrc` file inside rafael to reflect the master robot as bran2 in 2 places:  
    ```
    ROS_MASTER_URI=http://{vpn of bran2}:11311
    Bru name with the bran2 vpn
    ```  
  - Launch the necessary ROS nodes:  
    `roslaunch turtlebot3_bringup turtlebot3_multirobot.launch`  

- **Check ROS Topics:**  
  - In another terminal, run:  
    `rostopic list`  
  - Check the list of active topics to ensure you see both:  
    - `/rafael/cmd_vel`  
    - `cmd_vel for bran2`  

**OTHER DEBUGGING TIPS**  

- `printenv | grep ROS`  
  - Outputs: `ROS_MASTER_URI=http://{vpn of robot}:11311`  

- **To make a Python script executable in ROS:**  
  `chmod +x <script_name>.py`  

---
### Clear description and tables of source files, nodes, messages, actions and so on

## **1. Overview of Source Files**
| **File Name**                  | **Description**                                                                                         |
|--------------------------------|---------------------------------------------------------------------------------------------------------|
| `interface.py`                 | Flask-based web interface allowing users to interact with the system by selecting cheese colors and viewing game results. |
| `jerry_odom.py`                | Handles Jerry’s odometry, subscribes to `/odom`, and publishes processed odometry to `/jerry_odom`.     |
| `jerry_robot.py`               | Implements Jerry's behavior: navigation, LIDAR-based obstacle avoidance, and block detection using image processing. |
| `object_detection.py`          | Detects colored blocks using a camera feed, and provides movement commands to approach the detected block. |
| `tom_odom.py`                  | Similar to `jerry_odom.py`, but for Tom. Subscribes to `/rafael/odom` and publishes data to `/tom_odom`. |
| `tom_robot.py`                 | Implements Tom's behavior: chasing Jerry using positional data and basic pathfinding.                  |
| `tom_robot_w_obstacle_avoidance.py` | Enhances Tom's chasing behavior with LIDAR-based obstacle avoidance.                                      |

---

## **2. Nodes and Topics**

| **Node Name**         | **File**                          | **Published Topics**       | **Subscribed Topics**      | **Description**                                              |
|-----------------------|-----------------------------------|----------------------------|----------------------------|--------------------------------------------------------------|
| `/interface`          | `interface.py`                   | N/A                        | N/A                        | Flask server for web-based game interaction.                |
| `/jerry_odom`         | `jerry_odom.py`                  | `/jerry_odom`              | `/odom`                   | Processes and publishes Jerry's odometry data.              |
| `/jerry_robot`        | `jerry_robot.py`                 | `/cmd_vel`, `/bounding_box`| `/jerry_odom`, `/scan`    | Implements Jerry's navigation and block targeting behavior. |
| `/object_detection`   | `object_detection.py`            | `/bounding_box`, `/cmd_vel`| `/scan`, `/cv_camera/image_raw` | Detects colored blocks and moves the robot toward them.    |
| `/tom_odom`           | `tom_odom.py`                    | `/tom_odom`                | `/rafael/odom`            | Processes and publishes Tom's odometry data.                |
| `/tom_robot`          | `tom_robot.py`                   | `/rafael/cmd_vel`          | `/tom_odom`, `/jerry_odom`| Controls Tom's motion to chase Jerry.                       |
| `/tom_chaser`         | `tom_robot_w_obstacle_avoidance.py` | `/rafael/cmd_vel`          | `/rafael/scan`            | Adds obstacle avoidance to Tom’s behavior.                  |

---

## **3. Messages and Topics**

| **Topic**            | **Type**                        | **Publisher**            | **Subscriber**            | **Purpose**                                               |
|----------------------|---------------------------------|--------------------------|--------------------------|-----------------------------------------------------------|
| `/cmd_vel`           | `geometry_msgs/Twist`          | `jerry_robot.py`, `object_detection.py` | N/A                | Controls Jerry's movement.                               |
| `/rafael/cmd_vel`    | `geometry_msgs/Twist`          | `tom_robot.py`, `tom_robot_w_obstacle_avoidance.py` | N/A | Controls Tom's movement.                                |
| `/jerry_odom`        | `geometry_msgs/Point`          | `jerry_odom.py`          | `jerry_robot.py`, `tom_robot.py` | Provides Jerry’s processed odometry.                    |
| `/tom_odom`          | `geometry_msgs/Point`          | `tom_odom.py`            | `tom_robot.py`            | Provides Tom’s processed odometry.                       |
| `/bounding_box`      | `geometry_msgs/Point`          | `object_detection.py`, `jerry_robot.py` | N/A | Publishes the bounding box center of detected blocks.  |
| `/scan`              | `sensor_msgs/LaserScan`        | ROS LIDAR hardware       | `jerry_robot.py`, `tom_robot_w_obstacle_avoidance.py` | Provides LIDAR data for obstacle detection.             |
| `/cv_camera/image_raw`| `sensor_msgs/Image`           | ROS Camera hardware      | `object_detection.py`     | Provides images for detecting colored blocks.            |

---

## **4. Actions Performed**

| **Action**                        | **Initiated By**         | **Details**                                                                 |
|-----------------------------------|--------------------------|-----------------------------------------------------------------------------|
| **Navigate to Goal**              | `jerry_robot.py`         | Jerry moves toward a specified goal while avoiding obstacles.               |
| **Detect Colored Block**          | `object_detection.py`    | Detects blocks of a specific color using HSV filtering and moves toward them.|
| **Chase Jerry**                   | `tom_robot.py`, `tom_robot_w_obstacle_avoidance.py` | Tom chases Jerry based on positional data.                                  |
| **Obstacle Avoidance**            | `jerry_robot.py`, `tom_robot_w_obstacle_avoidance.py` | Detects obstacles using LIDAR and avoids them by adjusting path.            |
| **Web Interaction**               | `interface.py`           | Users interact via a web interface to select cheese colors and play the game.|

---
### Story of the Project  

**How It Unfolded, How the Team Worked Together**  

Our project involved two robots, and we decided to focus on one robot first. There were two major components to this robot's design: the first was navigating through a series of obstacles, and the second was using the camera to detect colored blocks and head towards the block with the chosen color. We split these tasks, with one of us working on navigating through the obstacles and reaching a goal coordinate, while the other worked on detecting the colored blocks and moving towards a specific colored block.

Once both parts were functional, we focused on writing the chasing code for the second robot and combining the two components for the first robot. The next step was implementing the chasing functionality, which turned out to be quite challenging. As a result, both of us worked together on the chasing algorithm. After much trial and error, and with help from TAs, classmates, and the professor, we finally developed a working chasing algorithm that relied on the odometry of both robots.

For the final run, we needed to integrate the obstacle avoidance code with the chasing robot and test the entire race. This also presented challenges, especially when deciding when the robot should switch states—specifically, when to transition from “obstacle avoidance” mode to “find colored block” mode.

---

### Problems That Were Solved, Pivots That Had to Be Taken  

**Inaccurate Odom**  
Our target navigation code works by calculating the vector between the robot’s current position, and the goal coordinate (in the robot’s frame). This requires the odom to accurately reflect the robot’s position from its origin, at every iteration of the main loop. Our initial plan was to use both branbots for Tom and Jerry. However, the odom on the branbots were quite inaccurate. When the robot’s odom thought it had reached the goal coordinate, it was actually around 10 to 20 degrees off. This problem worsened the farther the branbots had to travel.  

This problem worsened the farther the branbots had to travel.  

We figured out why this was happening. The odom on branbots used the IMU as part of a sensor fusion algorithm, and the data returned by the IMU is imperfect (possibly due to an issue with the magnetometer) which causes the odom to be inaccurate. Turtlebots didn’t have this issue, and had a much more accurate odom (still not perfect). However, we ultimately realized that it’s not that much of a big issue if the branbot is off from the goal coordinate. As long as it stops before the row of colored blocks, the block detection code will take over and lead the robot to the correct block.  

**Multiple Robots on the Same ROSCORE**  
Since the Tom robot needs to chase the Jerry robot, it means that both robots have to be on the same ROSCORE and share their odom/locations for the chasing code to work. Initially, we tried to follow some of the guides we found in the lab notebook. When those didn’t work we collaborated with the TAs, professor, and classmates who were also using multiple robots. Ultimately, we got the Rafael turtlebot to act as an auxiliary robot which connects to a “core” robot that runs the ROSCORE.  

**Chasing**  
After getting multiple robots on the same ROSCORE, it was now time to have Tom chase Jerry. First, we decided to follow the “Double Turtle Follow” homework and have both Tom and Jerry on the same TF tree where they shared a parent node. This would allow us to calculate a transform between their frames and Tom would follow that transform to chase Jerry.  

However, this approach didn’t work out. This is likely because the 2 different odoms are on 2 different TF trees and cannot be combined. So, we decided to do it manually. Instead of using the `tf2_ros` library, we get the 2 different odom coordinates, assume an initial offset between the 2 robots, and use that information to calculate the vector between Tom and Jerry. After some tweaking, this approach worked well enough for us.  

**Pivoting Away from BranArm**  
Our initial plan was to have the BranArm robot (i.e., the robot with the claw) be the Jerry robot. The Jerry robot would use the claw to pick up a colored block and carry it to the end of the obstacle course. However, there were many complications with the BranArm bot and we didn’t have enough time to solve all the problems. So, we had to use a branbot/turtlebot for Jerry.  

---

### Your Own Assessment  

Our project demonstrated the complexities and excitement of multi-robot interactions. While we achieved a functional system with Jerry navigating obstacles and Tom effectively chasing, there were several takeaways:  

- **The importance of modular design:** Breaking tasks into smaller, manageable components expedited debugging and integration.  
- **Adapting to constraints:** Switching hardware and rewriting algorithms were crucial pivots that kept the project on track.  
- **Collaboration:** By dividing responsibilities initially and converging efforts during challenges, we maximized productivity.  
- **End-to-End Debugging:** Given how many systems and modules (both hardware and software) could and did go wrong, we learned how to debug thoroughly, look for bugs, and narrow down seemingly vague problems.  

This project not only enhanced our technical skills in ROS, LIDAR, and computer vision but also reinforced the value of perseverance and teamwork in robotics development.  
