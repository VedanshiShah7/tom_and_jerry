# Tom and Jerry: The Cheese Napping

## FINAL REPORT

**Authors:** Vedanshi Shah and Parthiv Ganguly

---

### Introduction

#### Problem Statement

Our project involves two robots playing a simplified game of tag. The robot being chased and moving towards the goal state (of the colored block) is assigned the role of "Jerry," while the secondary robot chasing takes on the role of "Tom." The Jerry bot must navigate through obstacles, find and pick up a colored block, and then make its way through more obstacles to reach the end wall, which is marked by a fiducial. The Tom robot's goal is to chase and “tag” Jerry by colliding with it or getting very close.

**Key Objectives:**

1. Block detection
2. Block pickup
3. Navigating to the end wall while avoiding obstacles
4. Programming the Tom robot for the chasing functionality
5. Implementing obstacles
6. Robot communication

---

### Relevant Literature

The following resources guided our design and implementation:

- **Model Creation:** Techniques for setting up simulation environments and robot configurations
  - [YouTube Tutorial 1](https://www.youtube.com/watch?v=YV8hlpBOhtw)
  - [YouTube Tutorial 2](https://www.youtube.com/watch?v=wUZO4wTvKCY)

- **Action Lib:** Implementing action clients for autonomous behaviors
  - [ROS Actionlib Documentation](http://wiki.ros.org/actionlib)

- **HSV Color Detection:** Understanding hue, saturation, and value for color-based object detection
  - [HSV Color Theory](https://learn.leighcotnoir.com/artspeak/elements-color/hue-value-saturation/)

- **Object and Color Detection:** Real-time color recognition and object localization
  - [GeeksforGeeks Object Detection](https://www.geeksforgeeks.org/detect-an-object-with-opencv-python/)
  - [GeeksforGeeks Multiple Color Detection](https://www.geeksforgeeks.org/multiple-color-detection-in-real-time-using-python-opencv/)

- **Follow-the-Gap Algorithm:**
  - [Scientific Paper](https://www.sciencedirect.com/science/article/pii/S0921889012000838#)

- **Anti-Gravity Obstacle Avoidance:**
  - [RoboWiki Anti-Gravity Movement](https://www.cse.chalmers.se/~bergert/robowiki-mirror/RoboWiki/robowiki.net/wiki/Anti-Gravity_Movement.html)

---

### What Was Created

#### Technical Descriptions

The robots have different goal states/milestones. The robots start at set distances, approximately 1 meter apart. The environment includes:

1. Obstacle sections marked by red brick walls.
2. A goal area containing a pre-chosen colored block.

We synchronized the robots on the same `roscore` and used odometry data for Jerry to navigate to the block and for Tom to chase Jerry.

#### Discussion of Algorithms, Modules, and Techniques

##### **Follow-the-Gap Method**

- **Purpose:** Navigate to goal coordinates while avoiding obstacles.
- **Mechanism:** Utilizes LIDAR data to find the widest gap between obstacles and directs the robot to follow it.
- **Limitations:** Struggled with long obstacles, leading to the adoption of alternate methods.

##### **Gravity Obstacle Avoidance Algorithm**

- **Purpose:** Navigate using gravitational force vectors.
- **Mechanism:**
  - Target coordinates exert attractive forces.
  - Obstacles exert repulsive forces.
  - Forces are combined to calculate movement.
- **Limitations:** Performed poorly in narrow corridors or large-surface obstacles.

##### **Custom Obstacle-Avoidance Algorithm**

- **Purpose:** Enhance obstacle avoidance by combining features of the previous two algorithms.
- **Mechanism:**
  - Divides the area around the robot into seven zones.
  - Zones are assigned costs based on angular distance from the front.
  - The robot selects the most obstacle-free zone with the lowest cost, adjusts its orientation, and continues moving.

##### **Chasing Algorithm**

- **Purpose:** Allow Tom to chase Jerry effectively.
- **Mechanism:**
  - Calculates the distance and angle to Jerry using odometry data.
  - Adjusts linear and angular velocities based on proximity to Jerry.
  - Stops upon "capturing" Jerry.

##### **Object Detection**

- **Purpose:** Detect and approach color-specific objects (blocks).
- **Mechanism:**
  - Uses OpenCV to process images in HSV color space.
  - Combines camera and LIDAR data for safe navigation.
  - Implements smoothing functions to reduce erratic movements.

---

### Guide on How to Use the Code

1. **Run the Code:**
   ```bash
   rosrun tom_and_jerry tom_odom.py
   rosrun tom_and_jerry jerry_odom.py
   rosrun tom_and_jerry tom_chases_jerry.py
   ```

2. **Setup Robots on the Same `roscore`:**

   - **Edit `.bashrc`:**
     ```bash
     nano ~/.bashrc
     ```
     Add the master robot VPN details.

   - **SSH into Robots:**
     ```bash
     ssh ubuntu@<robot_ip>
     ```

   - **Launch Nodes:**
     ```bash
     roslaunch turtlebot3_bringup turtlebot3_multirobot.launch
     ```

3. **Debugging Tips:**
   - Use `rostopic list` to verify topics.
   - Make scripts executable:
     ```bash
     chmod +x <script_name>.py
     ```

---

### Story of the Project

#### How It Unfolded

- **Jerry Development:**
  - Parthiv focused on obstacle navigation.
  - Vedanshi worked on block detection and movement.

- **Tom Development:**
  - Initially focused on chasing using TF trees.
  - Switched to manual vector calculations for improved accuracy.

#### Problems Solved

1. **Inaccurate Odom:**
   - Identified inaccuracies in the `branbot` odometry.
   - Switched to `turtlebot` for better results.

2. **Multiple Robots on Same `roscore`:**
   - Collaborated with TAs and peers to integrate robots.

3. **Chasing Challenges:**
   - Pivoted from TF-based methods to manual calculations.

4. **BranArm Issues:**
   - Abandoned the use of BranArm due to hardware challenges.

---

### Assessment

- **Key Takeaways:**
  - Modular design expedited debugging.
  - Adaptability ensured progress despite constraints.
  - Collaboration maximized efficiency.
  - Debugging skills improved significantly.

- **Technical Gains:**
  - Enhanced skills in ROS, LIDAR, and computer vision.

- **Personal Growth:**
  - Learned perseverance and teamwork.
