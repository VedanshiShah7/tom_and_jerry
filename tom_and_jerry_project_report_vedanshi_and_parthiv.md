# Tom and Jerry: The Cheese Napping

---

## Tom and Jerry: The Cheese Napping  
**FINAL REPORT**  

Vedanshi Shah and Parthiv Ganguly  

---

### Introduction  

**Problem statement, including original objectives**  
Our project involves two robots playing a simplified game of tag. The robot being chased and moving towards the goal state (of the colored block) is assigned the role of "Jerry" while the secondary robot chasing takes on the role of "Tom." The Jerry bot must navigate through obstacles, find and pick up a colored block, and then make its way through more obstacles to reach the end wall, which is marked by a fiducial. The Tom robot's goal is to chase and “tag” Jerry by colliding with it or getting very close.  

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

There are different goal states/milestones. The robots are started at different set distances, there is around a 1m gap between the two robots. There is an obstacle section with the red brick walls as the obstacles. Finally, there is an end stage with the goal of reaching the pre-chosen colored block.  

We worked on getting the robots on the same `roscore` and then trying to figure out how to use the information for one to chase the other → what would be involved with that and decided to create the following functionality using the odoms of the robots.  

---

### Discussion of Interesting Algorithms, Modules, Techniques  

**Follow-the-Gap Method**  
Our first order of business for both the Tom and Jerry robots was writing code that allowed them to navigate to their goal coordinate while avoiding obstacles. One method we found online to do this was the Follow-the-Gap Method (FGM). This algorithm uses LIDAR data to find the widest gap between obstacles, and then follows that gap. This code worked well with point obstacles but did poorly with longer obstacles. So, we switched to a different algorithm which used gravity-based obstacle avoidance.  

**Gravity Obstacle Avoidance Algorithm**  
Our next obstacle avoidance algorithm was based on the “gravity/anti-gravity” method. The gravity algorithm makes the target coordinate have an attractive gravitational force, and it makes obstacles detected by LIDAR have a repulsive gravitational force. The robot then adds the gravity force vectors to get the movement vector of the robot. We started using this algorithm, but it didn’t do well in narrow corridors or when confronted with large surface obstacles. So, we decided to write our own algorithm for avoiding obstacles.  

**Custom Obstacle-Avoidance Algorithm**  
Finally, we created our own version of an obstacle avoidance method by combining the previous two algorithms. The robot divides the region around it into 7 different zones, each with an associated cost based on how far it is (in terms of angular difference) from the front zone. It then stores all the LIDAR values in each zone that are below a threshold. It then finds the most “obstacle-free” zone (i.e., the zone with no LIDAR values below the threshold or the farthest value) with the lowest cost. This is the zone that the robot should point towards to avoid the obstacle. To do so, the robot slightly rotates backwards to point in this new direction, and then continues following its goal.  

...  
