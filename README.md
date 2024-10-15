# Tom and Jerry: The Cheese Napping
### Robotics Team Project Proposal

**Team Members:** Vedanshi Shah and Parthiv Ganguly

### Presentation
[Tom and Jerry: The Cheese Napping Proposal Presentation](https://docs.google.com/presentation/d/1jFoUX0UprEowCa0RUbwj3xY7Qs-4O8po43Qch4jqzwg/edit?usp=sharing)

### Report
[Project Report](https://docs.google.com/document/d/1C8kbmTvtKslRKZb1uP1KxQr6wsdHUNbl6zCn2M3lNq0/edit?usp=sharing)

## Description of the Project
Our project involves two robots playing a simplified game of tag. The robot with the claw is assigned the role of "Jerry," while the other robot takes on the role of "Tom." The Jerry bot must navigate through obstacles, find and pick up a colored block, and then make its way through more obstacles to reach the end wall, which is marked by a fiducial. The Tom robot's goal is to chase and “tag” Jerry by colliding with it or getting very close.

### Starting Condition
The Tom robot starts 0.7 meters behind the Jerry robot. In front of Jerry is an arrangement of walls acting as obstacles. After these obstacles is a row of colored blocks. Beyond that row are additional obstacles, and then the end wall, which is marked by a fiducial. The color of the block to be picked up by Jerry is selected via a website. Once the color is chosen, the chase begins.

- **Jerry:** Must navigate through the obstacles to reach the row of colored blocks, identify the correct block, approach it, and use its claw to grab it. After securing the block, it moves toward the end wall while avoiding obstacles.
  
- **Tom:** Moves toward Jerry as quickly as possible, aiming to collide with it to “tag” it.

### Game Outcomes
- **Jerry Wins:** If Jerry reaches the end wall with the block and without getting tagged, it wins. Both robots stop moving.
  
- **Tom Wins:** If Tom tags Jerry before it reaches the end wall, Tom wins. Both robots stop moving.

## How Will You Demonstrate It?
The robots are in their starting positions, as described above. The user opens the website GUI, selects a color, and clicks "run."

- Jerry navigates around the obstacles toward the row of blocks, while Tom follows closely.
- If Jerry reaches the row of blocks without being tagged, it approaches the selected color and uses its arm to pick up that block. After securing the block, Jerry heads toward the end wall while continuing to evade Tom.
- If Jerry approaches the end wall without getting tagged, it drops the block and wins. The website updates to reflect this outcome.
- If Tom tags Jerry before it reaches the end wall, Tom wins. If Jerry is holding a block, it drops it. The website updates to reflect this outcome.
- Afterward, we reposition both robots to their starting positions.

### Major Risks and Testing Plans
#### Risk 1 - Using the Claw
- Ensure the claw works correctly for basic functions.
- Test claw positioning and gripping strength.

#### Risk 2 - Object Detection
- Implement object recognition for colored blocks.
- Test detection accuracy and make adjustments as needed.

#### Risk 3 - Chasing and Tagging
- Ensure reliable coordinate sharing between robots for tagging.

#### Risk 4 - Jerry's Speed
- Ensure Jerry can pick up blocks quickly enough to avoid being tagged.

## Robots and Props Needed
- **Robots:** 2 Turtlebots, with 1 equipped with a claw/arm for picking up blocks.
- **Props:** Colored blocks (4 or 5 colors), obstacles (walls/barriers), and 1 fiducial to mark the end wall.

### Additional Optional Challenges
- Implement strategies for the chased robot (Jerry) to evade Tom.
- Ensure robots return to starting positions after the game ends.
- Have Jerry remember and return the block to its original location after the game.

## Testing Plan
1. Attach the claw and ensure it is operational.
2. Implement block detection using the robot’s camera.
3. Ensure Jerry can pick up the block and navigate towards the end wall.
4. Program Tom to chase Jerry accurately.
5. Test the complete game flow and adjust as necessary.

### Decision on Tagging Method
We plan to go ahead with sharing coordinates as it is a more reliable and accurate method.

### Table: Comparing Different Techniques for Chasing and Tagging

| Criteria                   | Sharing Coordinates | Fiducials           | Proximity Detection     |
|----------------------------|---------------------|----------------------|--------------------------|
| **Reliability**            | High                | Medium               | Medium                   |
| **Ease of Implementation** | High                | Medium               | Low                      |
| **Accuracy**               | High                | High                 | Medium                   |
| **Cost/Resource**          | Low                 | Medium               | Medium-High              |
| **Physical Constraints**    | Low                 | Low                  | Low                      |
| **Potential Issues**       | Odom inaccuracies    | Visual occlusion     | Environmental interference|

