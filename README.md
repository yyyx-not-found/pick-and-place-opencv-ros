# Pick and Place with OpenCV and ROS

This project demonstrates a pick-and-place task using a robotic arm, computer vision with OpenCV, and forward/inverse kinematics in ROS. It was developed as part of a robotics course project.

## Project Description

- **Computer Vision**: Use OpenCV to detect and find the centroids of colored blocks.
- **Pose Estimation**: Determine each block's position and orientation in the world frame (xw, yw, ωw).
- **Motion Planning**: Compute the goal joint angles using inverse kinematics.
- **Pick and Place**: Move each block and align them neatly along the world x-axis.

Blocks are made by fastening three colored cubes together to create a larger object with meaningful orientation.

## Technologies Used

- Python
- OpenCV
- ROS (Robot Operating System)
- Forward and Inverse Kinematics

## Results

- Successfully detect and locate at least two blocks.
- Pick and place the blocks aligned to the world frame x-axis.
- Maintain consistent block orientation (no flipping color-wise).

## References

- HSV Color Space: [Wikipedia](https://en.wikipedia.org/wiki/HSL_and_HSV)
- Blob Detection: [LearnOpenCV](https://www.learnopencv.com/blob-detection-using-opencv-python-c/)

## Disclaimer

This project is for educational and demonstration purposes only.  
**Do not use this project for assignments or academic submissions.**
