# RRT

I developed a Rapidly-Exploring Random Tree (RRT) algorithm to plan and execute the collision-free path of a UR5 Robotic Arm with a Robotiq gripper. This project uses the Moveit! library for collision detection. It uses the ROS framework to communicate with a physical UR5 and Rviz to visualize a digital twin. The digital twin is able to reach inside the window of a Bugatti with a path planning time of about 30 seconds. This RRT algorithm was also used for pick and place tasks with the physical UR5 and Robotiq gripper.
