# LynxmotionAL5DKinematics

Coursework code for my Robotics MSc course using Matlab. Simulation of the LynxmotionAL5D Kinematics. Code specifications as per below.  

A. From material covered during lectures, lab exercises and examples, complete the following tasks:
1) Derive a DH representation of forward kinematics for the Lynxmotion arm1. 
Use MATLAB, lecture material and further reading. Include all your investigations and report this.

2) Analyse the workspace of the centre of the wrist (5thjoint) when each preceding joint moves through its 
range of motion and plot the 2D and 3D views of the workspace.

3) Derive the inverse kinematics model for the manipulator (analytical solution).

B. Complete the following:
1) Plan a task* in MATLAB withat least 5 positions. This process 
should give you at least 5 sets of Cartesian coordinates specifying the end-effector position and orientation in 3Dspace.

2) Solve the Inverse Kinematics for these positions in 3D space and obtain sets of Joint Coordinates. 
Create an appropriate plot/animation in MATLAB for the motion of the robot.

3) Implement 3 different trajectories between the Cartesian Points identified above and create an 
appropriate plot to demonstrate them):
a.Implement a free motion between the points
b.Implement a straight line trajectory between thepoints
c.Set an obstacle between any two points (e.g. a cylinder between point 3 and 4) and implement an object avoidance trajectory.

C. Develop a kinematic simulation of the parallel robot in MATLAB to:
1) Solve and implement parallel robot Inverse Kinematics. Calculate the joint coordinates theta_i where i=1:3 from the 
Cartesian parameters of the Platform's centre {C}. These parameters are the (Xc, Yc) coordinates and the orientation of the platform (a). 
2) Plot the Parallel robots workspace for a given orientation a. This is crucial for the mechanism synthesis analysis.
