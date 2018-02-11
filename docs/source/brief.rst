Project Brief
=============

The goal of this project is to program the FRANKA Emika robotic arm (nicknamed “Panda”) to prepare some breakfast and feed a person. The outcome of the project will be demonstrated by performing the whole scenario fully autonomously, including the perception of the person, the person’s mouth, etc. For inspiration, please watch this video:
https://www.youtube.com/watch?v=E2evC2xTNWg

Technical Tools
^^^^^^^^^^^^^^^

Forward and Inverse Kinematics: To use the 7-dof kinematic model for Cartesian-to-Joint space mapping.
Redundancy Resolution: To resolve the 7-dof arm configuration while following a 6-dof hand pose trajectory.
Perception: Adding additional sensor for perceiving the person and his mouth. It could be an RGB-D camera (Kinect or Xtion) with skeleton tracking, or a face detecting high-speed webcam, for example.
Face detection and recognition: To identify the person for feeding.
Mouth tracking: To avoid problems when the person’s head is moving.
Motion Planning: Using OMPL/MoveIt or other motion planning libraries to generate viable collision-free trajectories for executing the movements.
Motion Control: To tune a Cartesian impedance controller or other for fast and smooth motion of the arm.
Hand control: To control the grasping with the 2 fingers of the hand.
Bonus points: For putting a baby bib on the person!

Equipment
^^^^^^^^^
FRANKA Emika (Panda), RGB-D camera (Kinect or Xtion), high-speed webcam

