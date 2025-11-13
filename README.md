# Vision-based Pick-and-Place System

## Problem 1: Sampling-based Method (RRT) for C-Space Exploration and Goal-Finding
- Customized RRT to bias tree expansion toward the goal configuration while respecting obstacles in configuration space.
- Steering rule: for each random sample `q_rand`, compute `q_near = argmin ||q_rand - q||_1`; take a bounded step `q_new = q_near + δ · (q_goal - q_near) / ||q_goal - q_near||_1` with `δ = 10`.
- Example iterations:
  - From `q_start = ⟨90, 120⟩` toward `q_goal = ⟨70, 100⟩`: `||·||_1 = 40`, step `δ·(-20, -20)/40 = ⟨-5, -5⟩`, yielding `⟨85, 115⟩`.
  - Sampling `⟨200, 120⟩`: nearest `⟨90, 120⟩`, `||·||_1 = 110`, step `⟨10, 0⟩`, yielding `⟨100, 120⟩`.
  - Wrapping across the workspace boundary (`⟨360, 115⟩ ≡ ⟨0, 115⟩`), steer from `⟨85, 115⟩`: `||·||_1 = 85`, step `⟨-10, 0⟩`, yielding `⟨75, 115⟩`.
- Repeating this process produced a collision-free path that wraps around the toroidal angle dimension when advantageous.

## Problem 2 · Camera Pose via Forward Kinematics
The task for this assignment is to make the robot pick up all the objects in one bin and place them in the other:

For the written part of the assignment, we:
- Modeled each joint with a homogeneous transform `T_i(θ_i)` and multiplied them to obtain the end-effector pose `T_WE = Π_i T_i(θ_i)`.
- Applied the calibrated camera offset `T_EC` to derive the camera pose in the world frame: `T_WC = T_WE · T_EC`.
- The resulting `T_WC` provides both rotation and translation needed to project image detections into world coordinates for the pick-and-place pipeline.

Then we implemented the overall pick and place system in python. 
Pre-Step 1: we loaded robots and objects in the environment with the UR5PickEnvironment initialization function, given to us. 

Step 1: get the top-down camera observaton from env.observe(), it returns an RGB-D image
Step 2: perception module which estimates object pose using pose_est_segicp()
Step 3: compute grasp pose from object pose
Step 4: execute actual pick and place action:
- execute_grasp(): (it does primitive grasping)
- move_tool():
it computes the robot target configuration using Inverse Kinematics to return the joint angles and translation to get the necessary end effector pose


then in preception.py we implement a pose estimation algorithm 
