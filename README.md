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


To run just these two algorithms, run python pose_based_pnp.py --use_state to just use the ground truth object poses. 


Next, to move beyond the ground truth object poses, we need to implement our pose estimation algorithm, which we do in preception.py.

in pose_est_segicp, we assume ground truth object segmentation is provided, skipping UNet training and inference. But we still need to get 
1) get the object point cloud from mask and depth. 2) get object model point cloud. 3) use ICP to align this two-point cloud and get the object pose

to test our pose estimatino algorithm, we can run python pose_based_pnp.py

the grasp success will be much lower with estimated pose instead of ground truth.

## Problem 3 · Visualization and Failure mode

The predicted pose estimation vs observed surface are not exactly aligned. This could be due to poor initialization. ICP relies on accurate correspondences and initial good alignment. If ICP is matching incorrect points, it could converge to a local minima and produce poor alignment results, like what we see in the screenshot. ICP could be matching incorrect points because the camera point cloud that ICP is corresponding to could be noisy or could be entirely missing part of the object. 

Bad results come from ICP's poor initialization, so there are a few ways to improve initial correspondence. 

First, in order to mitigate any potential of ICP forming incorrect point matches, we could add more cameras and improve the point cloud so that ICP has the full object pose to match to. 

Second, we could run an algorithm like RANSAC to use before ICP so that it produces a better initial pose and reduces the potential of ICP converging to a local minima.

Assumptions:

We didn't consider obstacles in our environment, but we have collision-detection algorithms like Rapidly-Exploring Random Tree which can help us explore an unknown space by incrementally searching the configuration space for a solution. It would make sure the robot never selects an action that would land us in the obstacle space. To deploy it, we would need to make sure we created a C-space and defined our obstacle vs free space. 

First, it never considered that the objects could be partially occluded, i.e. lighting was ideal. Second, it never considered that the object could not be rigid under the robot's grasp (strawberry could get mushy under grip). Third, it never considered that there may be a grasp pose that's not possible for the robot (i.e. outside of the robot's reach range). Fourth, it assumed that the robot's hardware execution was perfect. Fifth, the scene was clutter free and there was nothing in the way of the robot grasping one object (like another object).