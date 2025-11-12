# Vision-based Pick-and-Place System #

---

## Problem 1: Sampling-based Method (RRT) for C-Space Exploration and Goal-Finding

In this problem, we implement a Rapidly-exploring Random Tree (RRT) to explore a **2D configuration space (C-space)** defined by the two revolute joint angles of a planar robot arm:
\[
\theta_1, \theta_2 \in [0, 360)
\]
The configuration space wraps around the boundary, forming a **torus** topology.  
Each node in the RRT corresponds to a joint configuration \( N = \langle \theta_1, \theta_2 \rangle \).

We use the **Manhattan (L₁) distance metric**:
\[
d_{L1}(A,B) = \min(|\theta_{1A}-\theta_{1B}|,\,360-|\,\theta_{1A}-\theta_{1B}|) + \min(|\theta_{2A}-\theta_{2B}|,\,360-|\,\theta_{2A}-\theta_{2B}|)
\]
and a **fixed step size of 10°** to generate new nodes.

RRT is chosen over PRM (Probabilistic Roadmap) since it efficiently **biases exploration toward the goal** while maintaining randomized coverage of the C-space.

### Iteration Overview

- **Iteration 1:**  
  - Sample \( N_{\text{rand}} = \langle 70, 100 \rangle \)  
  - Nearest node \( N_{\text{near}} = \langle 90, 120 \rangle \)  
  - Step vector = \( (-5, -5) \)  
  - New node \( N_{\text{new}} = \langle 85, 115 \rangle \)  
  - **Valid (free space)** ✅  

- **Iteration 2:**  
  - Sample \( N_{\text{rand}} = \langle 200, 120 \rangle \) (inside obstacle)  
  - Grow from \( \langle 90, 120 \rangle \) since it’s closer than \( \langle 85, 115 \rangle \)  
  - Step vector = \( (10, 0) \)  
  - New node \( N_{\text{new}} = \langle 100, 120 \rangle \)  
  - **Valid (free space)** ✅  

- **Iteration 3:**  
  - Sample \( N_{\text{rand}} = \langle 360, 115 \rangle \equiv \langle 0, 115 \rangle \)  
  - Wrap-around distance used (due to C-space torus)  
  - Nearest node \( N_{\text{near}} = \langle 85, 115 \rangle \)  
  - Step vector = \( (-10, 0) \)  
  - New node \( N_{\text{new}} = \langle 75, 115 \rangle \)  
  - **Valid (free space)** ✅  

Through these iterations, the RRT incrementally explores the configuration space while respecting both the obstacle region and the wrap-around boundary conditions.

---

## Problem 2: Kinematics

In this section, we analyze the robot’s **forward kinematics** to determine the camera’s pose in the world frame.  
Given that the camera is rigidly attached to the end-effector, we compute:
\[
{}^{w}\!T_c = {}^{w}\!T_1 \, {}^{1}\!T_c
\]
where each transformation encodes a rotation by the joint angle and a translation along the corresponding link length.

We use homogeneous transformation matrices to describe both joint rotations and link translations, allowing us to express the camera pose \( {}^{w}\!T_c \) and the observed object position in world coordinates \( {}^{w}\!P = {}^{w}\!T_c \, {}^{c}\!P \).

---