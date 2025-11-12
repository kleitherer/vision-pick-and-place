# Vision-based Pick-and-Place System

## Problem 1: Sampling-based Method (RRT) for C-Space Exploration and Goal-Finding

We run RRT in **configuration space** (C-space) with wrapping:
- Joint ranges: \( \theta_1, \theta_2 \in [0, 360) \) (torus topology)
- Distance metric: Manhattan \(L_1\) with wrap per joint  
  \( d_{\text{circ}}(\alpha,\beta)=\min(|\alpha-\beta|,\,360-|\,\alpha-\beta\,|) \)  
  \( d^{\text{torus}}_{L1}((\theta_1,\theta_2),(\phi_1,\phi_2))=d_{\text{circ}}(\theta_1,\phi_1)+d_{\text{circ}}(\theta_2,\phi_2) \)
- Step size: **10°**

### Iteration Overview
- **Iter 1:** sample \(N_{\text{rand}}=\) ⟨70, 100⟩; grow from ⟨90, 120⟩.  
  \(d_{L1}=|{-20}|+|{-20}|=40\), step \(=10\cdot(-20/40,-20/40)=(-5,-5)\) → **⟨85, 115⟩** (valid)
- **Iter 2:** sample \(N_{\text{rand}}=\) ⟨200, 120⟩ (inside obstacle); nearest is ⟨90, 120⟩.  
  \(d_{L1}=110\), step \(=(10,0)\) → **⟨100, 120⟩** (valid)
- **Iter 3 (wrap):** sample \(N_{\text{rand}}=\) ⟨360, 115⟩ ≡ ⟨0, 115⟩.  
  Distances: from ⟨85,115⟩ → \(85\); ⟨90,120⟩ → \(95\); ⟨100,120⟩ → \(105\).  
  Nearest ⟨85,115⟩; step \(=10\cdot(-85/85,0)=(-10,0)\) → **⟨75, 115⟩** (valid)

## Problem 2: Kinematics

Camera pose in world frame:
$$ {}^{w}\!T_c = {}^{w}\!T_1 \, {}^{1}\!T_c $$

With standard planar 2R (no \(d\), \(\alpha=0\)):
$$
{}^{w}\!T_{1} =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & a_1\cos\theta_1\\
\sin\theta_1 & \cos\theta_1  & 0 & a_1\sin\theta_1\\
0&0&1&0\\
0&0&0&1
\end{bmatrix},
\qquad
{}^{1}\!T_{c} =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & a_2\cos\theta_2\\
\sin\theta_2 & \cos\theta_2  & 0 & a_2\sin\theta_2\\
0&0&1&0\\
0&0&0&1
\end{bmatrix}.
$$

World point from camera point \( {}^{c}\!P \):
$$ {}^{w}\!P = {}^{w}\!T_c \, {}^{c}\!P. $$