# Robotic Handpose Estimation: Simulated Handwriting with a SynGrasp Hand Model

A MATLAB simulation of a paradigmatic human hand gripping a virtual pen and reproducing a user-drawn, freehand trajectory, framed as an inverse kinematics problem solved frame by frame via constrained optimization.

## Overview

Given any 2D doodle drawn by hand, this project animates a kinematic hand model holding a pen and tracing that exact path, computing the finger joint angles needed at every frame to keep a tight three-point grip on the pen while its tip follows the trajectory and its shaft stays correctly oriented toward the wrist.

The hand itself is built with [SynGrasp](http://syngrasp.dii.unisi.it), an open-source MATLAB toolbox for modeling and analyzing grasping in robotic and human hands (Malvezzi, Gioioso, Salvietti and Prattichizzo, "SynGrasp: A MATLAB Toolbox for Underactuated and Compliant Hands," IEEE Robotics and Automation Magazine, 2015). This project uses SynGrasp's hand kinematics and contact-modeling functions as the underlying model, and adds the trajectory-tracking IK solver, grip calibration, and animation on top.

## How it works

**1. Hand and grip setup.** A SynGrasp paradigmatic hand (`SGparadigmatic`) is placed into a human-writing configuration (`SGhumanWritingConf`), and three contact points are defined for the grip: two on the thumb and one on a second finger (`SGaddFtipContact`, `SGaddContact`). A virtual object is generated at these contacts to represent the pen (`SGmakeObject`).

**2. Trajectory input.** The path to trace is captured interactively: the user free-draws a shape on a 2D canvas (`drawfreehand`), and the raw stroke is resampled into 200 evenly-spaced points by arc length using piecewise cubic interpolation, so the hand moves at a constant speed along the path regardless of how it was originally drawn.

**3. Grip calibration.** Before solving any motion, the three contact points' "tight grip" offsets around the pen shaft are computed once from the hand's home pose, so the pen radius stays consistent throughout the animation instead of drifting frame to frame.

**4. Per-frame inverse kinematics.** For each of the 200 trajectory frames, the target pen-tip position is known (from the trajectory) and the target pen orientation is computed by rotating the pen's home axis toward the vector connecting the tip to a fixed wrist pivot, using a Rodrigues rotation. This keeps the pen's tilt physically consistent as it moves across the page, the way a hand naturally reorients a pen while writing. The three finger contact points implied by that pose become the IK target.

Solving for the finger joint angles is framed as a constrained optimization problem rather than a closed-form solve: the cost function (`constrained_solver_step`) penalizes both the squared distance between the current and target contact points, and any violation of per-joint angle limits, and it's minimized frame by frame with `fminsearch`, MATLAB's derivative-free Nelder-Mead solver.

**5. Animation.** As each frame's joint angles are solved, the hand and a rendered pen cylinder are redrawn along the traced path, showing the full motion build up stroke by stroke.

## Files

| File | Role |
|---|---|
| `hand_model.m` | Builds the SynGrasp paradigmatic hand, sets the writing configuration, and defines the three-point pen grip and object |
| `get_traj.m` | Captures a freehand 2D drawing from the user and resamples it into an evenly-spaced 200-point trajectory |
| `kinematics.m` | Core IK solver: calibrates the grip, computes per-frame target contact points and pen orientation, solves joint angles via constrained `fminsearch`, and animates the result |

## Usage

Requires MATLAB with the [SynGrasp toolbox](http://syngrasp.dii.unisi.it) installed and on the MATLAB path.

Run the three scripts in order, in the same MATLAB session (each later script depends on variables left in the workspace by the one before it):

```matlab
hand_model   % builds `hand` and the pen `object`
get_traj     % draw a shape when prompted; builds `trajectory`
kinematics   % solves and animates the hand tracing `trajectory`
```

When `get_traj` runs, a blank canvas will open. Draw any continuous shape, then press Enter in the command window to finish. `kinematics` will then solve and animate the hand writing that shape.

## Reference

Malvezzi, M., Gioioso, G., Salvietti, G. and Prattichizzo, D. (2015). "SynGrasp: A MATLAB Toolbox for Underactuated and Compliant Hands." *IEEE Robotics and Automation Magazine*, 22(4), pp. 52 to 68.
