
# Singularity

A point where the robot loses one or more degrees of freedom (DOF) because of the arrangement of joint angles. At this point, the robot cannot move in a certain direction (e.g., trying to extend further when the arm is already fully straightened).

# Jacobian Matrix

In robot control, the Jacobian maps joint velocities to end-effector velocities:$$v = J \dot{q}$$  (Note: $v$ is linear/angular velocity of the end-effector, and $\dot{q}$ is the velocity of joint angles.)

Since we usually want to know how much to rotate the joints to achieve a desired end-effector movement:$$\dot{q} = J^{-1} v$$

So, we have to calculate the inverse matrix $J^{-1}$.


## Ill-conditioned Jacobian

Mathematically, Ill-conditioned means the matrix is highly sensitive to small changes, making it numerically unstable to find an inverse. When the robot approaches a singularity, the determinant of the Jacobian matrix approaches zero, causing the following problems:
- Explosion: Since calculating the inverse $J^{-1}$ involves dividing by the determinant, as the determinant gets closer to zero, the values in the inverse matrix explode toward infinity.
- Unbounded Velocity: To move the end-effector even a tiny distance (e.g., 1mm), the mathematical result ($\dot{q} = J^{-1}v$) demands that the joint angles rotate at physically impossible, "unbounded" speeds.
- Inability to Control: Because a real robot has physical limits and cannot rotate joints at such extreme speeds, the control system will suffer from severe vibrations (oscillation), produce errors, or trigger an emergency stop.
