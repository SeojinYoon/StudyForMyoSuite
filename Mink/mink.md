
# References
- https://kevinzakka.github.io/mink/tutorial/mujoco_basics.html

# bb

mink uses mujoco as its physics backend.

- mink.update(): runs only mj_kinematics and mj_comPos, which suffice for IK.
- rigid body transformation: these classes support composition throught @
    - SE3 (poses)
    - SO3 (rotations)
- Configuration: Configuration wraps a MuJoCo model and provides the kinematic quantities needed for IK: frame poses, Jacobians, and integration. See MuJoCo Basics for details.
    - get_transform_frame_to_world(frame_name, frame_type): get transform matrix from local frame into world frame
- keyframe: is a snapshot of the simulation state variables


# Jacobian conventions

mink uses body velocities (twists expressed in the local frame) and body Jacobians throughout. This is a consistent choice: the Jacobian and velocity/error it maps to are in the same frame.

get_frame_jacobian() converts mujoco's local-world-aligned jacobian into a body jacobian local in pinocchio terminology.

```python
# mink returns a body Jacobian
jac = configuration.get_frame_jacobian("end_effector", "site")
```

mink's Configuration wraps mujoco's model and data, providing
- Poses in the world frame via get_transform_frame_to_world()
- Body Jacobians in the local frame via get_frame_jacobian()
- Inertial matrix via get_inertia_matrix()

All quantities are expressed in the body frame, with conversions from mujoco's conventions handled automatically
