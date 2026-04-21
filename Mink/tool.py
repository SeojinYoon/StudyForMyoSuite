
import mujoco
import numpy as np
import mediapy as media
from mink import Configuration, SE3, SO3, FrameTask, solve_ik

def do_IK(configuration, 
                  tasks,
                  renderer, 
                  n_frames,
                  dt,
                  target_info,
                  is_render = True,
                  solver_name = "quadprog",
                  limits = [],
                  rendering_fps = 60,
                  is_clear_pos = True,
                  static_pos_name = "home"):
    if is_clear_pos:
        configuration.update_from_keyframe(static_pos_name)

    frames = []
    for frame_i in range(0, n_frames):

        for task in tasks:
            if type(task) is FrameTask:
                # Set target
                for target in target_info:
                    target_val = target_info[target][frame_i] # [w_o, x_o, y_o, z_o ,x,y,z]
                    new_target = SE3(wxyz_xyz = target_val)

                    task.set_target(new_target)
                    configuration.data.mocap_pos[0] = new_target.wxyz_xyz[4:]
                    configuration.data.mocap_quat[0] = new_target.wxyz_xyz[:4]
        
        # Perform inverse kinematics
        vel = solve_ik(configuration, tasks, dt, solver=solver_name, limits = limits)
        configuration.integrate_inplace(vel, dt)
        
        # Update the physical state of the simulation
        mujoco.mj_forward(configuration.model, configuration.data)
        
        # Save the current frame for video
        renderer.update_scene(configuration.data)

        if is_render:
            pixels = renderer.render()
            frames.append(pixels)

    if is_render:
        media.show_video(frames, fps = rendering_fps, loop = False)