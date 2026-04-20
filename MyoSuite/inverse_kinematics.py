
# Common Libraries
import os, mink, mujoco, mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

# Paths
os.chdir("/Users/seojin/myosuite/myosuite/simhive/myo_sim/arm")
_XML_ARM_Model = "myoarm.xml"

# Model specification
xml_string = f"""
        <mujoco model="MyoArm with Mocap">
            <include file="{_XML_ARM_Model}"/>
            <worldbody>
                <body name="target" pos="0 0 0" quat="0 1 0 0" mocap="true">
                    <geom type="box" size=".15 .15 .15" contype="0" conaffinity="0" rgba=".6 .3 .3 .2"/>
                </body>
            </worldbody>
        </mujoco>
        """

# Initialize model and data
model = mujoco.MjModel.from_xml_string(xml_string)
data = mujoco.MjData(model)

# Inverse Kinematics configuration
configuration = mink.Configuration(model)
tasks = [
    end_effector_task := mink.FrameTask(
        frame_name="S_grasp", # FrameTask: 로봇의 S_grasp (손바닥 부위) 지점이 목표 지점의 위치와 방향을 따르도록 한다.
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    ),
    posture_task := mink.PostureTask(model=model, cost=1e-2), # 로봇이 특정 자세를 유지하려는 성질을 부여함. 관절이 꺾이거나 이상한 자세가 되는 것을 방지하는 일종의 정규화 역할을 함.
]
solver = "quadprog"
pos_threshold = 1e-4
ori_threshold = 1e-4
max_iters = 20

with mujoco.viewer.launch_passive(model=model, 
                                  data=data, 
                                  show_left_ui=False, 
                                  show_right_ui=False) as viewer:
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)
    mujoco.mj_forward(model, data)

    # Initialize the mocap target at the end-effector site.
    mink.move_mocap_to_frame(model, data, "target", "S_grasp", "site")

    rate = RateLimiter(frequency=500.0, warn=False) # 시뮬레이션 속도를 일정하게(500Hz) 유지
    while viewer.is_running():
        # 사용자가 마우스로 옮긴 target의 현재 위치를 가져옴
        T_wt = mink.SE3.from_mocap_name(model, data, "target")

        # 그 위치를 IK가 쫓아가야 할 목표(set_target)로 설정함
        end_effector_task.set_target(T_wt)

        # Compute velocity and integrate into the next configuration.
        for i in range(max_iters):
            vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 1e-3) # quadprog 솔버를 사용하여 현재 위치에서 목표 위치로 가기 위한 관절 속도를 계산함.
            configuration.integrate_inplace(vel, rate.dt) # 계산된 속도에 시간 변화량(dt)을 곱해 다음 관절 위치(q)를 업데이트 함.
            err = end_effector_task.compute_error(configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold
            if pos_achieved and ori_achieved:
                break

        data.qpos[:] = configuration.q # qpos를 실제 MuJoCo 데이터에 적용하고 mj_step을 통해 물리 법칙을 계산함.
        mujoco.mj_step(model, data)

        # Visualize at fixed FPS.
        viewer.sync()
        rate.sleep()
