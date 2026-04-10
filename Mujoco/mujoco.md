
# References
- mujoco tutorial: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb

# MuJoCo

## Managed viewer

MuJoCo 엔진이 시뮬레이션의 시간 흐름, 물리 법칙 계산, 화면 렌더링을 직접 관리함. 사용자는 엔진이 돌아가는 도중에 특정 시점마다 실행된 콜백 함수만 전달함.

- viewer.launch(): launches an empty visualization session, where a model can be loaded by drag-and-drop
- viewer.launch(model): launches a visualization session for the given mjModel where the visualizer internally creates its own instance of mjData
- viewer.launch(model, data): is the same as above, except that the visualizer operates directly on the given mjData instance

## Standalone app

- python -m mujoco.viewer: launches an empty visualization session, where a model can be loaded by drag-and-drop
- python -m mujoco.viewer --mjcf=[xml_path]: launches a visualization session for the specified model file

## Passive viewer

- viewer.launch_passive: function launches the interactive viewer in a way which does not block, allowing user code to continue execution. In this mode, the user's script is responsible for timing and advancing the physics state, and mouse-drag perturbations will not work unless the user explicitly synchronizes incoming events.
- lock(): provides a mutext lock for the viewer as a context manager. Since the viewer operates its own thread, user code must ensure that it is holding the viewer lock before modifying any physics or visualization state. 
- sync(): synchronizes between the user's mjModel, mjData and the GUI. 
- key_callback: A callable which gets called each time a keyboard event occurs in the viewer window.

# Core functions

- mjModel: containes the model description, i.e., all quantities which do not change over time. 
    - ngeom: #geometrys in the scene
    - geom_rgba: their respective colors
    - geom(name): we can inspect properties using `model.geom("green_sphere")`
- mjData: contains the state and quantities that depend on it. The state is made up of time, generalized positions and generalized velocities. These are respectively data.time, data.qpos and data.qvel. In order to make a new mjData, all we need is our mjModel using `mujoco.MjData(model)`
    - contains functions of the state, for example the Cartesian positions of objects in the world frame. The (x, y, z) positions of our two geoms are `in data.geom_xpos`:
        - **To update the values in here, the data should be explicitly propagated.**
- mujoco.mj_kinematics(model, data): computes global Cartesian poses for all objects (excluding cameras and lights).  
- mujoco.mj_forward(): which invokes the entire pipeline up to the computation of accelerations. i.e., it computs $\dot{x} = f(x) $, where $x$ is the state. 
- mujoco.Renderer
- mjvScence: which is an object held by the renderer describing the visual scence. 
- mujoco.mj_step: which steps the state $x_{t+h} = f(x_t)$.

# Concept

- bodies: the things that move (and which have inertia) are called bodies.
- free body: is a body with a free joint having 6 DoFs, i.e., 3 translations and 3 rotations.
- 6 DoFs: is the number of independent movement which an object can have in space.
  - 3 translations: x, y, z
  - 3 rotations: roll, pitch, yaw

# Model

- xml: The xml string is written in MuJoco's MJCF, which is an XML-based modeling language 
  - &lt;mujoco&gt;: This is minimal requirement to make valid MJCF model
    - &lt;worldbody&gt;: All physical elements live inside of this tag. This is always the top-level body an constitutes the global origin in Cartesian coordinates.
    - &lt;option&gt;: set the integrator to the 4th order Runge Kutta.
    - &lt;asset&gt;: serves as a centralized repository for declaring reusable resources
    - &lt;keyframe&gt;: allows you to store and recall specific snapshots of the simulation state—including positions, velocities, and actuator states—enabling the model to be reset to precise initial conditions or complex configurations at any time
    - &lt;camera&gt;: defines a specific viewpoint within the simulation—specifying its position, orientation, and field of view
    - &lt;body&gt; defines a rigid object that groups geoms and is connected to the system through joints
      - &lt;geom&gt;: A tag for a geometric object that is visible and can participate in contact and collision.
      - &lt;joint&gt; defines the degrees of freedom that specify how a body can move relative to its parent.
      - &lt;freejoint&gt;

# Rendering

In order to render we'll need to instantiate a `Renderer` object and calls its `render` method.

```python
xml = """
<mujoco>
  <worldbody>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
# Make model and data
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Make renderer, render and show the pixels
with mujoco.Renderer(model) as renderer:
  mujoco.mj_forward(model, data)
  renderer.update_scene(data)

  media.show_image(renderer.render())
```
