
# References
- mujoco tutorial: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb
- modeling: https://mujoco.readthedocs.io/en/latest/modeling.html#mocap-bodies
- MJCF: https://mujoco.readthedocs.io/en/latest/XMLreference.html

# MuJoCo

- Managed viewer: MuJoCo handles the simulation's timeline, physics, and rendering. Users only need to define callback functions to be triggered at specific points during execution
  - API
    - viewer.launch(): launches an empty visualization session, where a model can be loaded by drag-and-drop
    - viewer.launch(model): launches a visualization session for the given mjModel where the visualizer internally creates its own instance of mjData
    - viewer.launch(model, data): is the same as above, except that the visualizer operates directly on the given mjData instance
  - advantage: this viewer is good when we want to simulate somthing itself without external interruption ex) RL simulation
- Standalone app
  - API
    - python -m mujoco.viewer: launches an empty visualization session, where a model can be loaded by drag-and-drop
    - python -m mujoco.viewer --mjcf=[xml_path]: launches a visualization session for the specified model file
- Passive viewer: The user manually manages the physics and time flow within a custom loop, while the engine performs computations only when explicitly called by the user
  - API
    - viewer.launch_passive: function launches the interactive viewer in a way which does not block, allowing user code to continue execution. In this mode, the user's script is responsible for timing and advancing the physics state, and mouse-drag perturbations will not work unless the user explicitly synchronizes incoming events.
    - lock(): provides a mutext lock for the viewer as a context manager. Since the viewer operates its own thread, user code must ensure that it is holding the viewer lock before modifying any physics or visualization state. 
    - sync(): synchronizes between the user's mjModel, mjData and the GUI. 
    - key_callback: A callable which gets called each time a keyboard event occurs in the viewer window.
  - advantage: This mode is ideal for tasks requiring precise data synchronization, such as inverse kinematics (IK) or mapping pre-recorded motion data, as it allows the user to update the model's state and sync it with the visualizer at any desired pace


# Core functions

- mjData: contains the state and quantities that depend on it. The state is made up of time, generalized positions and generalized velocities. These are respectively **data.time**, **data.qpos** and **data.qvel**. In order to make a new mjData, all we need is our mjModel using `mujoco.MjData(model)`
    - contains functions of the state, for example the Cartesian positions of objects in the world frame. The (x, y, z) positions of our two geoms are `in data.geom_xpos`:
        - **To update the values in here, the data should be explicitly propagated.**

- mujoco.mj_kinematics(model, data): computes global Cartesian poses for all objects (excluding cameras and lights).  
- mujoco.mj_forward(): which invokes the entire pipeline up to the computation of accelerations. i.e., it computs $\dot{x} = f(x) $, where $x$ is the state. 
- mujoco.Renderer
- mjvScence: which is an object held by the renderer describing the visual scence. 
- mujoco.mj_step: which steps the state $x_{t+h} = f(x_t)$.

# Concept

- MJCF: MuJoCo XML Format
- worldbody: The top-level body
- body: the things that move (and which have inertia) are called bodies.
  - **joint**, **geom**, **site**, **camera**, **light** is fixed to the local frame of the body and always moves with it.
  - free body: is a body with a free joint having 6 DoFs, i.e., 3 translations and 3 rotations.
- 6 DoFs: is the number of independent movement which an object can have in space.
  - 3 translations: x, y, z
  - 3 rotations: roll, pitch, yaw
- Spatial frame: contains location(x,y,z) and orientation, 6-DOF
  - right-handed rule
- geometric object: 겉모양과 충돌을 담당하는 요소
- MuJoco's frame types
  - Bodies: each body has two frames
      - Regular Frame: a frame used to define it as well as to position other elements relative to it
      - Inertial Frame: centered at the body's center of mass and aligned with its pricinpal axes of inertia.
  - Geoms: collision and visual geometry attached to bodies. Each geoms has its own frame, which may be offset from the parent body.
  - Sites:**Massless frames attached to bodies**. **Sites mark points of interest** such as end-effectors, sensor locations, and grasp points, without adding collision geometry. IK tasks are typically defined on sites.
    - Sites do not participant in collision detection or automated computation of inertial properties
- joint
  - joint in body, create motion degrees of freedom between parent and child body. Without joint, that child body is welded to its parent
  - types
    - hinge: rotation about one axis (1 DOF)
    - slide: translation along one axis (1 DOF)
    - ball: spherical joint (3 DOF, but 4 values in qpos due to quaternion overparameterization)
    - free: unconstrained 6-DOF motion (6 DOF but 7 values in qpos: 3 translation + 4 quaternion)
      - mujoco uses scalar-first quaternions (wxyz). where w is the real component

# Coordinate System

mujoco uses a right-handed coordinate system. By default:
- Z is up (gravity points in -Z)
- X is forward
- Y is lef

# Model

- MJCF formats(xml) -> document object model (DOM) -> mjSpec -> mjModel(compiled)
- Python
  - mjModel: containes the model description, i.e., all quantities which do not change over time. 
      - ngeom: #geometrys in the scene
      - geom_rgba: their respective colors
      - geom(name): we can inspect properties using `model.geom("green_sphere")`
  - mujoco.MjModel.from_xml_path: load model from xml file
  - mujoco.mj_saveModel: save model into a binary MJB file
  - mujoco.mj_saveLastXML: save mjSpec as MJCF

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

# MJCF

## Transform

The relationship between parent frame and child frame is represented as the attribute within a frame tag
- pos: represent child frame's position relative to parent frame origin
- quat: represent child frame's orientation relative to parent frame axes

```xml
<body name="link0" childclass="panda">
  <body name="link1" pos="0 0 0.333">
    <body name="link2" quat="1 -1 0 0"/> 
  </body>
</body>
```

## Kinematic tree

## Default settings

When a defaults class is defined within another defaults class, the child automatically inherits all attribute values from the parent. It can then override some or all of them by defining the corresponding attributes. The top-level defaults class does not have a pearnt, and so its attributes are initialized to internal defaults.

## Coordinate frames

The positions and orientations of all elements defined in the kinematic tree are expressed in local coordinates, relative to the parent body for bodies, and relative to the body that contains the element for geoms, joints, sites, cameras and lights

# Actuators

## Force limits

## Length range

## Stateful actuators

### Activation limts

### Muscles

# MyoConverter

MyoConverter is a tool for converting OpenSim musculoskeletal (MSK) models to the MuJoCo model format with optimized muscle kinematics and kinetics.

# MoCap bodies

mocap bodies are static children of the world (i.e., have no joints) and their mocap attribute is set to “true”. They can be used to input a data stream from a motion capture device into a MuJoCo simulation.

The first step is to define a mocap body in the MJCF model, and implement code that reads the data stream at runtime and sets mjData.mocap_pos and mjData.mocap_quat to the position and orientation received from the motion capture system. The simulate.cc code sample uses the mouse as a motion capture device, allowing the user to move mocap bodies around:

# Jacobian

mujoco's mj_jacBody, mj_jacSite, and mj_jacGeom return Jacobians in the local-world-aligned frame: the origin is at the frame, but axes are aligned with the world. In Pinocchio terminology, this is LOCAL_WORLD_ALIGNED
