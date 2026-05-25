
from importlib.resources import path

import opensim as osim
import time


# 모델 로드
model = osim.Model("/Users/seojin/Downloads/MOBL_41_OriginalModel/MOBL_ARMS_41.osim")


# visualizer 켜기
model.setUseVisualizer(True)

# state 초기화
state = model.initSystem()

# 모든 muscle excitation = 0
controller = osim.PrescribedController()

muscles = model.getMuscles()

for i in range(muscles.getSize()):
    muscle = muscles.get(i)

    controller.addActuator(muscle)

    controller.prescribeControlForActuator(
        muscle.getName(),
        osim.Constant(0.0)
    )

model.addController(controller)

# 다시 initialize
state = model.initSystem()

# muscle equilibrium
model.equilibrateMuscles(state)

# visualizer
viz = model.updVisualizer().updSimbodyVisualizer()

# manager
manager = osim.Manager(model)
manager.initialize(state)

# simulation
dt = 0.01
final_time = 5.0

t = 0.0

while t < final_time:

    t += dt

    state = manager.integrate(t)

    # 화면 업데이트
    model.realizeVelocity(state)

    time.sleep(0.01)
    