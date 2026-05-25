
import opensim as osim

path = "/Users/seojin/Downloads/MOBL_41_OriginalModel/MOBL_ARMS_fixed_41.osim"

base = osim.Model(path)
forces = base.getForceSet()

force_names = [
    forces.get(i).getName()
    for i in range(forces.getSize())
    if "Muscle" not in forces.get(i).getConcreteClassName()
]

good = []

for name in force_names:
    model = osim.Model(path)
    fs = model.updForceSet()

    for i in reversed(range(fs.getSize())):
        if fs.get(i).getName() == name:
            fs.remove(i)
            break

    try:
        model.initSystem()
        print("SUCCESS after removing:", name)
        good.append(name)
    except Exception:
        pass

print(good)