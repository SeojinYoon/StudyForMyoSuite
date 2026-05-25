
import opensim as osim

path = "/Users/seojin/Downloads/MOBL_41_OriginalModel/MOBL_ARMS_fixed_41.osim"

base = osim.Model(path)

forces = base.getForceSet()

muscle_names = []

for i in range(forces.getSize()):

    f = forces.get(i)

    if "Muscle" in f.getConcreteClassName():
        muscle_names.append(f.getName())

bad = []

for name in muscle_names:

    print("Testing:", name)

    model = osim.Model(path)

    fs = model.updForceSet()

    for i in reversed(range(fs.getSize())):

        f = fs.get(i)

        if f.getName() == name:
            fs.remove(i)
            break

    try:
        model.initSystem()

        print("SUCCESS after removing:", name)

        bad.append(name)

    except:
        pass

print("\nPossible problematic muscles:")
print(bad)
