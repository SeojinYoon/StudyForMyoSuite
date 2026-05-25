
import opensim as osim
path = "/Users/seojin/Downloads/MOBL_41_OriginalModel/MOBL_ARMS_fixed_41.osim"

model = osim.Model(path)
fs = model.updForceSet()

remove_names = [
    "s_glenohum",
    "m_glenohum",
    "i_glenohum",
    "coracohum",
]

for i in reversed(range(fs.getSize())):
    if fs.get(i).getName() in remove_names:
        print("remove:", fs.get(i).getName())
        fs.remove(i)

state = model.initSystem()
print("SUCCESS without GH ligaments")