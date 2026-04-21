
import os
from myoconverter.O2MPipeline import O2MPipeline

# define pipeline configurations
kwargs = {}  # define kwargs inputs
kwargs['convert_steps'] = [1, 2, 3]    # All three steps selected
kwargs['muscle_list'] = None           # No specific muscle selected, optimize all of them
kwargs['osim_data_overwrite'] = True   # Overwrite the Osim model state files
kwargs['conversion'] = True            # Yes, perform 'Cvt#' process
kwargs['validation'] = True            # Yes, perform 'Vlt#' process
kwargs['speedy'] = False               # Do not reduce the checking notes to increase speed
kwargs['generate_pdf'] = True          # Do not generate validation pdf report
kwargs['add_ground_geom'] = True       # Add ground to the model
kwargs['treat_as_normal_path_point'] = False    # Using constraints to represent moving and conditional path points

############### Simple Arm 2 DoFs 6 Muscles ################ 
dir_path = "/home/seojin/myoconverter/models/osim/Arm26"
osim_file = os.path.join(dir_path, "arm26.osim")
geometry_folder = os.path.join(dir_path, "Geometry")
output_folder = "/mnt/ext1/seojin/temp/myoconverter"
O2MPipeline(osim_file, geometry_folder, output_folder, **kwargs)
