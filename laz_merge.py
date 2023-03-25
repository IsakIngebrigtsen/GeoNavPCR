
import sys
import traceback
import laspy
import os
import numpy as np
import time
import absolute_navigator_ICP as Pr
from collect_filename import get_files
file_list = get_files(16, 1)  # the files from the 10th file and 5 files on # Take file nr. 17 next.
full_pc = np.zeros(3)
start_time = time.time()

for files in file_list:  # For loop that goes through the PCAP files, and the corresponding laz files.
    # Source_init is the raw laz file corresponding to the PCAP file
    laz_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + files + ".laz"
    las = laspy.read(laz_file)
    xyz = las.xyz
    full_pc = np.vstack((full_pc, xyz))
full_pc = np.delete(full_pc, 0, 0)
source = Pr.point_cloud_pros(full_pc)
source_np = np.asarray(source.points)


# full_pc = np.reshape(full_pc, (-1,3))
# source_filename = 'pros_data\\test.npy'
# np.save(source_filename, full_pc)
end = time.time()
total_time = end - start_time
total_time = np.round(total_time, 4)
print(f'Done, it took {total_time} seconds')


# file_list = get_files(16, 1)
# laz_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + file_list + ".laz"


# lass = laspy.read(laz_file)
# xyz = lass.xyz
def scaled_x_dimension(las_file):
    x_dimension = las_file.X
    y_dimension = las_file.Y
    z_dimension = las_file.Z
    scale = 0.001
    x_offset = las_file.header.offsets[0]
    y_offset = las_file.header.offsets[1]
    z_offset = las_file.header.offsets[2]

    x = (x_dimension * scale) + x_offset
    y = (y_dimension * scale) + y_offset
    z = (z_dimension * scale) + z_offset
    return np.stack((x, y, z), axis=-1)


# XYZ = scaled_x_dimension(las)

import pylas
import numpy as np
laz_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_200102.laz"
with pylas.open(laz_file) as fh:
    print('Points from Header:', fh.header.point_count)
    las = fh.read()
    print(las)
    print('Points from data:', len(las.points))
    ground_pts = las.classification == 2
    bins, counts = np.unique(las.return_number[ground_pts], return_counts=True)
    print('Ground Point Return Number distribution:')
    for r,c in zip(bins,counts):
        print('    {}:{}'.format(r,c))
las_pylas = np.stack((las.x, las.y, las.z), axis=-1)




print(np.unique(las.classification))

Pr.draw_las(full_pc, f'{file_list}_test_5.laz')
