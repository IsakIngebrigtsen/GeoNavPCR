import sys
import traceback
import laspy
import os
import numpy as np
import time
import absolute_navigator_ICP as Pr
from collect_filename import get_files
file_list = get_files(1, 43)  # the files from the 10th file and 5 files on # Take file nr. 17 next.
full_pc = np.zeros(3)
start_time = time.time()

for files in file_list:  # For loop that goes through the PCAP files, and the corresponding laz files.
    # Source_init is the raw laz file corresponding to the PCAP file
    laz_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + files + ".laz"
    las = laspy.read(laz_file)
    xyz = las.xyz
    full_pc = np.vstack((full_pc, xyz))
full_pc = np.delete(full_pc,0,0)
# full_pc = np.reshape(full_pc, (-1,3))
source_filename = 'pros_data\\full_source_PC_np.npy'
np.save(source_filename, full_pc)
end = time.time()
total_time = end - start_time
total_time = np.round(total_time, 4)
print(f'Done, it took {total_time} seconds')


