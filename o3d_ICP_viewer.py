import numpy as np
import copy
import time
from ICP_Point import draw_registration_result, draw_icp
import absolute_PCAP_ICP as Pr
from collect_filename import get_files
voxel_size = 0.5  # means 5cm for this dataset
file = get_files(10, 1)[0]
frame_index = 60
accumulatedTime = 0.0
startTime = time.perf_counter()
geoid_height = 39.438
source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + file + ".laz"
# source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Meged_file.las"

pc_raw_laz = Pr.read_laz(source_init)
pc_o3d_laz = Pr.point_cloud_pros(pc_raw_laz)
source = pc_o3d_laz
pcap_file, meta, sbet_file = Pr.filetype(file, system="ETPOS")
with open("frame_7.txt", 'w') as f:
    pc_transformed, time_sbet, coord = Pr.transform_pcap(pcap_file, meta, sbet_file, frame_index, geoid_height)
target = pc_transformed
saved_center = source.get_center()
downsampled_source, source_transformed, downsampled_target, target_transformed = Pr.initial_transform(source, target)
accumulatedTime += time.perf_counter() - startTime
print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")

threshold = 1
trans_init = np.identity(4)

draw_registration_result(source_transformed, target_transformed, trans_init)
trans_init = Pr.o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)
trans_init = Pr.o3d_icp(source_transformed, target_transformed, trans_init, iterations=1)
trans_init = draw_icp(source_transformed, target_transformed, trans_init)
target_transformed.transform(trans_init)

source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
target_ICP = copy.deepcopy(target_transformed).translate(target_transformed.get_center() + saved_center, relative=False)
trans_init = np.identity(4)
downsampled_source, source_transformed, downsampled_target, target_transformed = Pr.initial_transform(source_ICP, target_ICP)
trans_init = draw_icp(source_transformed, target_transformed, trans_init)

Pr.draw_las(source_ICP.points, "source_test")
Pr.draw_las(target_ICP.points, "target_test")

