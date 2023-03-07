import copy
import numpy as np
import open3d as o3
from probreg import cpd
import absolute_PCAP_ICP as Pr
from collect_filename import get_files
import time
import numpy as np
use_cuda = True
if use_cuda:
    import cupy as cp
    to_cpu = cp.asnumpy
    cp.cuda.set_allocator(cp.cuda.MemoryPool().malloc)
else:
    cp = np
    to_cpu = lambda x: x
import open3d as o3
import transforms3d as t3d
from probreg import cpd
from probreg import callbacks
import utils
import time

voxel_size = 0.5  # means 5cm for this dataset
file = get_files(17, 1)[0]
frame_index = 60
accumulatedTime = 0.0
startTime = time.perf_counter()
geoid_height = 39.438

source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + file + ".laz"
# source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\NDH-Lillehammer.laz"

# source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Meged_file.las"

pc_raw_laz = Pr.read_laz(source_init)

pcap_file, meta, sbet = Pr.filetype(file, system="ETPOS")
sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
sbet_file = sbet_PPP
with open("frame_7.txt", 'w') as f:
    pc_transformed, time_est, center_coord_utm32, heading = Pr.transform_pcap(pcap_file, meta, sbet_file, frame_index+1, geoid_height)
source = pc_transformed

with open("frame_7.txt", 'w') as f:
    pc_transformed, time_est, center_coord_utm32, heading = Pr.transform_pcap(pcap_file, meta, sbet_file, frame_index, geoid_height)
target = pc_transformed


# pc_raw_laz = remove_outside_source(pc_raw_laz, target, 25) # Cropped source from NDH
pc_o3d_laz = Pr.point_cloud_pros(pc_raw_laz)

# source = pc_o3d_laz
downsampled_source, source_transformed, downsampled_target, target_transformed = Pr.initial_transform(source, target)

source = cp.asarray(downsampled_source.points, dtype=cp.float32)
target = cp.asarray(downsampled_target.points, dtype=cp.float32)

rcpd = cpd.RigidCPD(source, use_cuda=use_cuda)
start = time.time()
tf_param, _, _ = rcpd.registration(target)
elapsed = time.time() - start
print("time: ", elapsed)

print("result: ", np.rad2deg(t3d.euler.mat2euler(to_cpu(tf_param.rot))),
      tf_param.scale, to_cpu(tf_param.t))
