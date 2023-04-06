import numpy as np
import copy
import time
from draw_registration import draw_registration_result, draw_icp
import absolute_navigator_ICP as Pr
from collect_filename import get_files
import open3d as o3d
import draw_registration


def remove_outside_source(source_pc, target_pc, meters = 25):
    init_pos = target_pc.get_center()
    # Extract a part of the cloud around the actual position. This is the cloud we are going to register against.
    a = source_pc
    partial_radius = meters
    points = a[(a[:, 0] >= init_pos[0] - partial_radius) & (a[:, 0] <= init_pos[0] + partial_radius) & (
                a[:, 1] >= init_pos[1] - partial_radius) & (a[:, 1] <= init_pos[1] + partial_radius)]
    return points


voxel_size = 0.5  # means 5cm for this dataset
file = get_files(10, 1)[0]
frame_index = 60
accumulatedTime = 0.0
startTime = time.perf_counter()
geoid_height = 39.438

source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + file + ".laz"
# source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\NDH-Lillehammer.laz"

# source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Meged_file.las"

pc_raw_laz = Pr.read_laz(source_init)

pcap_file, meta, sbet_file = Pr.filetype(file, system="ETPOS")
# sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
# sbet_file = sbet_PPP
with open("frame_7.txt", 'w') as f:
    pc_transformed, time_est, center_coord_utm32, heading, origin = Pr.transform_pcap(pcap_file, meta, sbet_file, frame_index, geoid_height)
target = pc_transformed

# pc_raw_laz = remove_outside_source(pc_raw_laz, target, 25) # Cropped source from NDH
pc_o3d_laz = Pr.point_cloud_pros(pc_raw_laz)
source = pc_o3d_laz
source_for_plotting = source.voxel_down_sample(voxel_size=0.2)
draw_registration.draw_absolute_registration_result(source_for_plotting, target,target.get_center() - origin)
saved_center = source.get_center()

downsampled_source, source_transformed, downsampled_target, target_transformed, target_center = Pr.initial_transform(source, target,center_coord_utm32)

accumulatedTime += time.perf_counter() - startTime
print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")

threshold = 1
trans_init = np.identity(4)
source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
# draw_registration.draw_icp(source_for_plotting, target_transformed, trans_init)
draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed,target_center-origin)
trans_init = Pr.o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)
trans_init = Pr.o3d_icp(source_transformed, target_transformed, trans_init, iterations=1)
# trans_init = draw_icp(downsampled_source, target_transformed, trans_init)

source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
# draw_registration.draw_icp(source_for_plotting, target_transformed, trans_init)
target_transformed.transform(trans_init)
source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
# draw_registration.draw_icp(source_for_plotting, target_transformed, trans_init)
draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed, target_center + trans_init[0:3, 3]-origin)
source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
target_ICP = copy.deepcopy(target_transformed).translate(target_center + trans_init[0:3, 3] + saved_center, relative=False)
# trans_init = draw_icp(voxeldown_source, target_transformed, trans_init)
source_for_plotting = source.voxel_down_sample(voxel_size=0.2)
draw_registration.draw_absolute_registration_result(source_for_plotting, target_ICP, target_center + trans_init[0:3, 3] + saved_center-origin)
"""
source_for_plotting.paint_uniform_color([1, 0.706, 0])
target_ICP.paint_uniform_color([0, 0.651, 0.929])
# o3d.visualization.draw_geometries([source_ICP, target_ICP])

vis = o3d.visualization.Visualizer()
vis.create_window()
for g in [source_for_plotting, target_ICP]:
    vis.add_geometry(g)

ropt = vis.get_render_option()
ropt.point_size = 1.0
ropt.background_color = np.asarray([0, 0, 0])

# initialize camera settings
ctr = vis.get_view_control()
ctr.set_zoom(0.3)
ctr.set_lookat(target_ICP.get_center())
ctr.set_up([0.85, 0.12, 0.52])
print('source init')
print(target_ICP.get_center())
# run visualizer main loop
print("Press Q or Excape to exit")
vis.poll_events()
vis.update_renderer()
# vis.capture_screen_image('img_Pointcloud\\RUN' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
vis.run()

# Pr.draw_las(source_ICP.points, "source_test")
# Pr.draw_las(target_ICP.points, "target_test")
"""
