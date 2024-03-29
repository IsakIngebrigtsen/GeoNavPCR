import ouster.pcap as pcap
import ouster.client as client
from contextlib import closing
from more_itertools import nth
import open3d as o3d
import numpy as np
import copy
import time
# Configure PCAP and JSON file paths

def draw_absolute_registration_result(pc_1,pc_2,target_center):

    source_temp = copy.deepcopy(pc_1)
    target_temp = copy.deepcopy(pc_2)

    source_temp.paint_uniform_color([1.0, 1.0, 1.0])
    target_temp.paint_uniform_color([1.0, 0.5, 0.0])

    # source_temp.paint_uniform_color([1, 1, 1])
    # target_temp.paint_uniform_color([0, 1, 0])
    # o3d.visualization.draw_geometries([source_ICP, target_ICP])
    # create point cloud and coordinate axes geometries
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=target_center)
    source_bounding = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(
        source_temp.get_axis_aligned_bounding_box())
    source_bounding.paint_uniform_color((0.0, 1.0, 0.0))
    target_bounding = o3d.geometry.LineSet.create_from_axis_aligned_bounding_box(
        target_temp.get_axis_aligned_bounding_box())
    target_bounding.paint_uniform_color((0.0, 0.0, 1.0))
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for g in [source_temp, target_temp,source_bounding,target_bounding]:
        vis.add_geometry(g)

    vis.add_geometry(axes)

    ropt = vis.get_render_option()
    ropt.point_size = 1
    ropt.background_color = np.asarray([0, 0, 0])


    # initialize camera settings
    ctr = vis.get_view_control()
    ctr.set_zoom(0.3)
    ctr.set_lookat(target_center)
    ctr.set_up((0.8, 0.2, 0.2))
    print('source init')
    print(target_temp.get_center())
    # run visualizer main loop
    print("Press Q or Excape to exit")
    vis.poll_events()
    # vis.run()
    vis.capture_screen_image('img_Pointcloud\\RUN_absolute' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
    vis.destroy_window()


def draw_registration_result(pc_1, pc_2, transformation):

    source_temp = copy.deepcopy(pc_1)
    target_temp = copy.deepcopy(pc_2)

    source_temp.paint_uniform_color([0.0, 0.5, 1.0])
    target_temp.paint_uniform_color([1.0, 0.5, 0.0])

    target_temp.transform(transformation)

    # create point cloud and coordinate axes geometries
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=target_temp.get_center())

    # initialize visualizer and rendering options
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for g in [source_temp, target_temp]:
        vis.add_geometry(g)

    vis.add_geometry(axes)
    ropt = vis.get_render_option()
    ropt.point_size = 0.5
    ropt.background_color = np.asarray([1, 1, 1])

    # initialize camera settings
    ctr = vis.get_view_control()
    ctr.set_zoom(0.3)
    ctr.set_lookat(target_temp.get_center())
    ctr.set_up([0.85, 0.12, 0.52])
    print('source init')
    print(target_temp.get_center())
    # run visualizer main loop
    print("Press Q or Excape to exit")
    vis.poll_events()
    vis.update_renderer()
    # vis.capture_screen_image('img_Pointcloud\\RUN' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
    vis.run()
    vis.destroy_window()


def match(source, target, threshold = 1, trans_init = None, max_iterations=100):

    # Initialize an initial transformation. This is meant to be a
    # rough transformation to align the frames, but as lidar frames
    # are roughly aligned anyway, we use the identity matrix.
    if trans_init is None:
        trans_init = np.identity(4)

    # Run NICP
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations))

    return reg_p2l


def remove_vehicle(frame, cloud = None):
    # Remove the vehicle, which is always stationary at the center. We don't want that
    # to interfere with the point cloud alignment.

    if cloud is None:
        cloud = frame

    vw = 0.7
    vl = 2.2
    return cloud[((frame[:, 0] > 0.2) | (frame[:, 0] < -vl)) | ((frame[:, 1] > vw) | (frame[:, 1] < -vw)) | ((frame[:, 2] > 0.3) | (frame[:, 2] < -2))]


def draw_icp(source,target,trans_init):

    threshold = 1
    accumulatedTime = 0


    print("Apply point-to-plane ICP")
    startTime = time.perf_counter()
    reg_p2l = o3d.pipelines.registration.registration_icp(
        target, source, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
    accumulatedTime += time.perf_counter() - startTime
    print(f"Time usage: {time.perf_counter() - startTime:0.4f} seconds.")
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    print("Transformed center:")
    print(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray([[0.0, 0.0, 0.0]]))).transform(
        reg_p2l.transformation).get_center())
    print("")

    transformation = reg_p2l.transformation
    draw_registration_result(source, target, trans_init)
    return transformation


if __name__ == "__main__":
    filename = "OS-1-128_992035000186_1024x10_20211021_194723"
    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Raw_Frames_Round_1\\"
    pcap_file = pathBase + filename + ".pcap"
    meta = pathBase + filename + ".json"
    from absolute_navigator_ICP import *
    from absolute_navigator_ICP import get_frame
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    from sys import path
    path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader

    initial_navigation_trajectory = "C:\\Users\\isakf\ \Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
    pcap_reader = PcapReader(pcap_file, meta, sbet_path=initial_navigation_trajectory)  # Erlend Dahl Teapot
    source_pc_numpy = np.load(
        'C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\full_source_PC_np.npy')
    center_coord_init_source = get_coordinate(pcap_reader, 70)
    # To speed up the cutting of the large point cloud, do an initial crop per file, to optimize the speeed.
    partial_radius = 50
    crop_coord, c_epoch = transform_mapprojection(crs_from=7912)
    east_init, north_init, alt_init, epoch_init = crop_coord.transform(center_coord_init_source['lat'],
                                                                       center_coord_init_source['lon'],
                                                                       center_coord_init_source['alt'], c_epoch)

    """
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(source, target, trans_init)
    """