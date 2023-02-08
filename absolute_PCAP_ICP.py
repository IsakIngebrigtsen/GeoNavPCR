import ouster.pcap as pcap
import ouster.client as client
from contextlib import closing
from more_itertools import nth
import laspy


def transform_pcap(pcap_raw, metadata, sbet=None, frame=None, geoid=0.0):
    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader
    pcap_reader = PcapReader(pcap_raw, metadata, sbet_path=sbet)  # Erlend Dahl Teapot

    pcap_reader.print_info(printFunc=lambda l: f.write(l + "\n"), frame_index=frame)  # Erlend Dahl Teapot
    position = pcap_reader.get_coordinates()  # Erlend Dahl Teapot
    # Collects the lat, lon, alt, and heading at a given frame Teapot
    lat = position[frame].x
    lon = position[frame].y
    heading = position[frame].heading
    print(heading)
    center_coord_utm33 = np.array((lat, lon, position[frame_index].alt - geoid))
    raw_pointcloud = get_frame(pcap_raw, metadata, frame)  # PCAP Software

    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    point_cloud_prossesing = pcap_reader.remove_vehicle(raw_pointcloud_correct_shape)  # Erlend Dahl Teapot
    point_cloud_prossesing = pcap_reader.remove_outside_distance(30, point_cloud_prossesing)  # Erlend Dahl Teapot
    pc_o3d = point_cloud_pros(point_cloud_prossesing)
    rotation_matrix = pc_o3d.get_rotation_matrix_from_axis_angle(np.array([0, 0, quadrant(heading)]))  # Open3d

    pc_o3d.rotate(rotation_matrix, center=(pc_o3d.get_center()))  # open3d
    pc_transformed_utm = copy.deepcopy(pc_o3d).translate(center_coord_utm33, relative=False)  # open3d
    return pc_transformed_utm


def get_frame(pcap_raw, metadata, frame):
    # Read the metadata from the JSON file.

    with open(metadata, 'r') as read:
        metadata = client.SensorInfo(read.read())

    # Open the LIDAR data source from the PCAP file
    lidar_data = pcap.Pcap(pcap_raw, metadata)

    # Read the xth frame
    with closing(client.Scans(lidar_data)) as scans:
        scan = nth(scans, frame)
    # Create a function that translates coordinates to a plottable coordinate system
    xyzlut = client.XYZLut(lidar_data.metadata)
    pc_nparray = xyzlut(scan)  # Transform point cloud to numpy array.
    return pc_nparray


def point_cloud_pros(xyz):
    # Process the point cloud data, so that it comes in a standardised format from open 3d.
    xyz = xyz.reshape((-1, 3))
    pc_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pc_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    return pc_o3d


def quadrant(heading):
    heading = heading - np.pi/2
    if heading < 0:
        return np.absolute(heading)
    elif heading > 0:
        return np.pi*2-heading


def initial_transform(init_source, init_target):
    # initial center for the source point cloud. subtract this to get local plottable coordinates.
    init_center = init_source.get_center()
    source_center_init = init_source.get_center() - init_center
    target_center_init = init_target.get_center() - init_center
    # initial local transformation to a local coordinate system
    source_trans = copy.deepcopy(init_source).translate(source_center_init, relative=False)
    target_trans = copy.deepcopy(init_target).translate(target_center_init, relative=False)
    voxeldown_source = source_trans.voxel_down_sample(voxel_size=0.5)
    voxeldown_target = target_trans.voxel_down_sample(voxel_size=0.5)
    return voxeldown_source, source_trans, voxeldown_target, target_trans


def get_gps_week(pcap_path=None, pcap_filename=None):
    import os
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.sbetParser import filename2gpsweek
    if pcap_path is not None:
        pcap_filename = os.path.basename(pcap_path)
    return filename2gpsweek(pcap_filename)


def read_laz(laz_file):
    las = laspy.read(laz_file)
    xyz = las.xyz
    return xyz


def o3d_icp(init_source, init_target, transformation, iterations, threshold_value=1):
    for k in range(iterations):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            init_source, init_target, threshold_value, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        if k > 1 and np.abs(np.mean(reg_p2l.transformation-transformation)) < 1e-16:
            print(f'ICP ran {k} number of iteration before converging.')
            transformation = reg_p2l.transformation
            break
        transformation = reg_p2l.transformation
    return transformation


def draw_las(pc_points, file_name):
    # draw_las created by code from https://laspy.readthedocs.io/en/latest/examples.html
    my_data = np.asarray(pc_points)

    header = laspy.LasHeader(point_format=3, version="1.2")
    header.add_extra_dim(laspy.ExtraBytesParams(name="random", type=np.int32))
    header.offsets = np.min(my_data, axis=0)
    header.scales = np.array([0.1, 0.1, 0.1])

    # 2. Create a Las
    las = laspy.LasData(header)

    las.x = my_data[:, 0]
    las.y = my_data[:, 1]
    las.z = my_data[:, 2]

    las.write(file_name + ".las")


if __name__ == "__main__":

    import sys
    import open3d as o3d
    import numpy as np
    import copy
    import time
    from ICP_Point import draw_registration_result, draw_icp

    voxel_size = 0.5  # means 5cm for this dataset
    file = "195032"
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    frame_index = 150
    geoid_height = 39.438
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + file + ".laz"
    # source_init = "C:\\Users\\isakf\Documents\\1_Geomatikk\\Master\\Data\\Meged_file.las"

    pc_raw_laz = read_laz(source_init)
    pc_o3d_laz = point_cloud_pros(pc_raw_laz)

    filename = "OS-1-128_992035000186_1024x10_20211021_" + file
    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
    pcap_file = pathBase + filename + ".pcap"
    meta = pathBase + filename + ".json"
    sbet_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"

    gps_week = get_gps_week(pcap_filename=filename)

    with open("frame_7.txt", 'w') as f:
        pc_transformed = transform_pcap(pcap_file, meta, sbet_file, frame_index, geoid_height)
    source = pc_o3d_laz
    target = pc_transformed
    saved_center = source.get_center()
    downsampled_source, source_transformed, downsampled_target, target_transformed = initial_transform(source, target)
    accumulatedTime += time.perf_counter() - startTime
    print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")
    threshold = 1
    trans_init = np.identity(4)
    draw_registration_result(downsampled_source, downsampled_target, trans_init)
    trans_init = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)

    trans_init = draw_icp(source_transformed, target_transformed, trans_init)

    target = copy.deepcopy(target_transformed).transform(trans_init)

    source_transformed = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
    target_transformed = copy.deepcopy(target_transformed).translate(target.get_center() + saved_center, relative=False)

    draw_las(source_transformed.points, "source_test")
    draw_las(target_transformed.points, "target_test")
