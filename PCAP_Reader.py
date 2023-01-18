
def get_frame(pcap_file, meta, frame):
    # Read the metadata from the JSON file.

    with open(meta, 'r') as f:
        metadata = client.SensorInfo(f.read())

    # Open the LIDAR data source from the PCAP file
    source = pcap.Pcap(pcap_file, metadata)

    # Read the 50th LIDAR frame
    with closing(client.Scans(source)) as scans:
        scan = nth(scans, frame)
    # Create a function that translates coordinates to a plottable coordinate system
    xyzlut = client.XYZLut(source.metadata)
    xyz = xyzlut(scan)
    return xyz


def remove_vehicle(frame, cloud = None):
    # Remove the vehicle, which is always stationary at the center. We don't want that
    # to interfere with the point cloud alignment.

    if cloud is None:
        cloud = frame

    vw = 0.7
    vl = 2.2
    return cloud[((frame[:, 0] > 0.2) | (frame[:, 0] < -vl)) | ((frame[:, 1] > vw) | (frame[:, 1] < -vw)) | ((frame[:, 2] > 0.3) | (frame[:, 2] < -2))]


def read_laz(laz_file):
    las = laspy.read(laz_file)
    xyz = las.xyz
    return xyz

def get_sbet_timestamp(packet=None, timestamps=None):
    if timestamps is None:
        timestamps = packet.header(client.ColHeader.TIMESTAMP)

        # There is a very slight difference, but using the final timestamp seems to give the best position.
    return timestamps[-1]

def get_gps_week(pcap_path = None, pcap_filename = None):
    from sbetParser import filename2gpsweek
    if pcap_path is not None:
        pcap_filename = os.path.basename(pcap_path)
    return filename2gpsweek(pcap_filename)

def print_frame_coordinates(gps_week,pcap_path, meta,sbet_path, frame_index=None, printFunc=print):
    from sbetParser import SbetParser
    ix = -1
    imu = -1
    with open(meta, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_path, metadata)
    sbet = SbetParser(sbet_path)
    gps_week = gps_week


    for packet in source:

        if isinstance(packet, client.LidarPacket):

            ix += 1
            if frame_index is not None and ix != frame_index:
                continue
            printFunc("")
            printFunc(f' frame_index = {ix}')

            if sbet is not None:
                sbet_pos = sbet.get_position(get_sbet_timestamp(packet), gps_week=gps_week)
                printFunc(
                    f' SBET = {sbet_pos}')
                sbet_lat = sbet_pos.lat
                sbet_lon = sbet_pos.lon
                sbet_alt = sbet_pos.alt
            xyz = get_frame(pcap_path, meta, frame_index)
            xyz = xyz.reshape((-1, 3))
            xyz = remove_vehicle(xyz)
            pc_o3d, downsampeled_pc_PCAP = point_cloud_pros(xyz)
            printFunc(f' center = {pc_o3d.get_center()}')
            x, y, z = transform_CRS(sbet_lat, sbet_lon, sbet_alt)
            pc_transformed = copy.deepcopy(pc_o3d).translate((x, y, z), relative=False)
            printFunc(f' center_transformed = {pc_transformed.get_center()}')

        elif isinstance(packet, client.ImuPacket):
                continue

    return pc_transformed
def point_cloud_pros(xyz,voxel_size = 0.5):
    # Process the point cloud data, so that it comes in a standardised format from open 3d.
    xyz = xyz.reshape((-1, 3))

    xyz = remove_vehicle(xyz)

    pc_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))

    pc_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    downsampeled_pc = pc_o3d.voxel_down_sample(voxel_size)

    return pc_o3d, downsampeled_pc

def transform_CRS(lat, lon, alt,FROM_CRS = 4326,TO_CRS = 32633):
    transformer = pyproj.Transformer.from_crs(FROM_CRS, TO_CRS)

    x, y, z = transformer.transform(lat,lon,alt)
    return x, y, z
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def draw_las(pc_points,filename):
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

    las.write(filename + ".las")

if __name__ == "__main__":
    import laspy
    import open3d as o3d
    import ouster.pcap as pcap
    import ouster.client as client
    from contextlib import closing
    from more_itertools import nth
    import numpy as np
    import os
    import pyproj
    import copy
    import time
    from ICP_Point import draw_registration_result, draw_icp
    voxel_size = 0.5  # means 5cm for this dataset
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    target_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"

    xyz = read_laz(source_init)
    pc_o3d_laz, downsampeled_pc = point_cloud_pros(xyz)

    filename = "OS-1-128_992035000186_1024x10_20211021_194723"
    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
    pcap_file = pathBase + filename + ".pcap"
    meta = pathBase + filename + ".json"
    sbet_path =  "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    xyz_Pcap = get_frame(pcap_file,meta,7)
    pc_o3d_Pcap, downsampeled_pc_PCAP = point_cloud_pros(xyz_Pcap)
    gps_week = get_gps_week(pcap_filename= filename)

    #print(np.asarray(pc_o3d_Pcap.points))
    #output = "C:\Users\isakf\Documents\1_Geomatikk\Master\Data\194721_PCAP_coord.txt"
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    with open("frame_7.txt", 'w') as f:
       pc_transformed_7 = print_frame_coordinates(gps_week, pcap_file, meta,sbet_path,70,printFunc=lambda l: f.write(l + "\n"))


    with open("frame_10.txt", 'w') as f:
       pc_transformed_10 = print_frame_coordinates(gps_week, pcap_file, meta,sbet_path,74,printFunc=lambda l: f.write(l + "\n"))


    source = pc_transformed_7
    target = pc_transformed_10
    saved_center = source.get_center()
    source_center_init = source.get_center() - saved_center
    target_center_init = target.get_center() - saved_center

    source_transformed = copy.deepcopy(source).translate(source_center_init, relative=False)
    target_transformed = copy.deepcopy(target).translate(target_center_init, relative=False)

    downsampled_source = source_transformed.voxel_down_sample(voxel_size=0.5)
    downsampled_target = target_transformed.voxel_down_sample(voxel_size=0.5)
    accumulatedTime += time.perf_counter() - startTime
    print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")



    threshold = 1
    trans_init = np.identity(4)
    draw_registration_result(downsampled_source, downsampled_target, trans_init)

    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(source_transformed, target_transformed, trans_init)



    target = copy.deepcopy(target_transformed).transform(trans_init)
    #source = copy.deepcopy(source_transformed).translate(source_center_init + saved_center, relative=False)
    #target = copy.deepcopy(target_transformed).translate(target_transformed.get_center() + saved_center, relative=False)
    #trans_init = draw_icp(source, source, trans_init)

    #target = copy.deepcopy(target).transform(trans_init)

    draw_las(source_transformed.points, "source_pointcloud")
    draw_las(target_transformed.points, "target_pointcloud")


    """
    voxel_size = 0.5
    source_down, source_fpfh = preprocess_point_cloud(source_transformed, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target_transformed, voxel_size)
    result_ransac = execute_fast_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac)
    """
    """
    transformation = result_ransac.transformation
    pc_10_trans = target_transformed.transform(transformation)
    print(np.asarray(pc_transformed_10.points))
    # 1. Create a new header
    my_data = np.asarray(pc_transformed_7.points)

    header = laspy.LasHeader(point_format=3, version="1.2")
    header.add_extra_dim(laspy.ExtraBytesParams(name="random", type=np.int32))
    header.offsets = np.min(my_data, axis=0)
    header.scales = np.array([0.1, 0.1, 0.1])

    # 2. Create a Las
    las = laspy.LasData(header)

    las.x = my_data[:, 0]
    las.y = my_data[:, 1]
    las.z = my_data[:, 2]

    las.write("new_file_3.las")
    """