
def transform_pcap(pcap, metadata, sbet=None, frame=None, geoid = 0.0):
    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot-lidar")
    from pcapReader import PcapReader
    import PCAP_Reader as reader

    pcap_reader = PcapReader(pcap, metadata, sbet_path = sbet)  # Erlend Dahl

    pcap_reader.print_info(printFunc=lambda l: f.write(l + "\n"), frame_index=frame)
    coord, rotate = pcap_reader.get_coordinates() #Erlend Dahl
    lat = coord[frame_index].x
    lon = coord[frame_index].y
    center_coord_UTM33 = np.array((lat, lon, coord[frame_index].alt - geoid))

    raw_pointcloud = reader.get_frame(pcap, metadata, frame) #PCAP Software
    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    point_cloud_prossesing = pcap_reader.remove_vehicle(raw_pointcloud_correct_shape) # Erlend Dahl
    point_cloud_prossesing = pcap_reader.remove_outside_distance(30, point_cloud_prossesing)
    pc_o3d = reader.point_cloud_pros(point_cloud_prossesing)

    pc_transformed_UTM = copy.deepcopy(pc_o3d).translate(center_coord_UTM33, relative=False)
    return pc_transformed_UTM
def initial_transform(source,target):
    saved_center = source.get_center()
    source_center_init = source.get_center() - saved_center
    target_center_init = target.get_center() - saved_center
    source_transformed = copy.deepcopy(source).translate(source_center_init, relative=False)
    target_transformed = copy.deepcopy(target).translate(target_center_init, relative=False)
    downsampled_source = source_transformed.voxel_down_sample(voxel_size=0.5)
    downsampled_target = target_transformed.voxel_down_sample(voxel_size=0.5)
    return downsampled_source, source_transformed,downsampled_target, target_transformed

def get_gps_week(pcap_path = None, pcap_filename = None):
    import os
    from sbetParser import filename2gpsweek
    if pcap_path is not None:
        pcap_filename = os.path.basename(pcap_path)
    return filename2gpsweek(pcap_filename)


if __name__ == "__main__":

    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot-lidar")
    from pcapReader import PcapReader
    import open3d as o3d
    import numpy as np
    import copy
    import time
    import PCAP_Reader as reader
    from ICP_Point import draw_registration_result,draw_icp
    voxel_size = 0.5  # means 5cm for this dataset
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_195548.laz"
    pc_raw_laz = reader.read_laz(source_init)
    pc_o3d_laz = reader.point_cloud_pros(pc_raw_laz)

    filename = "OS-1-128_992035000186_1024x10_20211021_195548"
    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
    pcap_file = pathBase + filename + ".pcap"
    meta = pathBase + filename + ".json"
    sbet_file = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"

    gps_week = get_gps_week(pcap_filename=filename)

    accumulatedTime = 0.0
    startTime = time.perf_counter()
    frame_index = 140
    geoid_height =  39.438

    with open("frame_7.txt", 'w') as f:
        pc_transformed = transform_pcap(pcap_file,meta, sbet_file, frame_index,geoid_height)
    with open("frame_7.txt", 'w') as f:
        pc_transformed_source = transform_pcap(pcap_file,meta, sbet_file, 74,geoid_height)
    source = pc_o3d_laz
    target = pc_transformed
    saved_center = source.get_center()
    downsampled_source, source_transformed,downsampled_target, target_transformed = initial_transform(source, target)
    accumulatedTime += time.perf_counter() - startTime
    print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")
    threshold = 1
    trans_init = np.identity(4)
    draw_registration_result(downsampled_source, downsampled_target, trans_init)
    transformation = np.identity(4)

    """
    for k in range(2):
        trans_init = transformation
        print("Apply point-to-plane ICP")
        startTime = time.perf_counter()
        reg_p2l = o3d.pipelines.registration.registration_icp(
            downsampled_source, downsampled_target, threshold, trans_init,
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
    """
    trans_init = draw_icp(downsampled_source, downsampled_target, transformation)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)
    trans_init = draw_icp(downsampled_source, downsampled_target, trans_init)

    trans_init = draw_icp(source_transformed, target_transformed, trans_init)
    target = copy.deepcopy(target_transformed).transform(trans_init)

    source_transformed = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
    target_transformed = copy.deepcopy(target_transformed).translate(target.get_center() + saved_center, relative=False)

    reader.draw_las(source.points, "source_test")
    reader.draw_las(target.points, "target_test")