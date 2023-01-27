

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
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_195836.laz"
    target_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"
    xyz = reader.read_laz(source_init)
    pc_o3d_laz, downsampeled_pc = reader.point_cloud_pros(xyz)

    filename = "OS-1-128_992035000186_1024x10_20211021_195836"
    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
    pcap_file = pathBase + filename + ".pcap"
    meta = pathBase + filename + ".json"
    sbet_path = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"

    gps_week = reader.get_gps_week(pcap_filename=filename)

    accumulatedTime = 0.0
    startTime = time.perf_counter()
    frame_index = 70
    geoid_height =  39.438
    with open("frame_7.txt", 'w') as f:
        pcap_test = PcapReader(pcap_file,meta, sbet_path=sbet_path)
        frame_count = pcap_test.count_frames(show_progress=True)
        pcap_test.print_info(printFunc=lambda l: f.write(l + "\n"), frame_index=frame_index)
        coord, rotate = pcap_test.get_coordinates()
        coords = np.array((coord[frame_index].x, coord[frame_index].y, coord[frame_index].alt-geoid_height))

    xyz = reader.get_frame(pcap_file, meta, frame_index)
    xyz = xyz.reshape((-1, 3))
    xyz = reader.remove_vehicle(xyz)
    pc_o3d, downsampeled_pc_PCAP = reader.point_cloud_pros(xyz)

    pc_transformed = copy.deepcopy(pc_o3d).translate(coords, relative=False)
    #
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()

    R = mesh.get_rotation_matrix_from_xyz((0, 0, rotate))
    pc_transformed.rotate(R, center=coords)

    source = pc_o3d_laz
    target = pc_transformed
    saved_center = source.get_center()
    # saved_center[2] -=39.438 #transform to NN2000
    source_center_init = source.get_center() - saved_center
    target_center_init = target.get_center() - saved_center
    # source_center_init[2] -= 39.438
    source_transformed = copy.deepcopy(source).translate(source_center_init, relative=False)
    target_transformed = copy.deepcopy(target).translate(target_center_init, relative=False)

    downsampled_source = source_transformed.voxel_down_sample(voxel_size=0.5)
    downsampled_target = target_transformed.voxel_down_sample(voxel_size=0.5)
    accumulatedTime += time.perf_counter() - startTime
    print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")

    threshold = 1
    trans_init = np.identity(4)
    draw_registration_result(downsampled_source, downsampled_target, trans_init)
    transformation = np.identity(4)
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
    trans_init = draw_icp(downsampled_source, downsampled_target, transformation)

    trans_init = draw_icp(source_transformed, target_transformed, trans_init)

    target = copy.deepcopy(target_transformed).transform(trans_init)
    # source = copy.deepcopy(source_transformed).translate(source_center_init + saved_center, relative=False)
    # target = copy.deepcopy(target_transformed).translate(target_transformed.get_center() + saved_center, relative=False)
    # trans_init = draw_icp(source, source, trans_init)

    # target = copy.deepcopy(target).transform(trans_init)

    source_transformed = copy.deepcopy(source).translate(source_center_init + saved_center, relative=False)
    target_transformed = copy.deepcopy(target).translate(target.get_center() + saved_center, relative=False)
    reader.draw_las(source_transformed.points, "test_190")
    reader.draw_las(target_transformed.points, "target_test")


    #R = mesh.get_rotation_matrix_from_xyz((0, 0, sbet_heading + np.pi))

    #pc_transformed.rotate(R, center=coords)

    """
    with open("frame_8.txt",'w') as f:
        pc_transformed_7, sbet_lat, timestamp_7 = reader.print_frame_coordinates(gps_week, pcap_file, meta, sbet_path, 10,
                                                                          printFunc=lambda l: f.write(l + "\n"))

    with open("frame_10.txt", 'w') as f:
        pc_transformed_10, sbet_lat, timestamp_10 = reader.print_frame_coordinates(gps_week, pcap_file, meta, sbet_path, 190,
                                                                            printFunc=lambda l: f.write(l + "\n"))
    """