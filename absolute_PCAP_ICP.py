import ouster.pcap as pcap
import ouster.client as client
from contextlib import closing
from more_itertools import nth
import laspy
import numpy as np
import copy
import open3d as o3d


def transform_mapprojection(crs_from=4937, crs_to=5972):
    # Function transformes the code from the correct EPSG to euref89 utm32
    # https://epsg.io/7912 EPSG 7912 is ITRF14, lat lon, height
    # https://epsg.io/5972 ETRS89 / UTM zone 32N + NN2000 height
    # https://epsg.io/4937 ETRS89 with lat lon ellipsoid

    from pyproj import Transformer
    import pandas as pd
    dayofyear = pd.Period("2021-10-21", freq="H").day_of_year
    currentepoch = int(2021) + int(dayofyear) / 365  # Current Epoch ex: 2021.45
    return Transformer.from_crs(crs_from, crs_to), currentepoch


def transform_pcap(pcap_raw, metadata, sbet_init=None, frame=None, fi=open("frame_7.txt", 'w')):

    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader
    pcap_reader = PcapReader(pcap_raw, metadata, sbet_path=sbet_init)  # Erlend Dahl Teapot
    fi = open("frame_7.txt", 'w')
    pcap_reader.print_info(printFunc=lambda l: fi.write(l + "\n"), frame_index=frame)  # Erlend Dahl Teapot
    position = pcap_reader.get_coordinates()  # Erlend Dahl Teapot
    # Collects the North, East, alt, and heading at a given frame Teapot
    lat = position[frame].lat
    lon = position[frame].lon
    time_est = position[frame].sow
    alt = position[frame].alt
    heading = position[frame].heading

    pyproj, c_epoch = transform_mapprojection(crs_from=7912) # Transform PPP from itrf14 to Euref89
    x_init, y_init, z_init, epoch_init = pyproj.transform(lat, lon, alt, c_epoch)
    import random
    center_coord_utm32 = np.array([x_init + random.uniform(-1,1), y_init + random.uniform(-1,1), z_init])
    raw_pointcloud = get_frame(pcap_raw, metadata, frame)  # PCAP Software

    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    # Removes the mobile mapping vehicle
    point_cloud_prossesing = pcap_reader.remove_vehicle(raw_pointcloud_correct_shape)  # Erlend Dahl Teapot removes vehicle
    # Remove all data outside of a 40 meters radius.
    point_cloud_prossesing = pcap_reader.remove_outside_distance(40, point_cloud_prossesing)  # Erlend Dahl Teapot
    print(f'minimum coord in numyp array {np.max(point_cloud_prossesing)}')
    pc_o3d = point_cloud_pros(point_cloud_prossesing)  # Point cloud porsessed by OPEN3ds sorftware
    # origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=(0,0,0))
    r = pc_o3d.get_rotation_matrix_from_xyz((0, 0, quadrant(heading)))  # Open3d
    pc_o3d.rotate(r, center=(0,0,0))  # open3d
    initial_origin = pc_o3d.get_center()
    pc_o3d = pc_o3d.translate([0,0,0], relative=False)  # open3d
    print(f'Center til å starte med {pc_o3d.get_center()}')

    pc_transformed_utm = pc_o3d.translate(center_coord_utm32, relative=False)  # open3d
    print(f'Center til å starte med {pc_transformed_utm.get_center()-center_coord_utm32}')

    print(r)
    return pc_transformed_utm, time_est, center_coord_utm32, heading, initial_origin


def get_frame(pcap_raw, metadata, frame):
    # Read the metadata from the JSON file.
    # Code collected by Erlend Dahl Teapot
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
    import open3d as o3
    # Process the point cloud data, so that it comes in a standardised format from open 3d.
    xyz = xyz.reshape((-1, 3))  # puts the point cloud on the correct xyz(numpy array) format
    pc_o3d = o3.geometry.PointCloud(o3.utility.Vector3dVector(xyz))  # Makes point cloud in open3d format
    pc_o3d.estimate_normals(search_param=o3.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))  # Estimates the normals

    return pc_o3d


def quadrant(heading):
    # Since the heading from the sbet files is not consistent with the quadrant. This code sets the heading in the right direction
    heading = heading - np.pi/2
    if heading < 0:
        return np.absolute(heading)
    elif heading > 0:
        return np.pi*2-heading


def initial_transform(init_source, init_target, center_coord_utm32):
    # initial center for the source point cloud. subtract this to get local plottable coordinates.
    init_center = init_source.get_center()
    source_center_init = init_source.get_center() - init_center
    print("init center")
    print(init_center - np.array([5.78927496e+05, 6.77628239e+06, 1.81186511e+02]))
    target_center_init = center_coord_utm32 - init_center
    # initial local transformation to a local coordinate system
    source_trans = copy.deepcopy(init_source).translate(source_center_init, relative=False)
    target_trans = copy.deepcopy(init_target).translate(target_center_init, relative=False)
    voxeldown_source = source_trans.voxel_down_sample(voxel_size=0.5)
    voxeldown_target = target_trans.voxel_down_sample(voxel_size=0.5)
    return voxeldown_source, source_trans, voxeldown_target, target_trans, target_center_init


def get_gps_week(pcap_path=None, pcap_filename=None):
    # Collects the filename2gpsweek from Erlend Dahl Teapot
    import os
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.sbetParser import filename2gpsweek
    if pcap_path is not None:
        pcap_filename = os.path.basename(pcap_path)
    return filename2gpsweek(pcap_filename)


def read_laz(laz_file):
    # Reads laz files to xyz(numpy arrays) format
    las = laspy.read(laz_file)
    xyz = las.xyz
    return xyz


def o3d_icp(init_source, init_target, transformation, iterations=1, threshold_value=1):
    # Runs Open3ds ICP engine, x number of iterations, and return the transformation matrix
    for i in range(iterations):
        reg_p2l = o3d.pipelines.registration.registration_icp(
            init_target, init_source, threshold_value, transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100))
        if i > 1 and np.abs(np.mean(reg_p2l.transformation-transformation)) < 1e-16:
            # print(f'ICP ran {k} times before converging.')
            transformation = reg_p2l.transformation
            break
        transformation = reg_p2l.transformation
    return transformation


def draw_las(pc_points, las_file_name):
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

    las.write(las_file_name + ".las")


def filetype(filename, system="ETPOS"):
    # Establishes which round of data is used, This is just made to shorten the code later.
    if system == "ETPOS":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
        sbet_raw = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
        return pcap_raw, metadata, sbet_raw
    elif system == "PPP":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-Standalone-PCAP\\"
        sbet_raw = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
        return pcap_raw, metadata, sbet_raw


if __name__ == "__main__":

    import sys
    import open3d as o3d
    import numpy as np
    import copy
    import time
    from collect_filename import get_files
    from cross_long_track_error import c_l_track_error
    import draw_registration
    # Grab Currrent Time Before Running the Code
    start = time.time()

    # Inputs for the data
    voxel_size = 0.5  # means 5cm for this dataset
    file_list = get_files(30, 1)  # the files from the 10th file and 5 files on # Take file nr. 17 next.
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    geoid_height = 39.438
    from_frame = 1
    to_frame = 198
    skips = 5
    sbet_prosess = "PPP"  # Choose between SBET_prosess "PPP" or "ETPOS"
    # Empty Numpy arrays, that are being filled in the for loops below
    std = []
    std_raw = []
    target_coord = []
    sbet_coord = []
    raw_coord = []
    full_sbet = []
    timesteps = []
    trans_matrix = []
    cross_track = []
    long_track = []
    direction = []
    movement_target = []
    initial_position = []
    # Source NDH
    # source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\NDH-Lillehammer.laz"
    # # Transformes the Laz file into Open 3d point cloud.
    # pc_raw_laz = read_laz(source_init)
    # pc_o3d_laz = point_cloud_pros(pc_raw_laz)
    # source = pc_o3d_laz  # Point cloud in open3d python format.

    for files in file_list:  # For loop that goes through the PCAP files, and the corresponding laz files.
        # Source_init is the raw laz file corresponding to the PCAP file
        source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + files + ".laz"

        # Transformes the Laz file into Open 3d point cloud.
        pc_raw_laz = read_laz(source_init)
        pc_o3d_laz = point_cloud_pros(pc_raw_laz)
        source = pc_o3d_laz  # Point cloud in open3d python format.

        # initializes the Sbet file as ether the PPP og ETPOS file.
        sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
        pcap_file, meta, sbet_ETPOS = filetype(files, system="ETPOS")  # Collects the correct PCAP, and metadata for the corresponding PCAP file.

        if sbet_prosess == "PPP":  # chooses if PPP or ETPOS file is being used.
            sbet_file = sbet_PPP
        else:
            sbet_file = sbet_ETPOS

        # Imports data from TEAPOT project.
        import sys
        sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
        # Imports the True Trajectory from the SBET file
        from teapot_lidar.sbetParser import SbetParser
        sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
        # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\sbet-output-UTC-1000.out"
        # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84.out"
        sbet = SbetParser(sbet_ref)

        # This for loop go through all frames choosen, with the designated skips
        for k in range(from_frame, to_frame, skips):
            frame_index = k  # Collects frame
            with open("frame_7.txt", 'w') as f:  # Transforms PCAP files to Open 3d point clouds, in the correct heading and coordinatesystem
                pc_transformed, time_sbet, init_coord, head, origo = transform_pcap(pcap_file, meta, sbet_file, frame_index)
            timesteps.append(time_sbet)  # collects all timesteps
            target = pc_transformed
            initial_position.append(init_coord)
            saved_center = source.get_center()  # Saves center for later transformation.

            # Initial transform that translate down to local coordinateframe
            downsampled_source, source_transformed, downsampled_target, target_transformed, target_center = initial_transform(source, target, init_coord)
            threshold = 1

            trans_init = np.identity(4)  # initial transformation matrix
            # import ICP_Point
            # trans_init = ICP_Point.draw_icp(source_transformed, target_transformed, trans_init)

            # At this point, both the target and the source are in a local coordinateframe
            # source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
            # draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed, target_center - origo)

            trans_init = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)  # Perform ICP on downsampled data
            trans_init = o3d_icp(source_transformed, target_transformed, trans_init, iterations=1)  # Perform ICP on the whole dataset.
            # import draw_registration

            # source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
            # draw_registration.draw_icp(source_for_plotting, target_transformed, trans_init)

            target_transformed.transform(trans_init)  # Final Transformation for the target point cloud
            movement = trans_init[0:3, 3] - origo
            # As a controll to esatblish the
            if trans_init[0, 3]*origo[0] < 0:
                movement[0] = trans_init[0, 3] + origo[0]
            elif trans_init[1, 3]*origo[1] < 0:
                movement[1] = trans_init[1, 3] + origo[1]
            movement_target.append(movement)
            print('movement')
            print(movement)
            
            source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
            draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed, target_transformed.get_center() - origo)
            trans_matrix.append(np.mean(trans_init))  # stores all transformation matrixes.
            # Below the Source and Target gets translated back to absolute coordinates. The reason for this is because of constraintes on Open 3d ICP algorithm

            source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
            target_ICP_center = init_coord + movement
            target_ICP = copy.deepcopy(target_transformed).translate(target_ICP_center, relative=False)
            print(f'target-target = {np.round(target_ICP.get_center() - target_ICP_center,4)}')

            # Get sbet coord for a gived timestep
            referance_positon = sbet.get_position_sow(time_sbet)
            true_heading = quadrant(referance_positon.heading)
            direction.append(true_heading)
            transformer, current_epoch = transform_mapprojection()
            X, Y, Z, epoch = transformer.transform(referance_positon.lat, referance_positon.lon,
                                                   referance_positon.alt, current_epoch)
            referance_coord = np.array([X, Y, Z])
            deviation = target_ICP_center - referance_coord

            # To make sure that the time step is correct, The for loop below gives a timespam for point to find the closest point to the True trajectory

            for steps in np.arange(time_sbet-0.2, time_sbet+0.2, 0.01):
                transformer, current_epoch = transform_mapprojection()
                temp_positon = sbet.get_position_sow(steps)
                X, Y, Z, epoch = transformer.transform(temp_positon.lat, temp_positon.lon,
                                                       temp_positon.alt, current_epoch)

                temp_coord = np.array([X, Y, Z])
                temp_std = target_ICP_center - temp_coord

                if np.sqrt(deviation[0]**2+deviation[1]**2) > np.sqrt(temp_std[0]**2+temp_std[1]**2):
                    referance_coord = temp_coord
                    deviation = temp_std
            cte, lte = c_l_track_error(referance_coord, target_ICP_center, true_heading)
            # Fill all numpy arrays with the correct variables for each iteration
            pre_activation = init_coord - referance_coord
            std.append(deviation)
            std_raw.append(pre_activation)
            target_coord.append(target_ICP_center)
            sbet_coord.append(referance_coord)
            raw_coord.append(init_coord)
            cross_track.append(cte)
            long_track.append(lte)


    # Reshapes the arrays,to more readable and usable data.
    sbet_full = np.reshape(full_sbet, (-1, 3))
    std = np.reshape(std, (-1, 3))
    std_raw = np.reshape(std_raw, (-1, 3))
    sbet_coord = np.reshape(sbet_coord, (-1, 3))
    target_coord = np.reshape(target_coord, (-1, 3))
    raw_coord = np.reshape(raw_coord, (-1, 3))
    initial_position = np.reshape(initial_position, (-1, 3))
    movement_target = np.reshape(movement_target, (-1, 3))

    # import matplotlib.pyplot as plt
    #
    # print(f'filename:{file}')
    # average_distance_target = np.sqrt(np.mean(std[:, 1]) ** 2 + np.mean(std[:, 0]) ** 2)
    # average_distance_before = np.sqrt(np.mean(std_raw[:, 1]) ** 2 + np.mean(std_raw[:, 0]) ** 2)
    # print(f'average distanse wrong is {average_distance_target} m, and before the registration is it {average_distance_before} m')
    # print(f'min value is x={np.min(np.abs(std[:, 0]))} y={np.min(np.abs(std[:, 1]))}')
    # print(f'max value is x={np.max(np.abs(std[:, 0]))} y={np.max(np.abs(std[:, 1]))}')
    #
    # fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    # fig.set_size_inches(18.5, 18.5, forward=True)
    # line1 = f'Estimated deviation between the cars trajectory against the true trajectory'
    # line2 = f'Average distanse error is {np.round(average_distance_target,2)} m, and before the registration: {np.round(average_distance_before,2)} m'
    # line3 = f'Based on the files {file_list}'
    # line4 = f'From frame {from_frame} to frame {to_frame} with {skips -1} skips'
    # line5 = f'min value is x={np.min(np.abs(std[:, 0]))} y={np.min(np.abs(std[:, 1]))}'
    # line6 = f'max value is x={np.max(np.abs(std[:, 0]))} y={np.max(np.abs(std[:, 1]))}'
    # fig.suptitle(line1 + '\n' + line2 + "\n" + line3 + "\n" + line4 + '\n' + line5 + '\n' + line6)
    # ax1.plot(target_coord[:, 0], target_coord[:, 1], color="green")
    # ax1.plot(sbet_coord[:, 0], sbet_coord[:, 1], color="red")
    # ax1.set_title("Prossesed target against referance", loc='center', wrap=True)
    # ax1.legend(["Target after point cloud registration", "Referance"])
    #
    # ax2.plot(raw_coord[:, 0], raw_coord[:, 1], color="green")
    # ax2.plot(sbet_coord[:, 0], sbet_coord[:, 1], color="red")
    # ax2.set_title("Raw target against referance", loc='center', wrap=True)
    # ax2.legend(["Raw target", "Referance"])
    #
    # ax3.plot(std[:, 0], color="green")
    # ax3.plot(std[:, 1], color="red")
    # ax3.set_title("Deviation between prosessed data and referance trajectory", loc='center', wrap=True)
    # ax3.legend(["North (meters)", "East (meters)"])
    #
    # ax4.plot(np.sqrt(std[:, 0]**2+std[:, 1]**2), color="green")
    # ax4.set_title("Deviation in 2d", loc='center', wrap=True)
    # ax4.legend(["Deviation (m)"])
    # fig.show()
    # fig.savefig('Estimatation' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
    #

    # Collects the first and last timestep from the frames
    min_time = timesteps[0] - 0.5
    max_time = timesteps[-1] + 0.5
    # Initialise the sbet reader
    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.sbetParser import SbetParser
    # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"

    # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"

    sbet = SbetParser(sbet_ref)
    # Produce the true trajectory with a set Hz
    true_trajectory = []
    Hz = 10
    transformer, current_epoch = transform_mapprojection()
    for k in np.arange(min_time, max_time, 1/Hz):
        temp_positon = sbet.get_position_sow(k)
        X, Y, Z, epoch = transformer.transform(temp_positon.lat, temp_positon.lon,
                                               temp_positon.alt, current_epoch)
        temp_coord = np.array([X, Y, Z])
        true_trajectory.append(temp_coord)

    true_trajectory = np.reshape(true_trajectory, (-1, 3))
    # Use K nearest neighbour to find the shortest distance between the True trajectory and the target trajectory
    import scipy
    tree = scipy.spatial.cKDTree(true_trajectory[:, 0:2])
    nearest_raw, ii = tree.query(raw_coord[:, 0:2])
    nearest_referanced, jj = tree.query(target_coord[:, 0:2])

    # Save the data as arrays
    true_filename = 'pros_data\\true_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
    sbet_filename = 'pros_data\\sbet_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
    raw_filename = 'pros_data\\raw_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
    target_filename = 'pros_data\\target_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
    np.save(true_filename, true_trajectory)
    np.save(sbet_filename, sbet_coord)
    np.save(raw_filename, raw_coord)
    np.save(target_filename, target_coord)

    #%% Plot the data as subplots.
    import matplotlib.pyplot as plt

    print(f'filename:{file_list[0]}')
    average_distance_target = np.mean(nearest_referanced)
    average_distance_before = np.mean(nearest_raw)
    print(f'average distanse wrong is {average_distance_target} m, and before the registration is it {average_distance_before} m')
    print(f'min deviation error is {np.min(nearest_referanced)} m and max:{np.max(nearest_referanced)}')

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    fig.set_size_inches(30, 30, forward=True)
    line1 = f'Estimated deviation between the cars trajectory against the true trajectory'
    std_dist = np.sqrt(std[:,0]**2+std[:,1]**2)
    line2 = f'Average distanse error is {np.round(average_distance_target,2)} m, and with the same timestamp: {np.round(np.mean(std_dist),2)} m'
    line3 = f'Based on the files {file_list}'
    line4 = f'From frame {from_frame} to frame {to_frame} with {skips -1} skips'
    line5 = f'min deviation error is {np.round(np.min(nearest_referanced),3)} m and max:{np.round(np.max(nearest_referanced),3)}m'
    line6 = f'Target trajectory = trajectory after prosessing trough point cloud registration'

    fig.suptitle(line1 + '\n' + line2 + "\n" + line3 + "\n" + line4 + '\n' + line5 + '\n' + line6)
    ax1.plot(target_coord[:, 0] - target_coord[0, 0], target_coord[:, 1] - target_coord[0, 1], color="green")
    ax1.plot(sbet_coord[:, 0]-target_coord[0, 0], sbet_coord[:, 1]-target_coord[0, 1], color="red")
    ax1.set_title("Target trajectory against true trajectory", loc='center', wrap=True)
    ax1.grid()
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.legend(["Target trajectory", "True trajectory"])

    ax2.plot(raw_coord[:, 0]-target_coord[0, 0], raw_coord[:, 1]-target_coord[0, 1], color="green")
    ax2.plot(sbet_coord[:, 0]-target_coord[0, 0], sbet_coord[:, 1]-target_coord[0, 1], color="red")
    ax2.set_title("PPP trajectory against True trajectory", loc='center', wrap=True)
    ax2.grid()
    ax2.set_xlabel("East (m)")
    ax2.set_ylabel("North (m)")
    ax2.legend(["PPP trajectory", "True trajectory"])

    # ax3.plot(std[:, 0], color="green")
    # ax3.plot(std[:, 1], color="red")
    # ax3.axhline(y=0.0, color='b', linestyle='-')
    # ax3.set_xlabel("Frames")
    # ax3.set_ylabel("Deviation (M)")
    # ax3.set_title("Deviation error between Target trajectory and True trajectory", loc='center', wrap=True)
    # ax3.legend(["North (meters)", "East (meters)"])

    dev_x = target_coord[:, 0] - raw_coord[:, 0]
    dev_y = target_coord[:, 1] - raw_coord[:, 1]
    dev_z = target_coord[:, 2] - raw_coord[:, 2]
    x_time = np.asarray(timesteps) - np.asarray(timesteps[0])
    ax3.plot(x_time, cross_track, '-bo', label="Cross Track Error")
    ax3.plot(x_time, long_track, '-ko', label="Long track error")
    # ax3.scatter(timesteps,cross_track, color="blue", label ="Cross Track Error")
    # ax3.scatter(timesteps, long_track, color = "green",label = "long track error")
    ax3.set_xlabel("Frames")
    ax3.set_ylabel("Cross Track Error")
    ax3.axhline(y=0.0, color='r', linestyle='-')
    # ax3.set_xticks(1, int(len(target_coord[:, 0])))
    ax3.set_title("Cross track and long track error", loc='center', wrap=True)
    ax3.legend()

    ax4.plot(x_time, nearest_raw, color="green")
    ax4.plot(x_time, nearest_referanced, color="blue")
    from scipy import stats
    dev = np.sqrt(std[:, 0]**2+std[:, 1]**2)
    st = []

    ax4.plot(x_time,dev, color="purple")
    ax4.plot(x_time, dev_z, color="red")
    # res = stats.linregress(timesteps, st)

    ax4.set_title("Deviation error in 2D from the true trajectory", loc='center', wrap=True)
    ax4.set_xlabel("Frames")
    ax4.set_ylabel("Deviation (m)")
    ax4.set_ylim([-1.5,1.5])
    # ax4.set_xticks(1, len(nearest_raw))
    ax4.legend(["PPP trajectory", "Nearest trajectory", "Nearest trajectory based on time","Deviation in height"])
    fig.show()
    fig.savefig('plots\\Estimatation' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
    # Grab Currrent Time After Running the Code
    end = time.time()

    # Subtract Start Time from The End Time
    total_time = end - start
    total_time = np.round(total_time, 4)
    print(f'Whoop im all done, I Have processed the files {file_list}, with the frames {from_frame} to {to_frame},with {skips} skips, and it took {total_time} seconds')
    print(f'Thats {np.round(total_time/60,2)} minutes or {np.round(total_time/(60*60),2)} hours')
    # Save info file about the script that run

    file_name = 'info_script_' + time.strftime("%Y-%m-%d-%H%M") + '_from_file ' + file_list[0]+'.txt'
    text_file = open('pros_data\\' + file_name, "w")
    text_file.write("\n")
    text_file.write(f'processed the file {file_list}, with the frames {from_frame} to {to_frame},with {skips} skips, and it took {total_time} seconds\n')
    text_file.write(f'Thats {np.round(total_time/60,2)} minutes or {np.round(total_time/(60*60),2)} hours\n')
    text_file.write(line1)
    text_file.write("\n")
    text_file.write(line2)
    text_file.write("\n")
    text_file.write(line3)
    text_file.write("\n")
    text_file.write(line4)
    text_file.write("\n")
    text_file.write(line5)
    text_file.write("\n")
    text_file.write('True trajectory filename: ' + true_filename)
    text_file.write("\n")
    text_file.write('SBET trajectory filename, that match with timestamps: ' + sbet_filename)
    text_file.write("\n")
    text_file.write('PPP trajectory filename: ' + raw_filename)
    text_file.write("\n")
    text_file.write('Georefferenced processed Target filename: ' + target_filename)
    text_file.write("\n")
    text_file.close()

