import laspy


def transform_mapprojection(crs_from=4937, crs_to=5972):
    """Transforms the coordinates from the specified EPSG to EUREF89 UTM32.
            https://epsg.io/7912 EPSG 7912 is ITRF14, lat lon, height
            https://epsg.io/5972 ETRS89 / UTM zone 32N + NN2000 height
            https://epsg.io/4937 ETRS89 with lat lon ellipsoid
    Args:
        crs_from (int, optional): The EPSG code of the original coordinate reference system. Defaults to 4937.
        crs_to (int, optional): The EPSG code of the target coordinate reference system. Defaults to 5972.

    Returns:
        A tuple containing a pyproj.Transformer object for transforming coordinates from the original CRS to the target CRS
        and the current epoch as a float, which is calculated using the day of year of October 21st in 2021.
        as this is the day the user captured the data
    """
    from pyproj import Transformer
    from pandas import Period
    dayofyear = Period("2021-10-21", freq="H").day_of_year
    currentepoch = int(2021) + int(dayofyear) / 365  # Current Epoch ex: 2021.45
    return Transformer.from_crs(crs_from, crs_to), currentepoch


def transform_pcap(pcap_raw, metadata, sbet_init, frame, init_pos, random_deviation):
    """
    Transform a point cloud from a PCAP file to UTM32 coordinates and process it using Open3D.

    Args:
        pcap_raw (str): Path to the PCAP file.
        metadata (str): Metadata associated with the PCAP file.
        sbet_init (str): Path to the SBET file.
        frame (int): The frame number of the point cloud to process.
        init_pos (dict): Initial position of the lidar in latitude, longitude, altitude, and heading.
        random_deviation (bool): If True, add a random deviation to the UTM coordinates.

    Returns:
        Tuple containing the transformed point cloud, the initial UTM32 coordinates, and the initial origin of the point cloud.

    """
    from sys import path
    from random import uniform
    from numpy import array

    path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader

    # Create PcapReader object and remove vehicle and distance outside 40 meters.
    pcap_process = PcapReader(pcap_raw, metadata, sbet_path=sbet_init)  # Erlend Dahl Teapot
    raw_pointcloud = get_frame(pcap_raw, metadata, frame)  # PCAP Software
    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    point_cloud_prossesing = pcap_process.remove_vehicle(raw_pointcloud_correct_shape)  # Erlend Dahl Teapot removes vehicle
    point_cloud_prossesing = pcap_process.remove_outside_distance(40, point_cloud_prossesing)  # Erlend Dahl Teapot

    # Process point cloud data with Open3D
    pc_o3d = point_cloud_pros(point_cloud_prossesing)
    r = pc_o3d.get_rotation_matrix_from_xyz((0, 0, quadrant(init_pos['heading'])))  # Open3d
    pc_o3d.rotate(r, center=(0, 0, 0))  # open3d
    initial_origin = pc_o3d.get_center()
    pc_o3d = pc_o3d.translate([0, 0, 0], relative=False)  # open3d

    # Apply UTM transformation
    pyproj, c_epoch = transform_mapprojection(crs_from=7912)  # Transform PPP from itrf14 to Euref89
    x_init, y_init, z_init, epoch_init = pyproj.transform(init_pos['lat'], init_pos['lon'], init_pos['alt'], c_epoch)
    if random_deviation is True:
        random_deviation = uniform(-1.5, 1.5)
    else:
        random_deviation = 0
    center_coord_utm32 = array([x_init + random_deviation, y_init + random_deviation, z_init])
    pc_transformed_utm = pc_o3d.translate(center_coord_utm32, relative=False)  # open3d

    return pc_transformed_utm, center_coord_utm32, initial_origin


def get_frame(pcap_raw, metadata, frame):
    """
    Extracts a specific frame from a LIDAR data source stored in a PCAP file.
    Code collected by Erlend Dahl Teapot
    Args:
        pcap_raw (str): Path to the PCAP file.
        metadata (str): Path to the JSON metadata file.
        frame (int): The frame number to extract.

    Returns:
        numpy.array: The point cloud data for the specified frame.
    """
    from more_itertools import nth
    from ouster.pcap import Pcap
    from ouster.client import Scans, XYZLut, SensorInfo
    from contextlib import closing
    # Read the metadata from the JSON file.
    with open(metadata, 'r') as read:
        metadata = SensorInfo(read.read())

    # Open the LIDAR data source from the PCAP file
    lidar_data = Pcap(pcap_raw, metadata)

    # Read the xth frame
    with closing(Scans(lidar_data)) as scans:
        scan = nth(scans, frame)

    # Create a function that translates coordinates to a plottable coordinate system
    xyzlut = XYZLut(lidar_data.metadata)

    # Transform point cloud to numpy array.
    pc_nparray = xyzlut(scan)

    return pc_nparray


def point_cloud_pros(xyz):
    """
    Convert point cloud data into Open3D point cloud format and estimate normals.

    Args:
    xyz (numpy.ndarray): point cloud data in numpy array format with shape (-1, 3)

    Returns:
    open3d.geometry.PointCloud: point cloud data in Open3D point cloud format with normals estimated
    """

    from open3d import geometry, utility
    # Reshape the point cloud data to the correct xyz (numpy array) format.
    xyz = xyz.reshape((-1, 3))

    # Convert the point cloud data to Open3D point cloud format.
    pc_o3d = geometry.PointCloud(utility.Vector3dVector(xyz))

    # Estimate normals of the point cloud.
    pc_o3d.estimate_normals(search_param=geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))  # Estimates the normals

    return pc_o3d


def quadrant(alpha):
    """
    Calculates the correct heading direction given the SBET heading value.

    Args:
    - alpha (float): The SBET heading value.

    Returns:
    - float: The heading direction in radians.
    """
    from numpy import pi, absolute
    alpha = alpha - pi/2
    if alpha < 0:
        return absolute(alpha)
    elif alpha > 0:
        return pi*2-alpha


def initial_transform(init_source, init_target, center_coord_utm32):
    """
    Performs transformation of point clouds to a local coordinate system and returns transformed point clouds.

    Args:
        init_source (open3d.geometry.PointCloud): Initial source point cloud.
        init_target (open3d.geometry.PointCloud): Initial target point cloud.
        center_coord_utm32 (ndarray): UTM coordinate for the center of the point cloud.

    Returns:
        tuple: A tuple containing:
            - voxeldown_source (open3d.geometry.PointCloud): Voxel downsampled source point cloud.
            - source_trans (open3d.geometry.PointCloud): Transformed source point cloud.
            - voxeldown_target (open3d.geometry.PointCloud): Voxel downsampled target point cloud.
            - target_trans (open3d.geometry.PointCloud): Transformed target point cloud.
            - target_center_init (numpy.ndarray): Initial target center.
    """
    from copy import deepcopy
    init_center = init_source.get_center()
    source_center_init = init_source.get_center() - init_center
    target_center_init = center_coord_utm32 - init_center
    # initial local transformation to a local coordinate system
    source_trans = deepcopy(init_source).translate(source_center_init, relative=False)
    target_trans = deepcopy(init_target).translate(target_center_init, relative=False)
    voxeldown_source = source_trans.voxel_down_sample(voxel_size=0.5)
    voxeldown_target = target_trans.voxel_down_sample(voxel_size=0.5)
    return voxeldown_source, source_trans, voxeldown_target, target_trans, target_center_init


def read_laz(laz_file):
    """
    Read a laz file and convert it to a numpy array format.

    Args:
        laz_file (str): The path to the laz file.

    Returns:
        numpy.ndarray: The numpy array representation of the point cloud.
    """
    # Read laz file using laspy package
    from laspy import read
    laz = read(laz_file)

    # Extract xyz coordinates from laz object
    xyz = laz.xyz
    return xyz


def o3d_icp(init_source, init_target, transformation, iterations=1, threshold_value=1):
    """
    Runs Open3D's ICP engine on the target and source point cloud and returns the transformation matrix.

    Args:
    - init_source (open3d.geometry.PointCloud): The source point cloud.
    - init_target (open3d.geometry.PointCloud): The target point cloud.
    - transformation (numpy.ndarray): The initial transformation matrix.
    - iterations (int, optional): The number of ICP iterations to run. Defaults to 1.
    - threshold_value (float, optional): The maximum correspondence distance threshold. Defaults to 1.

    Returns:
    - numpy.ndarray: The final transformation matrix.
    """
    from open3d import pipelines
    # Run ICP for the specified number of iterations
    for i in range(iterations):
        reg_p2l = pipelines.registration.registration_icp(
            init_target, init_source, threshold_value, transformation,
            pipelines.registration.TransformationEstimationPointToPlane(),
            pipelines.registration.ICPConvergenceCriteria(max_iteration=100))

        # Check for convergence
        if i > 1 and np.abs(np.mean(reg_p2l.transformation-transformation)) < 1e-16:
            transformation = reg_p2l.transformation
            break

        transformation = reg_p2l.transformation

    return transformation


def draw_las(pc_points, las_file_name):
    """Create a LAS file from a point cloud.
    draw_las created by code from https://laspy.readthedocs.io/en/latest/examples.html
    Args:
        pc_points (numpy array): Point cloud data in the form of a numpy array.
        las_file_name (str): Name of the LAS file to be created.

    Returns:
        None

    """
    # Convert numpy array to laspy data format
    from numpy import asarray, min, array, int32
    from laspy import LasHeader, LasData
    my_data = asarray(pc_points)
    header = LasHeader(point_format=3, version="1.2")
    header.add_extra_dim(laspy.ExtraBytesParams(name="random", type=int32))
    header.offsets = min(my_data, axis=0)
    header.scales = array([0.1, 0.1, 0.1])

    # 2. Create a Las
    las = LasData(header)

    las.x = my_data[:, 0]
    las.y = my_data[:, 1]
    las.z = my_data[:, 2]

    las.write(las_file_name + ".las")


def filetype(filename, system="ETPOS"):
    """Returns paths for various raw data files based on filename and system type.

    Args:
        filename (str): The filename to use for creating the path.
        system (str, optional): The type of system used to collect the data. Defaults to "ETPOS".

    Returns:
        Tuple[str, str, str]: A tuple of paths for the pcap_raw, metadata, and sbet_raw files.

    Raises:
        ValueError: If the system argument is not recognized.

    """
    if system == "ETPOS":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"

    elif system == "PPP":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-Standalone-PCAP\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
    else:
        # Raise an error if the system argument is not recognized.
        raise ValueError(f"System {system} is not recognized.")

    return pcap_raw, metadata


if __name__ == "__main__":

    import sys
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
    system_folder = "PPP"  # ETPOS system folder is the same dataset as the referance point cloud. PPP is a different round.
    file_list = get_files(25, 2, system_folder)  # the files from the 10th file and 5 files on # Take file nr. 17 next.
    from_frame = 1
    to_frame = 198
    skips = 3
    sbet_process = "PPP"  # Choose between SBET_prosess "PPP" or "ETPOS"
    standalone = True  # if True a 1.5 meters deviation is added to the sbet data.
    save_data = True
    print_point_cloud = False
    remove_outliers = False
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
    initial_coordinate = []
    """
    # Source NDH
    # source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\NDH-Lillehammer.laz"
    # Transformes the Laz file into Open 3d point cloud.
    # pc_raw_laz = read_laz(source_init)
    # pc_o3d_laz = point_cloud_pros(pc_raw_laz)
    # source_pc_numpy = pc_raw_laz  # Point cloud in open3d python format.
    """
    # Load the full point cloud used as source for the point cloud registration
    source_pc_numpy = np.load('C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\full_source_PC_np.npy')
    partial_radius = 50

    # Imports data from TEAPOT project.
    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader
    # Imports the True Trajectory from the SBET file
    from teapot_lidar.sbetParser import SbetParser
    sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
    sbet = SbetParser(sbet_ref)

    # initializes the Sbet file as ether the PPP og ETPOS file.
    sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
    sbet_ETPOS = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
    if sbet_process == "PPP":  # chooses if PPP or ETPOS file is being used.
        initial_navigation_trajectory = sbet_PPP
    else:
        initial_navigation_trajectory = sbet_ETPOS

    for files in file_list:  # Iterate through the PCAP files to

        pcap_file, meta = filetype(files, system_folder)  # Collects the correct PCAP, and metadata for the corresponding PCAP file.

        # Iterate through all selected frames with the designated skips.
        for frame_index in range(from_frame, to_frame, skips):
            start_frame = time.time()

            pcap_reader = PcapReader(pcap_file, meta, sbet_path=initial_navigation_trajectory)  # Erlend Dahl Teapot
            fi = open("frame.txt", 'w')
            pcap_reader.print_info(printFunc=lambda l: fi.write(l + "\n"), frame_index=frame_index)  # Erlend Dahl Teapot
            position = pcap_reader.get_coordinates()  # Erlend Dahl Teapot
            if frame_index >= len(position)-1:
                print('List index is out of range,PCAP file')
                break
            # Collects the North, East, alt, and heading at a given frame Teapot
            initial_position = {'lat': position[frame_index].lat, 'lon': position[frame_index].lon,
                                'time_est': position[frame_index].sow, 'alt': position[frame_index].alt,
                                'heading': position[frame_index].heading}

            # with open("frame_7.txt", 'w') as f:  # Transforms PCAP files to Open 3d point clouds, in the correct heading and coordinatesystem
            pc_transformed, init_coord, origo = transform_pcap(pcap_file, meta, initial_navigation_trajectory, frame_index, initial_position, standalone)
            timesteps.append(initial_position['time_est'])  # collects all timesteps

            # Get sbet coord for a gived timestep
            referance_positon = sbet.get_position_sow(initial_position['time_est'])
            true_heading = quadrant(referance_positon.heading)
            direction.append(true_heading)
            transformer, current_epoch = transform_mapprojection()
            X, Y, Z, epoch = transformer.transform(referance_positon.lat, referance_positon.lon,
                                                   referance_positon.alt, current_epoch)
            referance_coord = np.array([X, Y, Z])
            target = pc_transformed
            initial_coordinate.append(init_coord)

            # Get source pc
            partial_radius = 50

            points = source_pc_numpy[
                (source_pc_numpy[:, 0] >= referance_coord[0] - partial_radius) & (
                            source_pc_numpy[:, 0] <= referance_coord[0] + partial_radius) & (
                        source_pc_numpy[:, 1] >= referance_coord[1] - partial_radius) & (
                        source_pc_numpy[:, 1] <= referance_coord[1] + partial_radius)]  # Erlend Dahl
            # If the source point cloud is empty this file is not in the dataset, and next file is collected
            if points.size == 0:
                break
            source = point_cloud_pros(points)

            saved_center = source.get_center()  # Saves center for later transformation.
            # Initial transform that translate down to local coordinateframe
            downsampled_source, source_transformed, downsampled_target, target_transformed, target_center = initial_transform(source, target, init_coord)
            threshold = 1

            trans_init = np.identity(4)  # initial transformation matrix

            trans_init = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)  # Perform ICP on downsampled data
            trans_init = o3d_icp(source_transformed, target_transformed, trans_init, iterations=1)  # Perform ICP on the whole dataset.

            target_transformed.transform(trans_init)  # Final Transformation for the target point cloud
            movement = trans_init[0:3, 3] - origo
            # As a controll to esatblish the

            if trans_init[0, 3]*origo[0] < 0:
                movement[0] = trans_init[0, 3] + origo[0]
            elif trans_init[1, 3]*origo[1] < 0:
                movement[1] = trans_init[1, 3] + origo[1]

            movement_target.append(movement)
            # Plotting the target point cloud against the source point cloud
            if print_point_cloud is True:
                source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
                draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed, target_transformed.get_center() - origo)

            trans_matrix.append(np.mean(trans_init))  # stores all transformation matrixes.
            # Below the Source and Target gets translated back to absolute coordinates. The reason for this is because of constraintes on Open 3d ICP algorithm

            source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
            target_ICP_center = target_transformed.get_center() + saved_center - origo
            # target_ICP_center = init_coord + movement
            target_ICP = copy.deepcopy(target_transformed).translate(target_ICP_center, relative=False)

            deviation = target_ICP_center - referance_coord
            # To make sure that the time step is correct, The for loop below gives a timespam for point to find the closest point to the True trajectory

            # Remove outlisers, if the point cloud registration says to move more than 3 times the standard deviation
            # of the initial coordinate, the initial coordinate is initiates as the true coordinate.
            if remove_outliers is True:
                dev_2d = np.sqrt((target_ICP_center[0]-init_coord[0])**2+(target_ICP_center[1]-init_coord[1])**2)
                if standalone is True and dev_2d >= 4*1.5:
                    target_ICP_center = init_coord
                elif dev_2d >= 4*0.5:
                    target_ICP_center = init_coord

            cte, lte = c_l_track_error(referance_coord, target_ICP_center, true_heading)

            for steps in np.arange(initial_position['time_est']-0.2, initial_position['time_est']+0.2, 0.01):
                transformer, current_epoch = transform_mapprojection()
                temp_positon = sbet.get_position_sow(steps)
                X, Y, Z, epoch = transformer.transform(temp_positon.lat, temp_positon.lon,
                                                       temp_positon.alt, current_epoch)

                temp_coord = np.array([X, Y, Z])
                temp_std = target_ICP_center - temp_coord

                if np.sqrt(deviation[0]**2+deviation[1]**2) > np.sqrt(temp_std[0]**2+temp_std[1]**2):
                    referance_coord = temp_coord
                    deviation = temp_std

            # Fill all numpy arrays with the correct variables for each iteration
            pre_activation = init_coord - referance_coord
            std.append(deviation)
            std_raw.append(pre_activation)
            target_coord.append(target_ICP_center)
            sbet_coord.append(referance_coord)
            raw_coord.append(init_coord)
            cross_track.append(cte)
            long_track.append(lte)
            end_frame = time.time()
            total_frame = end_frame - start_frame
            print(f'It takes {total_frame} s per frame')

    # Reshapes the arrays,to more readable and usable data.
    sbet_full = np.reshape(full_sbet, (-1, 3))
    std = np.reshape(std, (-1, 3))
    std_raw = np.reshape(std_raw, (-1, 3))
    sbet_coord = np.reshape(sbet_coord, (-1, 3))
    target_coord = np.reshape(target_coord, (-1, 3))
    raw_coord = np.reshape(raw_coord, (-1, 3))
    initial_coordinate = np.reshape(initial_coordinate, (-1, 3))
    movement_target = np.reshape(movement_target, (-1, 3))

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
    std_dist = np.sqrt(std[:, 0]**2+std[:, 1]**2)
    line2 = f'Average distanse error is {np.round(average_distance_target,2)} m, and before the registration: {np.round(average_distance_before,2)} m'
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

    dev_x = target_coord[:, 0] - raw_coord[:, 0]
    dev_y = target_coord[:, 1] - raw_coord[:, 1]
    dev_z = target_coord[:, 2] - raw_coord[:, 2]
    x_time = np.asarray(timesteps) - np.asarray(timesteps[0])
    ax3.plot(x_time, cross_track, '-bo', label="Cross Track Error")
    ax3.plot(x_time, long_track, '-ko', label="Long track error")
    # ax3.scatter(x_time, cross_track, color="blue", label="Cross Track Error")
    # ax3.scatter(x_time, long_track, color="green", label="long track error")
    ax3.set_xlabel("Frames")
    ax3.set_ylabel("Cross Track Error")
    ax3.set_ylim([-1.5, 1.5])
    ax3.axhline(y=0.0, color='r', linestyle='-')
    # ax3.set_xticks(1, int(len(target_coord[:, 0])))
    ax3.set_title("Cross track and long track error", loc='center', wrap=True)
    ax3.legend()

    ax4.plot(x_time, nearest_raw, color="green")
    ax4.plot(x_time, nearest_referanced, color="blue")

    dev = np.sqrt(std[:, 0]**2+std[:, 1]**2)
    st = []

    ax4.plot(x_time, dev, color="purple")
    ax4.plot(x_time, dev_z, color="red")
    # res = stats.linregress(timesteps, st)

    ax4.set_title("Deviation error in 2D from the true trajectory", loc='center', wrap=True)
    ax4.set_xlabel("Frames")
    ax4.set_ylabel("Deviation (m)")
    ax4.set_ylim([-1.5, 1.5])
    ax4.axhline(y=0.0, color='r', linestyle='-')
    # ax4.set_xticks(1, len(nearest_raw))
    ax4.legend(["PPP trajectory", "Nearest trajectory", "Nearest trajectory based on time", "Deviation in height"])
    fig.show()
    # Grab Currrent Time After Running the Code
    end = time.time()

    # Subtract Start Time from The End Time
    total_time = end - start
    total_time = np.round(total_time, 4)
    print(f'Whoop im all done, I Have processed the files {file_list}, with the frames {from_frame} to {to_frame},with {skips} skips, and it took {total_time} seconds')
    print(f'Thats {np.round(total_time/60,2)} minutes or {np.round(total_time/(60*60),2)} hours')
    # Save info file about the script that run

    if save_data is True:

        fig.savefig('plots\\Estimatation' + time.strftime("%Y-%m-%d %H%M%S") + '.png')

        # Save the data as arrays
        true_filename = 'pros_data\\true_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        sbet_filename = 'pros_data\\sbet_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        raw_filename = 'pros_data\\raw_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        target_filename = 'pros_data\\target_coord_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        heading_filename = 'pros_data\\heading_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        np.save(heading_filename, direction)
        np.save(true_filename, true_trajectory)
        np.save(sbet_filename, sbet_coord)
        np.save(raw_filename, raw_coord)
        np.save(target_filename, target_coord)

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
