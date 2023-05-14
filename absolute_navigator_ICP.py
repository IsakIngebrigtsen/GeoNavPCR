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


def transform_pcap(pcap_raw, metadata, sbet_init, frame, init_pos, random_deviation, crs_epsg, seed_input):
    """
    Transform a point cloud from a PCAP file to UTM32 coordinates and process it using Open3D.

    Args:
        pcap_raw (str): Path to the PCAP file.
        metadata (str): Metadata associated with the PCAP file.
        sbet_init (str): Path to the SBET file.
        frame (int): The frame number of the point cloud to process.
        init_pos (dict): Initial position of the lidar in latitude, longitude, altitude, and heading.
        random_deviation (bool): If True, add a random deviation to the UTM coordinates.
        crs_epsg (int): EPSG value for the initial transformation of initial coordinates
        seed_input (int): initial Seed value of the parameters
    Returns:
        Tuple containing the transformed point cloud, the initial UTM32 coordinates, and the initial origin of the point cloud.

    """
    from sys import path
    from numpy import array, random

    path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader

    # Create PcapReader object and remove vehicle and distance outside 40 meters.
    pcap_process = PcapReader(pcap_raw, metadata, sbet_path=sbet_init)  # Erlend Dahl Teapot
    raw_pointcloud = get_frame(pcap_raw, metadata, frame)  # PCAP Software
    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    point_cloud_prossesing = pcap_process.remove_vehicle(raw_pointcloud_correct_shape)  # Erlend Dahl Teapot removes vehicle
    point_cloud_prossesing = pcap_process.remove_outside_distance(35, point_cloud_prossesing)  # Erlend Dahl Teapot

    # Process point cloud data with Open3D
    pc_o3d = point_cloud_pros(point_cloud_prossesing)
    r = pc_o3d.get_rotation_matrix_from_xyz((0, 0, quadrant(init_pos['heading'])))  # Open3d
    pc_o3d.rotate(r, center=(0, 0, 0))  # open3d
    initial_origin = pc_o3d.get_center()
    pc_o3d = pc_o3d.translate([0, 0, 0], relative=False)  # open3d

    # Apply UTM transformation
    pyproj, e = transform_mapprojection(crs_from=crs_epsg)  # Transform PPP from itrf14 to Euref89
    e_init, n_init, h_init, ep_init = pyproj.transform(init_pos['lat'], init_pos['lon'], init_pos['alt'], e)
    if random_deviation is True:
        # Put in a seed to get a consistent result.
        random.seed(seed_input)
        random_deviation_e = random.normal(0, 1.5)
        random_deviation_n = random.normal(0, 1.5)
        # random_deviation_alt = random.normal(0, 1.5)
    else:
        random_deviation_e = 0
        random_deviation_n = 0
    center_coord_utm32 = array([e_init + random_deviation_e, n_init + random_deviation_n, h_init])
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
        # Frame -1 to get the correct frame from the pcap vs sbet file.
        scan = nth(scans, frame-1)

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


def o3d_icp(init_source, init_target, transformation, iterations=1, threshold_value=1, model="Point2Plane"):

    """
    Runs Open3D's ICP engine on the target and source point cloud and returns the transformation matrix.

    Args:
    - init_source (open3d.geometry.PointCloud): The source point cloud.
    - init_target (open3d.geometry.PointCloud): The target point cloud.
    - transformation (numpy.ndarray): The initial transformation matrix.
    - iterations (int, optional): The number of ICP iterations to run. Defaults to 1.
    - threshold_value (float, optional): The maximum correspondence distance threshold. Defaults to 1.
    - model(str): The ICP algorithm used in the point cloud registration: either "Point2Plane" or "Point2Point"
    Returns:
    - numpy.ndarray: The final transformation matrix.
    - numpy.ndarray: The inlier RMSE value for the point cloud registration
    """

    from open3d import pipelines
    # Run ICP for the specified number of iterations
    convergence = pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    if model == "Point2Point":
        icp_algorithm = pipelines.registration.TransformationEstimationPointToPoint()
    elif model == "RANSAC":
        target_down, target_fpfh = preprocess_point_cloud(init_target)
        source_down, source_fpfh = preprocess_point_cloud(init_source)
        reg_p2l = execute_global_registration(target_down, source_down, target_fpfh,
                                              source_fpfh, voxel=0.5)
        transformation = reg_p2l.transformation
        reg_p2l = pipelines.registration.registration_icp(
            init_target, init_source, threshold_value, transformation, pipelines.registration.TransformationEstimationPointToPlane(), convergence)
        return transformation, reg_p2l.inlier_rmse
    else:
        print('Point2plane')
        icp_algorithm = pipelines.registration.TransformationEstimationPointToPlane()

    reg_p2l = pipelines.registration.registration_icp(
        init_target, init_source, threshold_value, transformation, icp_algorithm, convergence)

    for i in range(iterations - 1):
        reg_p2l = pipelines.registration.registration_icp(
            init_target, init_source, threshold_value, transformation, icp_algorithm, convergence)
        # Check for convergence
        if i > 1 and np.abs(np.mean(reg_p2l.transformation[0:3, 3]-transformation[0:3, 3])) < 1e-5:
            transformation = reg_p2l.transformation
            print(f'It did {i} number of iterations')
            break
        transformation = reg_p2l.transformation

    return transformation, reg_p2l.inlier_rmse


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
    header.scales = array([0.001, 0.001, 0.001])

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
    if system == "Round1":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = data_path + "Raw_Frames_Round_1\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
    elif system == "Round2":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = data_path + "Raw_Frames_Round_2\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
    elif system == "Round3":
        raw_file = "OS-1-128_992035000186_1024x20_20211020_" + filename
        pathbase = data_path + "Dovre\\pcap\\1_ned_20hz\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"
    elif system == "Round4":
        raw_file = "OS-1-128_992035000186_1024x20_20211020_" + filename
        pathbase = data_path + "Dovre\\pcap\\1_opp_20hz\\"
        pcap_raw = pathbase + raw_file + ".pcap"
        metadata = pathbase + raw_file + ".json"

    else:
        # Raise an error if the system argument is not recognized.
        raise ValueError(f"System {system} is not recognized.")

    return pcap_raw, metadata


def get_coordinate(pcap, frame_i=None):

    original = sys.stdout  # Save a reference to the original standard output
    if frame_i is None:
        with open("frame.txt", 'w') as fi:
            sys.stdout = fi  # Change the standard output to the file we created.
            pcap.print_info(printFunc=print, frame_index=1)  # Erlend Dahl Teapot
            sys.stdout = original  # Save a reference to the original standard output
        coord = pcap.get_coordinates(rotate=False)  # Erlend Dahl Teapot
        frame_i = int(len(coord)/2)
        init_pos = {'lat': coord[frame_i].lat, 'lon': coord[frame_i].lon,
                    'time_est': coord[frame_i].sow, 'alt': coord[frame_i].alt,
                    'heading': coord[frame_i].heading}
        return init_pos

    else:
        with open("frame.txt", 'w') as fi:
            sys.stdout = fi  # Change the standard output to the file we created.
            pcap.print_info(printFunc=print, frame_index=frame_i)  # Erlend Dahl Teapot
            sys.stdout = original  # Save a reference to the original standard output
        # Change the standard output to the file we created.
        coord = pcap.get_coordinates(rotate=False)  # Erlend Dahl Teapot
        if frame_i >= len(coord) - 1:
            print('List index is out of range,PCAP file {}')
            return False

        # Collects the North, East, alt, and heading at a given frame Teapot
        else:
            init_pos = {'lat': coord[frame_i].lat, 'lon': coord[frame_i].lon,
                        'time_est': coord[frame_i].sow, 'alt': coord[frame_i].alt,
                        'heading': coord[frame_i].heading}
            return init_pos

# RANSAC TEST


def preprocess_point_cloud(pcd, voxel=0.5):
    # Open 3d data
    import open3d as o3d
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel)

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


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel=0.5):
    import open3d as o3d
    # Open3d
    distance_threshold = voxel * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


if __name__ == "__main__":

    import sys
    import numpy as np
    import copy
    import time
    from collect_filename import get_files
    from Quality_Controll import c_l_track_error, root_mean_square, area_between_trajectories
    import draw_registration
    import open3d.cpu.pybind
    # Grab Currrent Time Before Running the Code
    start = time.time()

    # Inputs for the data!
    voxel_size = 0.5  # means 50cm for this dataset
    # Data path to the data ran in this program. use the structure from
    data_path = 'C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\'
    # Path to teapot_lidar project. Code forked with the hash f1e8d6ba6d9a0003ecc4630a878518c3778dabf4, with some minor
    # adjustments. Version used can be pulled from https://github.com/IsakIngebrigtsen/teapot-lidar
    data_path_teapot_lidar = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\master_code\\Master_thesis\\teapot_lidar"
    Area = "Lillehammer" # Lillehammer, and Dovre are areas that can be testet
    system_folder = "Round1"  # Round 1, and Round 2 is for Lillehammer, and Round 3 and Round 4 is for dovre.
    section = "Full"  # Full, Forest, Rural, Dense
    number_of_files = 1
    file_list = get_files(18, number_of_files, system_folder)  # the files from the 10th file and 5 files on # Take file nr. 17 next.
    from_frame = 10
    to_frame = 20
    skips = 4
    handle_outliers = True  # If True, the outliers get removed.
    algorithm = "Point2Plane"  # It is possible to change between Point-to-Point and Point-To-Plane
    sbet_process = "PPP"  # Choose between SBET_prosess "PPP" or "ETPOS"
    standalone = True  # if True a 1.5 meters deviation is added to the sbet data.
    save_data = True  # True and the data gets saved
    print_point_cloud = True  # True, and every PCR alignment gets printed as visualisations

    seed = 1
    total_number_of_frames = number_of_files * np.round((to_frame - from_frame + 1) / skips, 0)
    import sys
    sys.path.insert(0, data_path_teapot_lidar)
    # Empty Numpy arrays, that are being filled in the for loops below
    std = []
    std_raw = []
    target_coord = []
    true_coord = []
    initial_coord = []
    full_sbet = []
    timesteps = []
    trans_matrix = []
    cross_track = []
    long_track = []
    direction = []
    movement_target = []
    initial_coordinate = []
    outliers = {}
    inlier_rms = []
    num_outliers = 0
    frames_finished = 0
    removed_outliers = False
    target_pointcloud = open3d.cpu.pybind.geometry.PointCloud()
    source_pointcloud = open3d.cpu.pybind.geometry.PointCloud()
    """
    # Source NDH
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\NDH-Lillehammer.laz"
    # Transformes the Laz file into Open 3d point cloud.
    pc_raw_laz = read_laz(source_init)
    pc_o3d_laz = point_cloud_pros(pc_raw_laz)
    source_pc_numpy = pc_raw_laz  # Point cloud in open3d python format.
    """
    # Load the full point cloud used as source for the point cloud registration
    if Area == "Lillehammer":
        source_pc_numpy = np.load(data_path+'Referansepunktsky-LAZ\\full_source_PC_np.npy')
        # Imports data from TEAPOT project.
        from teapot_lidar.pcapReader import PcapReader
        # Imports the True Trajectory from the SBET file
        from teapot_lidar.sbetParser import SbetParser
        true_trajectory_SBET = data_path + "Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
        # sbet = SbetParser(true_trajectory_SBET)

        # initializes the Sbet file as ether the PPP og ETPOS file.
        sbet_PPP = data_path + "Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
        sbet_ETPOS = data_path + "Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
    else:
        source_pc_numpy = np.load(data_path+'Referansepunktsky-LAZ\\dovre_full_source_pc.npy')
        # Imports data from TEAPOT project.
        from teapot_lidar.pcapReader import PcapReader
        # Imports the True Trajectory from the SBET file
        from teapot_lidar.sbetParser import SbetParser
        true_trajectory_SBET = data_path + "Dovre\\navigation\\realtime-sbet-output-UTC-1000.out"
        # sbet = SbetParser(true_trajectory_SBET)

        # initializes the Sbet file as ether the PPP og ETPOS file.
        sbet_PPP = data_path + "Dovre\\navigation\\sbet-output-UTC-1000.out"
        sbet_ETPOS = data_path + "Dovre\\navigation\\realtime-sbet-output-UTC-1000.out"

    if sbet_process == "PPP":  # chooses if PPP or ETPOS file is being used.
        initial_navigation_trajectory = sbet_PPP
        EPSG = 7912
    else:
        initial_navigation_trajectory = sbet_ETPOS
        EPSG = 4937
    for files in file_list:  # Iterate through the PCAP files to
        frame_outlier_list = []
        outlier = False
        pcap_file, meta = filetype(files, system_folder)  # Collects the correct PCAP, and metadata for the corresponding PCAP file.
        pcap_reader = PcapReader(pcap_file, meta, sbet_path=initial_navigation_trajectory)  # Erlend Dahl Teapot

        center_coord_init_source = get_coordinate(pcap_reader)
        # To speed up the cutting of the large point cloud, do an initial crop per file, to optimize the speeed.
        partial_radius = 320
        crop_coord, c_epoch = transform_mapprojection(crs_from=7912)
        east_init, north_init, alt_init, epoch_init = crop_coord.transform(center_coord_init_source['lat'], center_coord_init_source['lon'],
                                                                           center_coord_init_source['alt'], c_epoch)

        init_part_of_source_np = source_pc_numpy[
            (source_pc_numpy[:, 0] >= east_init - partial_radius) & (
                    source_pc_numpy[:, 0] <= east_init + partial_radius) & (
                    source_pc_numpy[:, 1] >= north_init - partial_radius) & (
                    source_pc_numpy[:, 1] <= north_init + partial_radius)]  # Erlend Dahl

        # Iterate through all selected frames with the designated skips.
        for frame_index in range(from_frame, to_frame, skips):
            frames_finished += 1

            # implment seed for each frame
            seed += 1
            start_frame = time.time()

            initial_position = get_coordinate(pcap_reader, frame_index)
            if initial_position is False:
                break

            # with open("frame_7.txt", 'w') as f:  # Transforms PCAP files to Open 3d point clouds, in the correct heading and coordinatesystem
            pc_transformed, init_coord, origo = transform_pcap(pcap_file, meta, initial_navigation_trajectory, frame_index, initial_position, standalone, EPSG, seed)
            target = pc_transformed

            # Get source pc
            partial_radius = 50
            start_np = time.time()
            part_of_source_np = init_part_of_source_np[
                (init_part_of_source_np[:, 0] >= init_coord[0] - partial_radius) & (
                            init_part_of_source_np[:, 0] <= init_coord[0] + partial_radius) & (
                        init_part_of_source_np[:, 1] >= init_coord[1] - partial_radius) & (
                        init_part_of_source_np[:, 1] <= init_coord[1] + partial_radius)]  # Erlend Dahl

            # If the source point cloud is empty this file is not in the dataset, and next file is collected
            if part_of_source_np.size == 0:
                break
            source = point_cloud_pros(part_of_source_np)
            # Just to initialize a point cloud incase the first frame does not work
            saved_center = source.get_center()  # Saves center for later transformation.
            last_frame = source
            # Initial transform that translate down to local coordinateframe
            downsampled_source, source_transformed, downsampled_target, target_transformed, target_center = initial_transform(source, target, init_coord)
            threshold = 1
            # source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.2)
            # draw_registration.draw_absolute_registration_result(source_for_plotting, target_transformed,

            #                                                    target_transformed.get_center()-origo)
            trans_init = np.identity(4)  # initial transformation matrix

            trans_init, rmse = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9, model=algorithm)  # Perform ICP on downsampled data
            trans_init, init_inlier_rmse = o3d_icp(source_transformed, target_transformed, trans_init, iterations=1, model=algorithm)  # Perform ICP on the whole dataset.
            print(f'inlier_RMSE :{np.round(init_inlier_rmse,3)}')
            inlier_rms.append(init_inlier_rmse)
            # Remove outlisers, if the point cloud registration says to move more than 3 times the standard deviation
            # of the initial coordinate, the initial coordinate is initiates as the true coordinate.
            if handle_outliers is True:
                if init_inlier_rmse > 0.20:
                    print(f'Warning: high RMSE value, continue ICP to converge for frame {frame_index} in file {files}')

                    trans_init, rmse = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9,
                                               model=algorithm)  # Perform ICP on downsampled data
                    trans_init, inlier_rmse = o3d_icp(source_transformed, target_transformed, trans_init, iterations=1,
                                                      model=algorithm)  # Perform ICP on the whole dataset.
                    print(f'inlier_RMSE :{np.round(inlier_rmse,3)}')
                    if inlier_rmse >= init_inlier_rmse-0.03 or inlier_rmse >= 0.25:
                        print('This is an outlier')
                        frame_outlier = [frame_index, f'Inlier RMSE: {np.round(inlier_rmse,3)}']
                        frame_outlier_list.append(frame_outlier)
                        outlier = True
                        num_outliers += 1
                        removed_outliers = True
                        continue
            else:
                if init_inlier_rmse > 0.20:
                    print('This is an outlier')
                    frame_outlier = [frame_index, f'Inlier RMSE: {np.round(init_inlier_rmse, 3)}']
                    frame_outlier_list.append(frame_outlier)
                    outlier = True
                    num_outliers += 1

            # Save the initial coordinate and the timesteps
            timesteps.append(initial_position['time_est'])  # collects all timesteps
            initial_coordinate.append(init_coord)
            target_transformed.transform(trans_init)  # Final Transformation for the target point cloud

            movement = trans_init[0:3, 3] - origo
            # As a controll to esatblish the

            if trans_init[0, 3]*origo[0] < 0:
                movement[0] = trans_init[0, 3] + origo[0]
            elif trans_init[1, 3]*origo[1] < 0:
                movement[1] = trans_init[1, 3] + origo[1]

            movement_target.append(movement)
            # Plotting the target point cloud against the source point cloud

            # source_for_plotting = source_transformed.voxel_down_sample(voxel_size=0.1)
            # draw_registration.draw_absolute_registration_result(source_for_plotting, target_pointcloud, target_transformed.get_center() - origo)

            trans_matrix.append(np.mean(trans_init))  # stores all transformation matrixes.
            # Below the Source and Target gets translated back to absolute coordinates. The reason for this is because of constraintes on Open 3d ICP algorithm

            source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
            target_ICP_center = target_transformed.get_center() + saved_center - origo

            if print_point_cloud is True:
                target_ICP = copy.deepcopy(target_transformed).translate(target_ICP_center, relative=False)
                target_ICP_2 = copy.deepcopy(target_transformed).translate(target_transformed.get_center() + saved_center, relative=False)
                target_pointcloud += target_ICP_2
                source_pointcloud += source
            # Get sbet coord for a gived timestep
            pcap_reader_true = PcapReader(pcap_file, meta, sbet_path=true_trajectory_SBET)  # Erlend Dahl Teapot
            true_position = get_coordinate(pcap_reader_true, frame_index)
            """
            with open("frame.txt", 'w') as f:
                sys.stdout = f  # Change the standard output to the file we created.
                pcap_reader.print_info(printFunc=print, frame_index=frame_index)  # Erlend Dahl Teapot
                sys.stdout = original_stdout  # Save a reference to the original standard output
            true_coord = pcap_reader_true.get_coordinates(rotate=False)  # Erlend Dahl Teapot
            # Collects the North, East, alt, and heading at a given frame Teapot

            true_position = {'lat': true_coord[frame_index].lat, 'lon': true_coord[frame_index].lon,
                             'time_est': true_coord[frame_index].sow, 'alt': true_coord[frame_index].alt,
                             'heading': true_coord[frame_index].heading}
            """
            # true_coordinates_lat_lon = sbet.get_position_sow(initial_position['time_est'])
            true_heading = quadrant(true_position['heading'])
            direction.append(true_heading)
            transformer, current_epoch = transform_mapprojection(crs_from=4937)
            East_true, North_true, alt_true, epoch = transformer.transform(true_position['lat'], true_position['lon'],
                                                                           true_position['alt'], current_epoch)
            true_coordinates_UTM = np.array([East_true, North_true, alt_true])

            deviation = target_ICP_center - true_coordinates_UTM

            cte, lte = c_l_track_error(true_coordinates_UTM, target_ICP_center, true_heading)

            # Fill all numpy arrays with the correct variables for each iteration
            pre_activation = init_coord - true_coordinates_UTM
            std.append(deviation)
            std_raw.append(pre_activation)
            target_coord.append(target_ICP_center)
            true_coord.append(true_coordinates_UTM)
            initial_coord.append(init_coord)
            cross_track.append(cte)
            long_track.append(lte)
            end_frame = time.time()
            total_frame = end_frame - start_frame
            print(f'It takes {np.round(total_frame,3)} s per frame')
            print(f'it is {np.round((frames_finished / total_number_of_frames)*100, 3)} % finished')
            print(f'Estimated {np.round((total_number_of_frames-frames_finished)*13/60,2)} minutes left')
        if outlier is True:
            outliers[files] = frame_outlier_list

    # Reshapes the arrays,to more readable and usable data.
    if print_point_cloud is True:
        target_pointcloud = copy.deepcopy(target_pointcloud).translate(target_pointcloud.get_center()-target_ICP_center, relative=False)
        source_pointcloud = copy.deepcopy(source_pointcloud).translate(source_pointcloud.get_center()-target_ICP_center, relative=False)
        source_for_plotting = source_pointcloud.voxel_down_sample(voxel_size=0.2)
        draw_registration.draw_absolute_registration_result(source_for_plotting, target_pointcloud,
                                                            target_pointcloud.get_center() - origo)
    sbet_full = np.reshape(full_sbet, (-1, 3))
    std = np.reshape(std, (-1, 3))
    std_raw = np.reshape(std_raw, (-1, 3))
    true_coord = np.reshape(true_coord, (-1, 3))
    target_coord = np.reshape(target_coord, (-1, 3))
    initial_coord = np.reshape(initial_coord, (-1, 3))
    initial_coordinate = np.reshape(initial_coordinate, (-1, 3))
    movement_target = np.reshape(movement_target, (-1, 3))

    # Collects the first and last timestep from the frames
    min_time = timesteps[0] - 0.5
    max_time = timesteps[-1] + 0.5
    # Initialise the sbet reader
    import sys
    sys.path.insert(0, data_path_teapot_lidar)
    from teapot_lidar.sbetParser import SbetParser
    # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    true_trajectory_SBET = data_path + "Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"

    # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"

    sbet = SbetParser(true_trajectory_SBET)
    # Produce the true trajectory with a set Hz
    """
    true_trajectory = []
    Hz = 10
    transformer, current_epoch = transform_mapprojection()
    for k in np.arange(min_time, max_time, 1/Hz):
        temp_positon = sbet.get_position_sow(k)
        X_true, Y_true, Z_true, epoch = transformer.transform(temp_positon.lat, temp_positon.lon,
                                                              temp_positon.alt, current_epoch)
        temp_coord = np.array([X_true, Y_true, Z_true])
        true_trajectory.append(temp_coord)
    
    true_trajectory = np.reshape(true_trajectory, (-1, 3))
    """
    # Use K nearest neighbour to find the shortest distance between the True trajectory and the target trajectory
    import scipy
    tree = scipy.spatial.cKDTree(true_coord[:, 0:2])
    nearest_raw, ii = tree.query(initial_coord[:, 0:2])
    nearest_referanced, jj = tree.query(target_coord[:, 0:2])

    rms_n_init, rms_e_init, rms_alt_init = np.round(root_mean_square(initial_coord, true_coord), 2)
    rms_n_target, rms_e_target, rms_alt_target = np.round(root_mean_square(target_coord, true_coord), 2)
    rms_n_target_v_init, rms_e_target_v_init, rms_alt_target_v_init = np.round(root_mean_square(initial_coord, target_coord), 2)
    area_target_init_traj = np.round(area_between_trajectories(initial_coord, target_coord), 2)
    area_target_source = np.round(area_between_trajectories(target_coord, true_coord), 2)
    area_init_traj_source = np.round(area_between_trajectories(initial_coord, true_coord), 2)

    dev_plane_target_source = np.round(np.percentile(np.sqrt((target_coord[:, 0] - true_coord[:, 0]) ** 2 + (target_coord[:, 2] - true_coord[:, 2]) ** 2), 95), 2)
    dev_height_target_source = np.round(np.percentile(np.sqrt((target_coord[:, 2] - true_coord[:, 2]) ** 2), 95), 2)

    dev_plane_init_source = np.round(np.percentile(np.sqrt((initial_coord[:, 0] - true_coord[:, 0]) ** 2 + (initial_coord[:, 2] - true_coord[:, 2]) ** 2), 95), 2)
    dev_height_init_source = np.round(np.percentile(np.sqrt((initial_coord[:, 2] - true_coord[:, 2]) ** 2), 95), 2)

    dev_plane_init_target = np.round(np.percentile(np.sqrt((initial_coord[:, 0] - target_coord[:, 0]) ** 2 + (initial_coord[:, 2] - target_coord[:, 2]) ** 2), 95), 2)
    dev_height_init_target = np.round(np.percentile(np.sqrt((initial_coord[:, 2] - target_coord[:, 2]) ** 2), 95), 2)

    # draw_las(source_transformed, "source.las")

    #%% Plot the data as subplots.
    import matplotlib.pyplot as plt

    print(f'filename:{file_list[0]}')
    average_distance_target = np.mean(nearest_referanced)
    average_distance_before = np.mean(nearest_raw)
    print(f'average distanse wrong is {average_distance_target} m, and before the registration is it {average_distance_before} m')
    print(f'min deviation error is {np.min(nearest_referanced)} m and max:{np.max(nearest_referanced)}')
    print(f'RMSE value for initial coordinates: {rms_n_init, rms_e_init, rms_alt_init}')
    print(f'RMSE value for estimated coordinates after point cloud registration:\n {rms_n_target, rms_e_target, rms_alt_target}')
    print(f'RMSE value for initial coordinates against estimated coordinates: {rms_n_target_v_init, rms_e_target_v_init, rms_alt_target_v_init}')
    if bool(outliers) is True:
        print(f'The outliers after the point cloud registration is in file with frame and inlier RMSE value'
              f': {str(outliers)}')
    elif handle_outliers is False:
        print(f'The file is not corrected for outliers')
    else:
        print('There are no outliers after the point cloud registration')
    plt.style.use('fivethirtyeight')
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    fig.set_size_inches(30, 30, forward=True)
    line1 = f'Estimated deviation between the target trajectory against the true trajectory'
    std_dist = np.sqrt(std[:, 0]**2+std[:, 1]**2)
    line2 = f'Average distanse error is {np.round(average_distance_target,2)} m, and before the registration: {np.round(average_distance_before,2)} m'
    line3 = f'Based on the files {file_list}'
    line4 = f'From frame {from_frame} to frame {to_frame} with {skips -1} skips'
    line5 = f'min deviation error is {np.round(np.min(nearest_referanced),3)} m and max:{np.round(np.max(nearest_referanced),3)}m'
    line6 = f'Target trajectory = trajectory after prosessing trough point cloud registration'

    fig.suptitle(line1 + '\n' + line2 + "\n" + line3 + "\n" + line4 + '\n' + line5 + '\n' + line6)
    ax1.plot(target_coord[:, 0] - target_coord[0, 0], target_coord[:, 1] - target_coord[0, 1])  # , '-bo')
    ax1.plot(true_coord[:, 0] - target_coord[0, 0], true_coord[:, 1] - target_coord[0, 1])  # , '-ro')
    ax1.set_title("Target trajectory against true trajectory", loc='center', wrap=True)
    # ax1.grid()
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.legend(["Target trajectory", "True trajectory"])

    ax2.plot(initial_coord[:, 0] - target_coord[0, 0], initial_coord[:, 1] - target_coord[0, 1], '-ro')  # , color='green')
    ax2.plot(true_coord[:, 0] - target_coord[0, 0], true_coord[:, 1] - target_coord[0, 1], '-bo')  # , color='red')
    ax2.set_title("PPP trajectory against True trajectory", loc='center', wrap=True)
    # ax2.grid()
    ax2.set_xlabel("East (m)")
    ax2.set_ylabel("North (m)")
    ax2.legend(["PPP trajectory", "True trajectory"])

    dev_x = target_coord[:, 0] - initial_coord[:, 0]
    dev_y = target_coord[:, 1] - initial_coord[:, 1]
    dev_z = target_coord[:, 2] - initial_coord[:, 2]
    x_time = np.asarray(timesteps) - np.asarray(timesteps[0])
    ax3.plot(x_time, cross_track, label="Cross Track Error")
    ax3.plot(x_time, long_track, label="Long track error")
    # ax3.scatter(x_time, cross_track, color="blue", label="Cross Track Error")
    # ax3.scatter(x_time, long_track, color="green", label="long track error")
    ax3.set_xlabel("Frames")
    ax3.set_ylabel("Cross Track Error")
    ax3.set_ylim([-1.5, 1.5])
    ax3.axhline(y=0.0, color='black', linestyle='-')
    # ax3.set_xticks(1, int(len(target_coord[:, 0])))
    ax3.set_title("Cross track and long track error", loc='center', wrap=True)
    ax3.legend()

    # ax4.plot(x_time, nearest_raw, color="green")
    # ax4.plot(x_time, nearest_referanced)
    # ax4.plot(x_time, raw_coord[:, 0]-sbet_coord[:, 0])
    # ax4.plot(x_time, raw_coord[:, 1]-sbet_coord[:, 1])
    # ax4.plot(x_time, dev_x)
    # ax4.plot(x_time, dev_y)
    dev = np.sqrt(std[:, 0]**2+std[:, 1]**2)
    st = []

    ax4.plot(x_time, dev)
    ax4.plot(x_time, dev_z, color="red")
    # res = stats.linregress(timesteps, st)

    ax4.set_title("Deviation error in 2D from the true trajectory", loc='center', wrap=True)
    ax4.set_xlabel("Frames")
    ax4.set_ylabel("Deviation (m)")
    # ax4.set_ylim([-0.2, 1.5])
    ax4.axhline(y=0.0, color='black', linestyle='-')
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
        # true_filename = 'pros_data\\true_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        sbet_filename = 'pros\\true_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        raw_filename = 'pros\\initial_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        target_filename = 'pros\\target_trajectory_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        heading_filename = 'pros\\heading_' + time.strftime("%Y-%m-%d_%H%M") + '.npy'
        np.save(heading_filename, direction)
        # np.save(true_filename, true_trajectory)
        np.save(sbet_filename, true_coord)
        np.save(raw_filename, initial_coord)
        np.save(target_filename, target_coord)

        file_name = 'info_script_' + time.strftime("%Y-%m-%d-%H%M") + '_from_file ' + file_list[0]+'.txt'
        text_file = open('pros\\' + file_name, "w")
        if standalone is True:
            GNSS_system = "Standalone"
        elif sbet_process == "PPP" and standalone is False:
            GNSS_system = "PPP"
        else:
            GNSS_system = "ETPOS"
        text_file.write(f'Processing of the following trajectory:{section}, from {system_folder} with initial trajectory {GNSS_system}\n')
        text_file.write(f'And algorithm {algorithm}')

        text_file.write("\n")
        text_file.write(f'processed the file {file_list}, with the frames {from_frame} to {to_frame},with {skips} skips, and it took {total_time} seconds\n')
        text_file.write(f'Thats {np.round(total_time/60,2)} minutes or {np.round(total_time/(60*60),2)} hours\n\n\n')
        text_file.write(f'Total number of frames are {len(initial_coordinate[:,0])}, with average time of {total_time/len(initial_coordinate[:,0])} per frame\n\n\n')
        text_file.write(f'RMSE value for initial traj against true traj: {rms_n_init, rms_e_init, rms_alt_init} (n,e,alt)\n')
        text_file.write(f'RMSE value for target traj against true traj:{rms_n_target, rms_e_target, rms_alt_target} (n,e,alt)\n')
        text_file.write(f'RMSE value for initial traj against target traj: {rms_n_target_v_init, rms_e_target_v_init, rms_alt_target_v_init} (n,e,alt)\n\n\n')

        text_file.write(f'Area between trajectories for initial trajectory vs true trajectory {area_init_traj_source} m^2\n')
        text_file.write(f'Area between trajectories for target trajectory vs true trajectory {area_target_source} m^2\n')
        text_file.write(f'Area between trajectories for initial traj vs target trajectory {area_target_init_traj} m^2\n\n\n')

        text_file.write(f'95% percentile initial trajectory vs true trajectory {dev_plane_init_source} m in 2d and  {dev_height_init_source}m in height\n ')
        text_file.write(f'95% percentile target trajectory vs true trajectory {dev_plane_target_source} m in 2d and  {dev_height_target_source}m in height\n ')
        text_file.write(f'95% percentile initial trajectory vs target trajectory {dev_plane_init_target} m in 2d and  {dev_height_init_target}m in height\n ')

        if bool(outliers) is True:
            text_file.write(f'Total number of outliers are {num_outliers}, {np.round((num_outliers/(num_outliers+len(initial_coordinate[:,0])))*100,1)}% of the total number of frames\n\n')
        if removed_outliers is False:
            text_file.write('The results are not corrected for outliers\n\n')
        text_file.write(line1)
        text_file.write("\n")
        text_file.write(line2)
        text_file.write("\n")
        text_file.write(line5)
        text_file.write("\n")
        if bool(outliers) is True:
            text_file.write("The outliers of the file are frames where the inlier RMSE value is higher than 0.25m after \n "
                            "An initial attempt to converge the outliers")
            text_file.write("\n")
            text_file.write(str(outliers))

        else:
            text_file.write('There are no outliers after the point cloud registration\n\n')
        text_file.close()

    # Reshapes the arrays,to more readable and usable data.sbet_full = np.reshape(full_sbet, (-1, 3))
