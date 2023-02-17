import ouster.pcap as pcap
import ouster.client as client
from contextlib import closing
from more_itertools import nth
import laspy
import numpy as np
import copy
import open3d as o3d
def transform_pcap(pcap_raw, metadata, sbet_init=None, frame=None, geoid=0.0, fi=open("frame_7.txt", 'w')):

    import sys
    sys.path.insert(0, "C:/Users/isakf/Documents/1_Geomatikk/Master/master_code/teapot_lidar")
    from teapot_lidar.pcapReader import PcapReader
    pcap_reader = PcapReader(pcap_raw, metadata, sbet_path=sbet_init)  # Erlend Dahl Teapot

    pcap_reader.print_info(printFunc=lambda l: fi.write(l + "\n"), frame_index=frame)  # Erlend Dahl Teapot
    position = pcap_reader.get_coordinates()  # Erlend Dahl Teapot
    # Collects the North, East, alt, and heading at a given frame Teapot
    north = position[frame].x
    east = position[frame].y
    heading = position[frame].heading
    time_est = position[frame].sow
    alt = position[frame].alt - geoid
    center_coord_utm32 = np.array((north, east, alt))
    raw_pointcloud = get_frame(pcap_raw, metadata, frame)  # PCAP Software

    raw_pointcloud_correct_shape = raw_pointcloud.reshape((-1, 3))
    # Removes the mobile mapping vehicle
    point_cloud_prossesing = pcap_reader.remove_vehicle(raw_pointcloud_correct_shape)  # Erlend Dahl Teapot removes vehicle
    # Remove all data outside of a 40 meters radius.
    point_cloud_prossesing = pcap_reader.remove_outside_distance(40, point_cloud_prossesing)  # Erlend Dahl Teapot
    pc_o3d = point_cloud_pros(point_cloud_prossesing)  # Point cloud porsessed by OPEN3ds sorftware
    rotation_matrix = pc_o3d.get_rotation_matrix_from_axis_angle(np.array([0, 0, quadrant(heading)]))  # Open3d

    pc_o3d.rotate(rotation_matrix, center=pc_o3d.get_center())  # open3d
    pc_transformed_utm = copy.deepcopy(pc_o3d).translate(center_coord_utm32, relative=False)  # open3d
    return pc_transformed_utm, time_est, center_coord_utm32


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
    xyz = xyz.reshape((-1, 3))  # puts the point cloud xyz(numpy array) format
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


def filetype(filename, system="ETPOS"):
    # Establishes which round of data is used, This is just made to shorten the code later.
    if system == "ETPOS":
        raw_file = "OS-1-128_992035000186_1024x10_20211021_" + filename
        pathbase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP\\"
        sbet_raw = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
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
    # Inputs for the data
    voxel_size = 0.5  # means 5cm for this dataset
    file_list = get_files(16, 1)  # the files from the 10th file and 5 files on
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    geoid_height = 39.438
    from_frame = 0
    to_frame = 196
    skips = 195
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
    for files in file_list:  # For loop that goes through the PCAP files, and the corresponding laz files.
        # Source_init is the raw laz file corresponding to the PCAP file
        source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + files + ".laz"
        # Transformes the Laz file into Open 3d point cloud.
        pc_raw_laz = read_laz(source_init)
        pc_o3d_laz = point_cloud_pros(pc_raw_laz)
        source = pc_o3d_laz  # Point cloud in open3d python format.

        # initializes the Sbet file as ether the PPP og ETPOS file.
        sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"
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
        sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
        # sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84.out"
        sbet = SbetParser(sbet_ref)

        # This for loop go through all frames choosen, with the designated skips
        for k in range(from_frame, to_frame, skips):
            frame_index = k  # Collects frame
            with open("frame_7.txt", 'w') as f:  # Transforms PCAP files to Open 3d point clouds, in the correct heading and coordinatesystem
                pc_transformed, time_sbet, coord = transform_pcap(pcap_file, meta, sbet_file, frame_index, geoid_height)
            timesteps.append(time_sbet)  # collects all timesteps
            target = pc_transformed

            saved_center = source.get_center()  # Saves center for later transformation.

            # Initial transform that translate down to local coordinateframe
            downsampled_source, source_transformed, downsampled_target, target_transformed = initial_transform(source, target)
            threshold = 1

            trans_init = np.identity(4)  # initial transformation matrix
            # import ICP_Point
            # trans_init = ICP_Point.draw_icp(source_transformed, target_transformed, trans_init)

            # At this point, both the target and the source are in a local coordinateframe
            trans_init = o3d_icp(downsampled_source, downsampled_target, trans_init, iterations=9)  # Perform ICP on downsampled data
            trans_init = o3d_icp(source_transformed, target_transformed, trans_init, iterations=1)  # Perform ICP on the whole dataset.
            import ICP_Point
            trans_init = ICP_Point.draw_icp(source_transformed, target_transformed, trans_init)

            target_transformed.transform(trans_init)  # Final Transformation for the target point cloud
            trans_matrix.append(np.mean(trans_init))  # stores all transformation matrixes.
            # Below the Source and Target gets translated back to absolute coordinates. The reason for this is because of constraintes on Open 3d ICP algorithm
            source_ICP = copy.deepcopy(source_transformed).translate(saved_center, relative=False)
            target_ICP = copy.deepcopy(target_transformed).translate(target_transformed.get_center() + saved_center, relative=False)

            # Get sbet coord for a gived timestep
            referance_positon = sbet.get_position_sow(time_sbet)
            referance_coord = np.array([referance_positon.x, referance_positon.y, referance_positon.alt - geoid_height])
            deviation = target_ICP.get_center() - referance_coord
            # To make sure that the time step is correct, The for loop below gives a timespam for point to find closest point to the True trajectory
            for steps in np.arange(time_sbet-0.1, time_sbet+0.1, 0.1):
                temp_positon = sbet.get_position_sow(steps)
                temp_coord = np.array([temp_positon.x, temp_positon.y, temp_positon.alt - geoid_height])
                temp_std = target_ICP.get_center() - temp_coord

                if np.sqrt(deviation[0]**2+deviation[1]**2) > np.sqrt(temp_std[0]**2+temp_std[1]**2):
                    referance_coord = temp_coord
                    deviation = temp_std

            # Fill all numpy arrays with the correct variables for each iteration
            pre_activation = target.get_center() - referance_coord
            std.append(deviation)
            std_raw.append(pre_activation)
            target_coord.append(target_ICP.get_center())
            sbet_coord.append(referance_coord)
            raw_coord.append(target.get_center())

    # Reshapes the arrays,to more readable and usable data.
    sbet_full = np.reshape(full_sbet, (-1, 3))
    std = np.reshape(std, (-1, 3))
    std_raw = np.reshape(std_raw, (-1, 3))
    sbet_coord = np.reshape(sbet_coord, (-1, 3))
    target_coord = np.reshape(target_coord, (-1, 3))
    raw_coord = np.reshape(raw_coord, (-1, 3))

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
    sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    #sbet_ref = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"

    sbet = SbetParser(sbet_ref)
    # Produce the true trajectory with a set Hz
    true_trajectory = []
    Hz = 50
    for k in np.arange(min_time, max_time, 1/Hz):
        temp_positon = sbet.get_position_sow(k)
        temp_coord = np.array([temp_positon.x, temp_positon.y, temp_positon.alt - geoid_height])
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
    line2 = f'Average distanse error is {np.round(average_distance_target,2)} m, and with the same timestamp: {np.round(np.mean(std),2)} m'
    line3 = f'Based on the files {file_list}'
    line4 = f'From frame {from_frame} to frame {to_frame} with {skips -1} skips'
    line5 = f'min deviation error is {np.round(np.min(nearest_referanced),3)} m and max:{np.round(np.max(nearest_referanced),3)}m'
    line6 = f'Target trajectory = trajectory after prosessing trough point cloud registration'
    fig.suptitle(line1 + '\n' + line2 + "\n" + line3 + "\n" + line4 + '\n' + line5 + '\n' + line6)
    ax1.plot(target_coord[:, 0], target_coord[:, 1], color="green")
    ax1.plot(true_trajectory[:, 0], true_trajectory[:, 1], color="red")
    ax1.set_title("Target trajectory against true trajectory", loc='center', wrap=True)
    ax1.grid()
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.legend(["Target trajectory", "True trajectory"])

    ax2.plot(raw_coord[:, 0], raw_coord[:, 1], color="green")
    ax2.plot(true_trajectory[:, 0], true_trajectory[:, 1], color="red")
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
    ax3.plot(dev_x, color="blue")
    ax3.plot(dev_y, color="orange")
    ax3.plot(np.sqrt(dev_x**2+dev_y**2), color = "cyan")
    ax3.set_xlabel("Frames")
    ax3.set_ylabel("Deviation (M)")
    ax3.axhline(y=0.0, color='r', linestyle='-')
    # ax3.set_xticks(1, int(len(target_coord[:, 0])))
    ax3.set_title("Deviation error between Target trajectory and PPP trajectory", loc='center', wrap=True)
    ax3.legend(["Deviation in North", "Deviation in East", "Deviation in 2D"])

    ax4.plot(nearest_raw, color="red")
    ax4.plot(nearest_referanced, color="green")
    from scipy import stats
    dev = np.sqrt(std[:,0]**2+std[:,1]**2)
    st = []
    for out in dev:
        if out > 3:
            st.append(np.median(dev))
        else:
            st.append(out)
    ax4.plot(st)
    res = stats.linregress(timesteps, st)

    ax4.set_title("Deviation error in 2D from the true trajectory", loc='center', wrap=True)
    ax4.set_xlabel("Frames")
    ax4.set_ylabel("Deviation (m)")
    ax4.set_ylim([-0.04,3])
    # ax4.set_xticks(1, len(nearest_raw))
    ax4.legend(["PPP trajectory", "Nearest trajectory","Nearest trajectory based on time"])
    fig.show()
    fig.savefig('plots\\Estimatation' + time.strftime("%Y-%m-%d %H%M%S") + '.png')
