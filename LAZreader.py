import open3d as o3d
import numpy as np
import time
import laspy
import ICP_Point as ICP
import copy
import ouster.pcap as pcap
import ouster.client as client
from contextlib import closing
from more_itertools import nth
import open3d as o3d
import numpy as np
import copy
import time
import matplotlib.pyplot as plt
def draw_registration_result(source, target, transformation):

    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])

    source_temp.transform(transformation)

    # create point cloud and coordinate axes geometries
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(1.0)

    # initialize visualizer and rendering options
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for g in [source_temp, target_temp]:
        vis.add_geometry(g)

    vis.add_geometry(axes)
    ropt = vis.get_render_option()
    ropt.point_size = 1.0
    ropt.background_color = np.asarray([0, 0, 0])

    # initialize camera settings
    ctr = vis.get_view_control()
    ctr.set_zoom(0.1)
    ctr.set_lookat(source.get_center())
    ctr.set_up([0.85, 0.12, 0.52])

    # run visualizer main loop
    print("Press Q or Excape to exit")
    vis.run()
    vis.destroy_window()

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


def prepare_dataset_PCAP(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    Referance = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"
    las = laspy.read(Referance)
    xyz_CPOS = las.xyz
    las = laspy.read(PPP)
    xyz_PPP = las.xyz
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_CPOS))
    #source = source.points

    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_PPP))

    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    Referance = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"
    PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    las = laspy.read(Referance)
    xyz_CPOS = las.xyz
    las = laspy.read(PPP)
    xyz_PPP = las.xyz
    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_CPOS))
    #source = source.points

    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_PPP))

    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    #draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
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

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size,result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


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


if __name__ == "__main__":

    voxel_size = 0.5  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(
        voxel_size)

    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    print(result_fast)
    draw_registration_result(source_down, target_down, result_fast.transformation)

    # Create a 3D matplotlib plot showing the data
    [x, y, z] = [c.flatten() for c in np.dsplit(source_down.points, 3)]
    ax = plt.axes(projection='3d')
    r = 20
    ax.set_xlim3d([-r, r])
    ax.set_ylim3d([-r, r])
    ax.set_zlim3d([-r / 2, r / 2])
    plt.axis('off')
    z_col = np.minimum(np.absolute(z), 5)
    ax.scatter(x, y, z, c=z_col, s=0.2)

    plt.show()

    """
    result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                     voxel_size,result_fast)
    print(result_icp)
    draw_registration_result(source, target, result_icp.transformation)

"""

    """
    Referance = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    accumulatedTime = 0.0
    startTime = time.perf_counter()
    las = laspy.read(Referance)
    xyz_CPOS = las.xyz
    source = xyz_CPOS

    PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"
    las = laspy.read(PPP)
    xyz_PPP = las.xyz
    target = xyz_PPP
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(xyz_CPOS)


    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(xyz_PPP)
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    accumulatedTime += time.perf_counter() - startTime
    print(f"Time usage: {time.perf_counter() - startTime:0.4f} seconds.")
    print("")
    downsampled_source = source.voxel_down_sample(voxel_size=0.5)
    downsampled_target = target.voxel_down_sample(voxel_size=0.5)
    accumulatedTime += time.perf_counter() - startTime
    print(f"Downsampling (0.5) performed in {(time.perf_counter() - startTime) / 2.0:0.4f} seconds per cloud.")

    source_feature = source.points.registration.Feature
    target_feature = target.points.registration.Feature

    test = o3d.registration.registration_fast_based_on_feature_matching(source, target, source_feature, target_feature)

    
    threshold = 1
    trans_init = np.identity(4)
    ICP.draw_registration_result(downsampled_source, downsampled_target, trans_init)

    trans_init = ICP.draw_icp(downsampled_source, downsampled_target, trans_init)

    # See PyCharm help at https://www.jetbrains.com/help/pycharm/
    # Create a PPTK viewer to show the data
    # poc = pptk.viewer(xyz_CPOS)
    # poc = pptk.viewer(xyz_PPP)
    # poc.set(point_size=0.005)


    # Segmentation
    #downpcd = pc.voxel_down_sample(voxel_size=0.5)
    plane_model, inliers = pc.segment_plane(distance_threshold=1, ransac_n=10, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = pc.select_by_index(inliers)
    outlier_cloud = pc.select_by_index(inliers, invert=True)

    o3d.visualization.draw_geometries([inlier_cloud])
"""
