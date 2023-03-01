import open3d as o3d
import numpy as np
import time
import laspy
import open3d as o3d
import numpy as np
import copy
import time
import matplotlib.pyplot as plt
import draw_registration as ICP

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

def prepare_dataset(source_init,target_init,voxel_size):
    """Function to prepare laz files, and convert them to Point cloud"""
    print(":: Load two point clouds and disturb initial pose.")
    #PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    las_sour = laspy.read(source_init)
    xyz_source = las_sour.xyz
    laz_tar = laspy.read(target_init)
    xyz_target= laz_tar.xyz

    source = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_source))
    target = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz_target))

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
    source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_194620.laz"
    target_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\1024x10_20211021_200226.laz"
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source_init, target_init, voxel_size)
    """
    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    print(result_fast)
    ICP.draw_registration_result(source_down, target_down, result_fast.transformation)

    # Create a 3D matplotlib plot showing the data
    """
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
