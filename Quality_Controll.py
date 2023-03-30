import numpy as np


def c_l_track_error(p1, p2, heading):

    """Calculates the cross-track error (cte) and along-track error (lte) between two points.

    Args:
        p1 (numpy.array): The coordinates of the starting point in the form [x, y, z].
        p2 (numpy.array): The coordinates of the ending point in the form [x, y, z].
        heading (float): The heading angle in radians.

    Returns:
        tuple: A tuple containing the cross-track error (cte) and along-track error (lte) in meters.
    """

    # Calculate the vector between the two points
    v = p2 - p1

    # Normalize the vector
    v_norm = np.linalg.norm(v)

    # From grunnleggende landmåling
    tg = (p2[0]-p1[0])/(p2[1]-p1[1])

    if p2[0]-p1[0] < 0 < p2[1]-p1[1]:
        tg = -tg + np.pi
    elif p2[0]-p1[0] < 0 and p2[1]-p1[1] < 0:
        tg = -tg + np.pi
    elif p2[1]-p1[1] < 0 < p2[0]-p1[0]:
        tg = -tg + 2*np.pi

    # Calculate the perpendicular vector from the starting point to the line connecting the two points
    # w = np.cross(v_norm, np.array([0, 0, 1]))
    # print(f'test{w} stop TEEEEST')
    # Calculate the cross-track error
    cte = np.sin(tg-np.pi+heading) * v_norm
    lte = np.cos(tg-np.pi+heading) * v_norm

    return cte, lte


def root_mean_square(target_points,source_points):
    """
    Calculates the root-mean-square (RMS) errors in the North, East, and altitude (up) directions
    between two sets of target and source points.

    Args:
        target_points (numpy.array): Array of shape (3, n) containing target points in the format [X, Y, altitude].
        source_points (numpy.array): Array of shape (3, n) containing source points in the format [X, Y, altitude].

    Returns:
        Tuple of three floats representing the RMS errors in the North, East, and altitude (up) directions, respectively.
    """

    target_dict = {'East': target_points[:, 0], 'North': target_points[:, 1], 'alt': target_points[:, 2]}
    source_dict = {'East': source_points[:, 0], 'North': source_points[:, 1], 'alt': source_points[:, 2]}

    rms_n = np.sqrt((np.sum((target_dict['North']-source_dict['North'])**2))/len(source_dict['North']))
    rms_e = np.sqrt((np.sum((target_dict['East']-source_dict['East'])**2))/len(source_dict['East']))
    rms_alt = np.sqrt((np.sum((target_dict['alt']-source_dict['alt'])**2))/len(source_dict['alt']))

    return rms_e, rms_n, rms_alt


def standard_deviation(coord_1, coord_2):
    """
    Calculates the standard deviation of N, E, and altitude values between two sets of 3D coordinates.

    Args:
    - coord_1 (numpy.ndarray): A 2D array of shape (N,3) containing the N, E, and altitude values for the first set of coordinates.
    - coord_2 (numpy.ndarray): A 2D array of shape (N,3) containing the N, E, and altitude values for the second set of coordinates.

    Returns:
    - tuple: A tuple containing the standard deviation of N, E, and altitude values between the two sets of coordinates.
    """
    # Extracting the N, E, and altitude values from the two sets of coordinates
    x1, y1, z1 = coord_1[:,0], coord_1[:,1], coord_1[:,2]
    x2, y2, z2 = coord_2[:,0], coord_2[:,1], coord_2[:,2]

    # Calculating the standard deviation of the N, E, and altitude values
    std_N = np.sqrt(np.sum((x1-x2)**2)/(len(x1)-1))
    std_E = np.sqrt(np.sum((y1-y2)**2)/(len(y1)-1))
    std_alt = np.sqrt(np.sum((z1 - z2) ** 2) / (len(z1) - 1))

    # Returning the tuple containing the standard deviation of N, E, and altitude values
    return std_N, std_E, std_alt


if __name__ == "__main__":

    import numpy as np

    target = np.load("pros_data\\Standalone\\Round1_fulltraj_corrected_outliers\\target_coord_2023-03-20_0641.npy")
    source = np.load("pros_data\\Standalone\\Round1_fulltraj_corrected_outliers\\sbet_coord_2023-03-20_0641.npy")
    init_target = np.load("pros_data\\Standalone\\Round1_fulltraj_corrected_outliers\\raw_coord_2023-03-20_0641.npy")
    rms_e_init, rms_n_init, rms_alt_init = np.round(root_mean_square(init_target,target),2)
    rms_e_target, rms_n_target, rms_alt_target = np.round(root_mean_square(target, source),2)
    print(f'Init target: {rms_n_init, rms_e_init, rms_alt_init}')
    print(f'target: {rms_n_target, rms_e_target, rms_alt_target}')
    std_N, std_E, std_alt = standard_deviation(init_target, source)
    std_N_tar, std_E_tar,std_alt_tar = standard_deviation(init_target, target)
    import numpy as np
    import similaritymeasures
    import matplotlib.pyplot as plt

    exp_data = init_target[:,0:2]
    num_data = source[:,0:2]
    xi, eta, xiP, etaP = similaritymeasures.normalizeTwoCurves(exp_data[:,0], exp_data[:,1], num_data[:,0], num_data[:,1])
    # exp_data = np.array((xi,eta))
    # num_data = np.array((xiP,etaP))

    # quantify the difference between the two curves using PCM
    pcm = similaritymeasures.pcm(exp_data, num_data)
    # quantify the difference between the two curves using
    # Discrete Frechet distance
    df = similaritymeasures.frechet_dist(exp_data, num_data)
    # quantify the difference between the two curves using
    # area between two curves
    area = similaritymeasures.area_between_two_curves(exp_data, num_data)
    # quantify the difference between the two curves using
    # Curve Length based similarity measure
    cl = similaritymeasures.curve_length_measure(exp_data, num_data)
    # quantify the difference between the two curves using
    # Dynamic Time Warping distance
    dtw, d = similaritymeasures.dtw(exp_data, num_data)
    # mean absolute error
    mae = similaritymeasures.mae(exp_data, num_data)
    # mean squared error
    mse = similaritymeasures.mse(exp_data, num_data)

    # print the results
    print(pcm, df, area, cl, dtw, mae, mse)

    # plot the data
    plt.figure()
    plt.plot(exp_data[:, 0], exp_data[:, 1])
    plt.plot(num_data[:, 0], num_data[:, 1])
    plt.show()

    ## And much more!

    """
    from sklearn.metrics import r2_score
    from scipy.spatial.distance import cdist
    Y = cdist(source[:,0:2], target[:,0:2], 'euclidean')

    x_data = r2_score(source[:,0]-target[0,0], init_target[:,0]-target[0,0],multioutput='variance_weighted')
    y_data = r2_score(source[:,1]-target[0,1], init_target[:,1]-target[0,1],multioutput='variance_weighted')
    print(np.round(x_data, 8), np.round(y_data, 8))
    from frechetdist import frdist
    test = frdist(source[:,0:2]-target[0,0:2],target[:,0:2]-target[0,0:2])

    from sklearn.metrics import r2_score
    x_data = r2_score(source[:,0]-target[0,0], target[:,0]-target[0,0],multioutput='variance_weighted')
    y_data = r2_score(source[:,1]-target[0,1], target[:,1]-target[0,1],multioutput='variance_weighted')
    """
    # print(np.round(x_data, 8), np.round(y_data, 8))
    print(f'standard deviation between the coordinates and the\ncalculated coordinates: {std_N, std_E,std_alt}')
    print(f'standard deviation between the initial coordinates and the\ncalculated coordinates: {std_N_tar, std_E_tar,std_alt_tar}')
    """
    target = np.load("pros_data\\target_coord_2023-03-08_0655.npy")
    source = np.load("pros_data\\sbet_coord_2023-03-08_0655.npy")
    init_target = np.load("pros_data\\raw_coord_2023-03-08_0655.npy")
    rms_n_init, rms_e_init, rms_alt_init = root_mean_square(init_target,source)
    rms_n_target, rms_e_target, rms_alt_target = root_mean_square(target, source)
    print(f'Init target: {rms_n_init, rms_e_init, rms_alt_init}')
    print(f'target: {rms_n_target, rms_e_target, rms_alt_target}')
    std_N, std_E, std_alt = standard_deviation(init_target, source)
    std_N_tar, std_E_tar,std_alt_tar = standard_deviation(target, source)
    print(f'standard deviation between the initial coordinates and the\ncalculated coordinates: {std_N, std_E,std_alt}')
    print(f'standard deviation between the initial coordinates and the\ncalculated coordinates: {std_N_tar, std_E_tar,std_alt_tar}')
    """