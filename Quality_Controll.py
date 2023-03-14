import numpy as np


def c_l_track_error(p1, p2, heading):

    """Calculates the cross-track error (cte) and along-track error (lte) between two points.

    Args:
        p1 (numpy.ndarray): The coordinates of the starting point in the form [x, y, z].
        p2 (numpy.ndarray): The coordinates of the ending point in the form [x, y, z].
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

    target_dict = {'Y': target_points[:, 0],'X': target_points[:, 1], 'alt': target_points[:, 2]}
    source_dict = {'Y': source_points[:, 0], 'X': source_points[:, 1], 'alt': source_points[:, 2]}

    rms_n = np.sqrt((np.sum((target_dict['X']-source_dict['X'])**2))/len(source_dict['X']))
    rms_e = np.sqrt((np.sum((target_dict['Y']-source_dict['Y'])**2))/len(source_dict['X']))
    rms_alt = np.sqrt((np.sum((target_dict['alt']-source_dict['alt'])**2))/len(source_dict['X']))

    return rms_n, rms_e, rms_alt


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

    target = np.load("pros_data\\target_coord_2023-03-13_0644.npy")
    source = np.load("pros_data\\sbet_coord_2023-03-13_0644.npy")
    init_target = np.load("pros_data\\raw_coord_2023-03-13_0644.npy")
    rms_n_init, rms_e_init, rms_alt_init = np.round(root_mean_square(init_target,target),3)
    rms_n_target, rms_e_target, rms_alt_target = np.round(root_mean_square(target, source),3)
    print(f'Init target: {rms_n_init, rms_e_init, rms_alt_init}')
    print(f'target: {rms_n_target, rms_e_target, rms_alt_target}')
    std_N, std_E, std_alt = standard_deviation(init_target, source)
    std_N_tar, std_E_tar,std_alt_tar = standard_deviation(target, source)
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