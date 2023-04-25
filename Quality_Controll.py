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
    # Calculate the cross-track error
    cte = np.sin(tg-np.pi+heading) * v_norm
    lte = np.cos(tg-np.pi+heading) * v_norm

    return cte, lte


def root_mean_square(target_points, source_points):
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
    x1, y1, z1 = coord_1[:, 0], coord_1[:, 1], coord_1[:, 2]
    x2, y2, z2 = coord_2[:, 0], coord_2[:, 1], coord_2[:, 2]

    # Calculating the standard deviation of the N, E, and altitude values
    std_n = np.sqrt(np.sum((x1-x2)**2)/(len(x1)-1))
    std_e = np.sqrt(np.sum((y1-y2)**2)/(len(y1)-1))
    stdalt = np.sqrt(np.sum((z1 - z2) ** 2) / (len(z1) - 1))

    # Returning the tuple containing the standard deviation of N, E, and altitude values
    return std_n, std_e, stdalt


def area_between_trajectories(target_points, source_points):
    """
    Calculates the area between two trajectories.

    This function uses the similaritymeasures package to calculate the area
    between two trajectories represented by sets of points in 2D space.

    Args:
        target_points (ndarray): An array of shape (n, 3) representing the
            points of the target trajectory.
        source_points (ndarray): An array of shape (m, 3) representing the
            points of the source trajectory.

    Returns:
        float: The area between the two trajectories.

    Example:
        target_points = np.array([[0, 0], [1, 1], [2, 2]])
        source_points = np.array([[0, 0], [1, 2], [2, 1]])
        area = area_between_trajectories(target_points, source_points)
        print(area)  # Output: 0.5
    """
    import similaritymeasures
    exp_data = target_points[:, 0:2]
    num_data = source_points[:, 0:2]
    area = similaritymeasures.area_between_two_curves(exp_data, num_data)
    return area


def percentile(traj1, traj2, percent=95):
    dev_plane_1_2 = np.round(np.percentile(np.sqrt((traj1[:, 0]-traj2[:, 0])**2+(traj1[:, 2]-traj2[:, 2])**2), percent), 3)
    dev_height_1_2 = np.round(np.percentile(np.sqrt((traj1[:, 2]-traj2[:, 2])**2), percent), 3)

    return dev_plane_1_2, dev_height_1_2


def dev(traj1, traj2):

    deviation = np.sqrt((traj1[:, 0] - traj2[:, 0]) ** 2 + (traj1[:, 2] - traj2[:, 2]) ** 2)
    return deviation

if __name__ == "__main__":

    import numpy as np
    target = np.load("results\\Standalone\\Round1_fulltraj_corrected_outliers\\target_trajectory_2023-04-20_0902.npy")
    source = np.load("results\\Standalone\\Round1_fulltraj_corrected_outliers\\true_trajectory_2023-04-20_0902.npy")

    target_r1_co = np.load("results\\Standalone\\Round1_fulltraj_corrected_outliers\\target_trajectory_2023-04-20_0902.npy")
    source_r1_co = np.load("results\\Standalone\\Round1_fulltraj_corrected_outliers\\true_trajectory_2023-04-20_0902.npy")

    init_traj = np.load("results\\Standalone\\Round1_fulltraj_corrected_outliers\\initial_trajectory_2023-04-20_0902.npy")

    target_r1_un = np.load("results\\Standalone\\Round1_fulltraj_uncorrected_outliers\\target_trajectory_2023-04-20_0854.npy")
    source_r1_un = np.load("results\\Standalone\\Round1_fulltraj_uncorrected_outliers\\true_trajectory_2023-04-20_0854.npy")
    init_traj_r1_un = np.load("results\\Standalone\\Round1_fulltraj_uncorrected_outliers\\initial_trajectory_2023-04-20_0854.npy")

    target_r2_un = np.load(
        "results\\Standalone\\Round2_fulltraj_uncorrected_outliers\\target_trajectory_2023-04-21_0743.npy")
    source_r2_un = np.load(
        "results\\Standalone\\Round2_fulltraj_uncorrected_outliers\\true_trajectory_2023-04-21_0743.npy")

    target_r2_co = np.load(
        "results\\Standalone\\Round2_fulltraj_corrected_outliers\\target_trajectory_2023-04-22_1021.npy")
    source_r2_co = np.load(
        "results\\Standalone\\Round2_fulltraj_corrected_outliers\\true_trajectory_2023-04-22_1021.npy")

    target_r1_pp = np.load(
        "results\\Standalone\\Round1_fulltraj_uncorrected_Point_to_Point\\target_trajectory_2023-04-24_0656.npy")
    source_r1_pp = np.load(
        "results\\Standalone\\Round1_fulltraj_uncorrected_Point_to_Point\\true_trajectory_2023-04-24_0656.npy")
    target_r1_PPP_un = np.load(
        "results\\PPP\\Round1_uncorrected_outliers\\target_trajectory_2023-04-18_0654.npy")
    source_r1_PPP_un = np.load(
        "results\\PPP\\Round1_uncorrected_outliers\\true_trajectory_2023-04-18_0654.npy")

    target_r1_PPP = np.load(
        "results\\PPP\\Round1_corrected_outliers\\target_trajectory_2023-04-25_0818.npy")
    source_r1_PPP = np.load(
        "results\\PPP\\Round1_corrected_outliers\\true_trajectory_2023-04-25_0818.npy")
    target_r2_PPP = np.load(
        "results\\PPP\\Round2_corrected_outliers\\target_trajectory_2023-04-17_0536.npy")
    source_r2_PPP = np.load(
        "results\\PPP\\Round2_corrected_outliers\\true_trajectory_2023-04-17_0536.npy")
    target_r2_PPP_un = np.load(
        "results\\PPP\\Round2_uncorrected_outliers\\target_trajectory_2023-04-16_1230.npy")
    source_r2_PPP_un = np.load(
        "results\\PPP\\Round2_uncorrected_outliers\\true_trajectory_2023-04-16_1230.npy")

    rms_n_init, rms_e_init, rms_alt_init = np.round(root_mean_square(init_traj, target), 2)
    rms_n_target, rms_e_target, rms_alt_target = np.round(root_mean_square(target, source), 2)
    rms_n_init_source, rms_e_init_source, rms_alt_init_source = np.round(root_mean_square(init_traj, source), 2)
    dev_plane_target_source = percentile(target, source)
    dev_plane_target_source_r1_un = percentile(target_r1_un, source_r1_un)

    import matplotlib.pyplot as plt
    import scipy
    tree = scipy.spatial.cKDTree(source_r2_un[:, 0:2])
    nearest_raw, ii = tree.query(source_r2_un[:, 0:2])
    nearest_referanced, jj = tree.query(target_r2_un[:, 0:2])

    a = dev(target_r1_co, source_r1_co)
    b = dev(target_r1_un, source_r1_un)
    c = dev(target_r2_un, source_r2_un)
    d = dev(target_r2_co, source_r2_co)
    e = dev(target_r1_pp, source_r1_pp)
    f = dev(target_r1_PPP, source_r1_PPP)
    g = dev(target_r2_PPP, source_r2_PPP)
    h = dev(target_r1_PPP_un, source_r1_PPP_un)
    i = dev(target_r2_PPP_un, source_r2_PPP_un)

    p = np.linspace(50, 100, 20)
    plt.style.use('fivethirtyeight')
    fig, ax = plt.subplots(1, 1)
    fig.set_size_inches(15, 25, forward=True)
    fig1 = np.percentile(a, p, method='linear')
    fig2 = np.percentile(f, p, method='linear')
    # for method, style, color in lines:
    ax.plot(
             p, fig1,
             label="Round1 corrected for outliers", linestyle=':')
    ax.plot(
         p, np.percentile(b, p, method='linear'),
         label="Round1 uncorrected for outliers", linestyle=':')
    ax.plot(
        p, np.percentile(e, p, method='linear'),
        label="Round1 uncorrected for outliers point to point", linestyle=':')
    ax.plot(
        p, fig2,
        label="Round1 corrected for outliers PPP initial trajectory", linestyle='--')
    ax.plot(
        p, np.percentile(h, p, method='linear'),
        label="Round1 uncorrected for outliers PPP initial trajectory", linestyle='--')

    ax.plot(
        p, np.percentile(c, p, method='linear'),
        label="Round2 uncorrected for outliers", linestyle='-.')
    ax.plot(
        p, np.percentile(d, p, method='linear'),
        label="Round2 corrected for outliers", linestyle='-.')

    ax.plot(
        p, np.percentile(g, p, method='linear'),
        label="Round2 corrected for outliers PPP initial trajectory", linestyle='-')
    ax.plot(
        p, np.percentile(i, p, method='linear'),
        label="Round2 uncorrected for outliers PPP initial trajectory", linestyle='-')
    ax.axvline(x=95, color='red', linestyle='--', lw = 2)

    ax.set_yscale('log', base=2)
    import matplotlib as mat
    ax.get_yaxis().set_major_formatter(mat.ticker.ScalarFormatter())

    # ax.get_xaxis().set_major_formatter(mat.ticker.ScalarFormatter())
    # ax.set_yticks([0, 0.2, 0.4, 0.8, 1, 3, 10, 30])
    # ax.set_ylim([0, 0.5])
    ax.set(
        title='Percentiles for Full trajectories',
        xlabel='Percentile',
        ylabel='Estimated planimetric deviation (m)'
    ) #, yticks=np.round(np.arange(0, np.max(b),1/np.max(b)),2))
    ax.legend()
    plt.show()
    fig.savefig('full_traj_percentile.png')

    target_r1_forest = np.load("results\\Standalone\\Round1_forest\\target_trajectory_2023-04-20_1941.npy")
    source_r1_forest = np.load("results\\Standalone\\Round1_forest\\true_trajectory_2023-04-20_1941.npy")

    target_r2_forest = np.load("results\\Standalone\\Round2_forest\\target_trajectory_2023-04-20_1937.npy")
    source_r2_forest = np.load("results\\Standalone\\Round2_forest\\true_trajectory_2023-04-20_1937.npy")

    target_r1_dense_2 = np.load("results\\Standalone\\Round1_urban\\target_trajectory_2023-04-21_2343.npy")
    source_r1_dense_2 = np.load("results\\Standalone\\Round1_urban\\true_trajectory_2023-04-21_2343.npy")

    target_r2_dense_2 = np.load("results\\Standalone\\Round2_urban\\target_trajectory_2023-04-21_2342.npy")
    source_r2_dense_2 = np.load("results\\Standalone\\Round2_urban\\true_trajectory_2023-04-21_2342.npy")

    target_r1_rural = np.load("results\\Standalone\\Round1_rural\\target_trajectory_2023-04-22_2226.npy")
    source_r1_rural = np.load("results\\Standalone\\Round1_rural\\true_trajectory_2023-04-22_2226.npy")

    target_r2_rural = np.load("results\\Standalone\\Round2_rural\\target_trajectory_2023-04-22_2229.npy")
    source_r2_rural = np.load("results\\Standalone\\Round2_rural\\true_trajectory_2023-04-22_2229.npy")

    a = dev(target_r1_forest, source_r1_forest)
    b = dev(target_r2_forest, source_r2_forest)
    forest = (a+b)/2
    c = dev(target_r1_dense_2[0:1171], source_r1_dense_2[0:1171])
    d = dev(target_r2_dense_2, source_r2_dense_2)
    dense = (c+d)/2
    e = dev(target_r1_rural, source_r1_rural)
    f = dev(target_r2_rural, source_r2_rural)
    rural = (e+f)/2

    p = np.linspace(50, 100, 50)
    plt.style.use('fivethirtyeight')
    plt.rcParams.update({'font.size': 18})
    fig, ax = plt.subplots(1, 1)
    fig.set_size_inches(25, 10, forward=True)
    import scipy.interpolate as interp

    # for method, style, color in lines:
    # define the range of x values to interpolate over
    """
    x_range = np.linspace(50, 100, 5000)

    # perform spline interpolation on each dataset
    forest_interp = interp.interp1d(p, np.percentile(forest, p, method='linear'), kind='cubic')
    dense_interp = interp.interp1d(p, np.percentile(dense, p, method='linear'), kind='cubic')
    rural_interp = interp.interp1d(p, np.percentile(rural, p, method='linear'), kind='cubic')

    # plot the interpolated data
    ax.plot(forest_interp(x_range), x_range, label="Forested area", linestyle='-.', linewidth=4)
    ax.plot(dense_interp(x_range), x_range, label="Urban street", linestyle='-', linewidth=4)
    ax.plot(rural_interp(x_range), x_range, label="Rural area", linestyle='--', linewidth=4)
    """
    ax.plot(
            np.percentile(forest, p, method='linear'), p,
            label="Forested area", linestyle='-.', linewidth=4)
    # ax.plot(
    #   p, np.percentile(b, p, method='linear'),
    #  label="Round2 Forest", linestyle='-.')
    ax.plot(
        np.percentile(dense, p, method='linear'), p,
        label="Urban street", linestyle='-', linewidth=4)
    # ax.plot(
    #    p, np.percentile(d, p, method='linear'),
    #    label="Round2 Dense Street", linestyle='-')

    ax.plot(
        np.percentile(rural, p, method='linear'),p,
        label="Rural area", linestyle='--', linewidth=4)

    # ax.plot(
    #    p, np.percentile(f, p, method='linear'),
    #    label="Round2 rural street", linestyle='--')

    # ax.set_yscale('log', base=2)
    import matplotlib as mat
    # ax.get_yaxis().set_major_formatter(mat.ticker.ScalarFormatter())
    # ax.set_ylim([3.5, 4])
    # ax.get_xaxis().set_major_formatter(mat.ticker.ScalarFormatter())
    # ax.set_yticks([0, 0.2, 0.4, 0.8, 1, 3, 10, 30])
    ax.set(
        title='Percentiles for Sections of the trajectory',
        ylabel='Percentile',
        xlabel='Estimated planimetric deviation (m)'
    ) #, yticks=np.round(np.arange(0, np.max(b),1/np.max(b)),2))
    ax.legend()
    plt.show()
    fig.savefig('Sections_traj_percentile.png')



    """
    print('This is an outlier')
    frame_outlier = [frame_index, f'Inlier RMSE: {np.round(init_inlier_rmse, 3)}']
    frame_outlier_list.append(frame_outlier)
    outlier = True
    num_outliers += 1
    """
    """
    print(
        f'95% percentile initial trajectory vs true trajectory {dev_plane_init_source} m in 2d and  {dev_height_init_source}m in height')
    print(
        f'95% percentile target trajectory vs true trajectory {dev_plane_target_source} m in 2d and  {dev_height_target_source}m in height')
    print(
        f'95% percentile initial trajectory vs target trajectory {dev_plane_init_target} m in 2d and  {dev_height_init_target}m in height')
    """
    print(f'Init vs source: {rms_n_init_source, rms_e_init_source, rms_alt_init_source}')
    print(f'target vs source: {rms_n_target, rms_e_target, rms_alt_target}')
    print(f'init vs target: {rms_n_init, rms_e_init, rms_alt_init}')

    std_N, std_E, std_alt = standard_deviation(init_traj, source)
    std_N_tar, std_E_tar, std_alt_tar = standard_deviation(init_traj, target)
    import numpy as np
    import similaritymeasures
    area_target_init_traj = np.round(area_between_trajectories(init_traj, target), 2)
    area_target_source = np.round(area_between_trajectories(target, source), 2)
    area_init_traj_source = np.round(area_between_trajectories(init_traj, source), 2)
    print(f'Area between trajectories for initial trajectory vs true trajectory {area_init_traj_source} m^2')
    print(f'Area between trajectories for target trajectory vs true trajectory {area_target_source} m^2')
    print(f'Area between trajectories for initial traj vs target trajectory {area_target_init_traj} m^2')
    from scipy import stats
    rng = np.random.default_rng()
    stats.ttest_ind(target[:,0], init_traj[:,0],equal_var=False)

    """
    import matplotlib.pyplot as plt

    exp_data = target[:,0:2]
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
    print(area, mse)

    # plot the data
    plt.figure()
    plt.plot(exp_data[:, 0], exp_data[:, 1])
    plt.plot(num_data[:, 0], num_data[:, 1])
    plt.show()

    ## And much more!

    """
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
    
    # print(np.round(x_data, 8), np.round(y_data, 8))
    print(f'standard deviation between the coordinates and the\ncalculated coordinates: {std_N, std_E,std_alt}')
    print(f'standard deviation between the initial coordinates and the\ncalculated coordinates: {std_N_tar, std_E_tar,std_alt_tar}')

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
