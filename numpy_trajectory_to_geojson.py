def numpy_to_dataframe(numpy_array):
    import pandas as dataframe
    df = dataframe.DataFrame(numpy_array, columns=['North', 'East', 'alt'])
    return df


if __name__ == "__main__":

    import numpy as np
    import pandas as pd
    import geopandas as gpd
    from shapely.geometry import LineString,Point
    print('Input the numpyarray and path to be processed into geojson format:\n')
    numpy_array = input()
    filename = numpy_array.split("\\")
    filename = filename[3].split(".")
    filename = filename[0]
    trajectory = np.load(numpy_array)
    list = []
    df = pd.DataFrame(trajectory, columns=['North', 'East', 'alt'])
    points = []
    for traj in trajectory:

        list.append((float(traj[0]), float(traj[1])))

    # gdf = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.North, df.East), crs="EPSG:5972")
    line = LineString(list)
    df = {'filename': [filename], 'geometry': [line]}
    gdf = gpd.GeoDataFrame(df, geometry='geometry', crs="EPSG:5972")
    gdf.to_json()
    save_file = 'pros_data\\'+filename+'.geojson'
    gdf.to_file(save_file, driver="GeoJSON")

    """
    True_trajectory = np.load("pros_data\\true_trajectory_2023-02-28_0735.npy")
    list = []
    for traj in True_trajectory:
        list.append((float(traj[0]), float(traj[1])))


    df = pd.DataFrame(True_trajectory, columns=['North', 'East', 'alt'])

    line = LineString(list)

    # create a dataframe with needed attributes and required geometry column
    df = {'GWU': ['Dept Geography'], 'geometry': [line]}
    # Convert shapely object to a geodataframe
    gdf = gpd.GeoDataFrame(df, geometry='geometry', crs="EPSG:5972")

    #gdf = geopandas.GeoDataFrame(
    #    true_traj_df, geometry=geopandas.points_from_xy(true_traj_df.North, true_traj_df.East), crs="EPSG:5972")
    gdf.to_json()
    gdf.to_file("pros_data\\true_traj.geojson", driver="GeoJSON")

    SBET_trajectory = np.load('pros_data\\sbet_coord_2023-02-28_0735.npy')
    SBET_traj_df = numpy_to_dataframe(SBET_trajectory)



    PPP_trajectory = np.load('pros_data\\raw_coord_2023-02-28_0735.npy')
    PPP_traj_df = numpy_to_dataframe(PPP_trajectory)
    Georefferenced_Target = np.load('pros_data\\target_coord_2023-02-28_0735.npy')
    target_traj_df = numpy_to_dataframe(Georefferenced_Target)
    """



