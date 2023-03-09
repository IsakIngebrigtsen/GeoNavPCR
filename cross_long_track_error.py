
import numpy as np


def c_l_track_error(p1, p2, heading):
    # Create vectors for the two points
    # p1 and p2 are vectors in x,y,z

    # Calculate the vector between the two points
    v = p2 - p1

    # Normalize the vector
    v_norm = np.linalg.norm(v)
    # From grunnleggende landmåling

    tg = (p2[0]-p1[0])/(p2[1]-p1[1])
    """
    if p2[0]-p1[0] < 0 < p2[1]-p1[1]:
        tg = -tg + np.pi
    elif p2[0]-p1[0] < 0 and p2[1]-p1[1] < 0:
        tg = -tg + np.pi
    elif p2[1]-p1[1] < 0 < p2[0]-p1[0]:
        tg = -tg + 2*np.pi
    """
    # Calculate the perpendicular vector from the starting point to the line connecting the two points
    # w = np.cross(v_norm, np.array([0, 0, 1]))
    # print(f'test{w} stop TEEEEST')
    # Calculate the cross-track error
    cte = np.sin(tg-np.pi+heading) * v_norm
    lte = np.cos(tg-np.pi+heading) * v_norm

    return cte, lte

if __name__ == "__main__":

    from pyproj import Proj
    from sbet_Reader import read_sbet
    from absolute_navigator_ICP import quadrant
    import matplotlib.pyplot as plt
    transformer = Proj.from_crs(4326, 5972)
    import matplotlib.pyplot as plt





    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032-smrmsg.out"
    sbet_filename_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-EUREF89-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-EUREF89-UTC-Lidar-10Hz-1.743 0.044 -0.032-smrmsg.out"

    sbet_np, smrmsg_np = read_sbet(sbet_filename, smrmsg)
    sbet_np_PPP, smrmsg_np_PPP = read_sbet(sbet_filename_PPP, smrmsg_PPP)
    import numpy as np
    test = []
    # for k in sbet_np['heading']:
    #    test.append(quadrant(k)*180/np.pi)
    #    # print(quadrant(k)*180/np.pi)

    sbet_np["lon"]*180/np.pi
    lon_ETPOS = sbet_np["lon"]*180/np.pi
    lat_ETPOS = sbet_np["lat"]*180/np.pi
    lon_PPP = sbet_np_PPP["lon"] * 180 / np.pi
    lat_PPP = sbet_np_PPP["lat"] * 180 / np.pi

    # plt.plot(lat_ETPOS, lon_ETPOS)
    from pyproj import Transformer
    transformer = Transformer.from_crs(4937, 5972)
    import pandas as pd
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    current_epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [current_epoch for k in range(sbet_np_PPP['lat'].size)]
    X_PPP, Y_PPP, Z_PPP, epoch = transformer.transform(lat_PPP, lon_PPP, sbet_np_PPP['alt'], current_epoch)
    transformer = Transformer.from_crs(7912, 5972)
    X_ETPOS, Y_ETPOS, Z_ETPOS, epoch = transformer.transform(lat_ETPOS, lon_ETPOS, sbet_np['alt'], current_epoch)

    P_PPP = np.array([X_PPP, Y_PPP, Z_PPP])
    P_ETPOS = np.array([X_ETPOS, Y_ETPOS, Z_ETPOS])
    Cross = []
    long = []
    heading = []
    for head in sbet_np["heading"]:
        heading.append(quadrant(head))
    for k in range(P_PPP.shape[1]):
        PPP = P_PPP[:, k]
        ETPOS = P_ETPOS[:, k]
        cte, lte = c_l_track_error(ETPOS,PPP,heading[k])
        Cross.append(cte)
        long.append(lte)

    import matplotlib.pyplot as plt
    plt.plot(Cross,color = "red", label = "Cross-Track-Error")
    #plt.plot(long, color = "green")
    #plt.ylim([-1.2,1.2])
    plt.axhline(y=0.0, color='green', linestyle='-')
    plt.legend()
    plt.show()

    P_PPP = np.array([X_PPP, Y_PPP, Z_PPP])
    P_ETPOS = np.array([X_ETPOS, Y_ETPOS, Z_ETPOS])
    p2 = P_PPP[0:3,1]
    p1 = P_ETPOS[0:3,1]
    v = p2 - p1

    tg = (p2[0]-p1[0])/(p2[1]-p1[1])
    if p2[0]-p1[0] < 0 < p2[1]-p1[1]:
        tg = -tg + np.pi
    elif p2[0]-p1[0] < 0 and p2[1]-p1[1] < 0:
        tg = -tg + np.pi
    elif p2[1]-p1[1] < 0 < p2[0]-p1[0]:
        tg = -tg + 2*np.pi

    # Normalize the vector
    v_norm = np.linalg.norm(v)

    # Calculate the perpendicular vector from the starting point to the line connecting the two points
    # w = np.cross(v_norm, np.array([0, 0, 1]))
    # print(f'test{w} stop TEEEEST')
    # Calculate the cross-track error
    heading = quadrant(sbet_np["heading"][1])

    cte = tg * v_norm
    lte = np.cos(heading) * v_norm

