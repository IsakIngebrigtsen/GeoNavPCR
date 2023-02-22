

if __name__ == "__main__":

    from pyproj import Proj
    from sbet_Reader import read_sbet
    from absolute_PCAP_ICP import quadrant
    import matplotlib.pyplot as plt
    transformer = Proj.from_crs(4326, 5972)
    import matplotlib.pyplot as plt

    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84-smrmsg.out"
    sbet_filename_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84-smrmsg.out"

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
    transformer = Transformer.from_crs(9000, 5972)
    import pandas as pd
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    current_epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [current_epoch for k in range(sbet_np_PPP['lat'].size)]
    X_PPP, Y_PPP, Z_PPP, epoch = transformer.transform(lat_PPP, lon_PPP, sbet_np_PPP['alt'], current_epoch)
    X_ETPOS, Y_ETPOS, Z_ETPOS, epoch = transformer.transform(lat_ETPOS, lon_ETPOS, sbet_np['alt'], current_epoch)
    X_PPP = X_PPP - X_ETPOS[0]
    X_ETPOS = X_ETPOS - X_ETPOS[0]
    Y_PPP = Y_PPP - Y_ETPOS[0]
    Y_ETPOS = Y_ETPOS - Y_ETPOS[0]
    dev_y = Y_ETPOS - Y_PPP
    dev_x = X_ETPOS-X_PPP
    from sklearn.linear_model import LinearRegression

    model = LinearRegression()
    model.fit(sbet_np["time"], np.sqrt(dev_y**2+dev_x**2))
    r_sq = model.score(sbet_np["time"], np.sqrt(dev_y**2+dev_x**2))
    plt.plot(sbet_np["time"], np.sqrt(dev_y**2+dev_x**2))

    plt.show()

