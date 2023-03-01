import numpy as np
import time
import logging as log
import datetime
import pandas as pd
import pyproj
""" 
Code established from Superviser Morten Brunes
"""

# LEAP SECONDS
DELTA_UNIX_GPS = 18
datetimeformat = "%Y%m%d_%H%M%S"
epoch = datetime.datetime.strptime("19800106_000000", datetimeformat)


# finner gps uke fra dato i filnavn pcapfil
def filename2gpsweek(pcap_file):
    ymd_hms = (pcap_file.split('x')[1]).split('_')[1] + '_' + \
              (pcap_file.split('x')[1]).split('_')[2].replace('.pcap', '')
    utc = datetime.datetime.strptime(ymd_hms, datetimeformat)
    tdiff = utc - epoch + datetime.timedelta(seconds=DELTA_UNIX_GPS)
    gpsweek = tdiff.days // 7
    return gpsweek


# regner om fra unix time til gps seconds of week. Lidar bruker unix og sbet bruker seconds of week (SoW)
def timestamp_unix2sow(unix, gps_week):
    # Correction by Erlend: subtract epoch unix time as well!
    # Another correction (?) by Erlend: removed subtraction of DELTA_UNIX_GPS -- this makes PCAP and SBET correspond.
    sow = unix - 315964800 - (gps_week * 604800)
    return sow


def timestamp_sow2unix(sow, gps_week):
    unix = sow + 315964800 + (gps_week * 604800)
    return unix


def read_sbet(sbet_filename, smrmsg) -> np.array:
    start_sbet = time.time()

    sbet_record_types = [

        ("time", np.float64),

        ("lat", np.float64),  # radians

        ("lon", np.float64),  # radians

        ("alt", np.float64),

        ("x-vel", np.float64),  # m/s

        ("y-vel", np.float64),

        ("vert-vel", np.float64),

        ("roll", np.float64),  # radians

        ("pitch", np.float64),

        ("heading", np.float64),

        ("wander", np.float64),  # radians

        ("x-acc", np.float64),  # m/s^2

        ("y-acc", np.float64),

        ("vert-acc", np.float64),

        ("x-angrate", np.float64),  # radians/s

        ("y-angrate", np.float64),

        ("z-angrate", np.float64)

    ]

    smrmsg_record_types = [

        ("time", np.float64),

        ("lat-std", np.float64),  # radians

        ("lon-std", np.float64),  # radians

        ("alt-std", np.float64), # sier noe om nøyaktigheten på fila. Punktskyen er direkte avledet av navigasjonen. 1-2 cm

        ("roll-std", np.float64),

        ("pitch-std", np.float64),

        ("yaw-std", np.float64),

        ("unknown1", np.float64),  # akselerasjon std.dev?

        ("unknown2", np.float64),

        ("unknown3", np.float64)

    ]

    sbet_np = np.fromfile(sbet_filename, dtype=np.dtype(sbet_record_types))

    smrmsg_np = np.fromfile(smrmsg, dtype=np.dtype(smrmsg_record_types))

    log.debug("Time read sbet: {:.3f}".format(time.time() - start_sbet))

    return sbet_np, smrmsg_np


if __name__ == "__main__":
    from pyproj import Transformer
    transformer = Transformer.from_crs(4937, 5972)
    import matplotlib.pyplot as plt

    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84-smrmsg.out"
    sbet_filename_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\sbet-output-UTC-1000.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\sbet-output-UTC-1000-smrmsg.out"
    smrmsg_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84-smrmsg.out"
    sbet_filename_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84.out"

    FROM_CRS = 4326 #WGS84
    TO_CRS =  25832 #UTM32
    sbet_np_ETPOS, smrmsg_np_ETPOS  =read_sbet(sbet_filename, smrmsg)
    sbet_np_PPP, smrmsg_np_PPP  =read_sbet(sbet_filename_PPP, smrmsg_PPP)
    sbet_np_stand, smrmsg_np_stand = read_sbet(sbet_filename_stand, smrmsg_stand)
    # lat = np.mean(smrmsg_np["lat-std"])
    # lon = np.mean(smrmsg_np["lon-std"])
    # alt = np.mean(smrmsg_np["lon-std"])
    # time = np.mean(smrmsg_np["time"])
    # print(lat,lon,alt,time)
    gpsweek = filename2gpsweek("C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-Standalone-PCAP\\OS-1-128_992035000186_1024x10_20211021_200041.pcap")
    # plt.plot(sbet_np_ETPOS['lat'], sbet_np_ETPOS['lon'], color = "red")
    # plt.plot(sbet_np_PPP['lat'], sbet_np_PPP['lon'], color = "blue")

    sbet_np_ETPOS['lat'] = sbet_np_ETPOS['lat']*180 / np.pi
    sbet_np_ETPOS['lon'] = sbet_np_ETPOS['lon'] * 180 / np.pi
    sbet_np_PPP['lat'] = sbet_np_PPP['lat'] * 180 / np.pi
    sbet_np_PPP['lon'] = sbet_np_PPP['lon'] * 180 / np.pi
    sbet_np_stand['lat'] = sbet_np_stand['lat'] * 180 / np.pi
    sbet_np_stand['lon'] = sbet_np_stand['lon'] * 180 / np.pi
    X_ETPOS, Y_ETPOS,Z_ETPOS = transformer.transform(sbet_np_ETPOS['lat'],sbet_np_ETPOS['lon'],sbet_np_ETPOS['alt'])

    import pandas as pd
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    current_epoch = int(2021) + int(doy)/365  # Current Epoch ex: 2021.45
    current_epoch = [current_epoch for k in range(sbet_np_PPP['lat'].size)]
    transformer = Transformer.from_crs(4326, 5972)
    X_PPP, Y_PPP, Z_PPP, epoch = transformer.transform(sbet_np_PPP['lat'], sbet_np_PPP['lon'], sbet_np_PPP['alt'], current_epoch)
    X_stand, Y_stand = transformer.transform(sbet_np_stand['lat'], sbet_np_stand['lon'])

    # #std_ETPOS_PPP = np.sqrt((X_PPP-X_ETPOS)**2 + (Y_PPP-Y_ETPOS)**2)
    # av = []
    # for test in std_ETPOS_PPP:
    #     if test > 1.0:
    #         test = np.mean(av)
    #     av.append(test)
    # std_ETPOS_stand = np.sqrt((X_stand-X_ETPOS)**2 + (Y_stand-Y_ETPOS)**2)
    #plt.plot(std_ETPOS_PPP, color = "blue")
    #print(np.mean(std_ETPOS_PPP))

    fig, (ax1) = plt.subplots(1, 1)
    fig.set_size_inches(30, 30, forward=True)
    ax1.plot(smrmsg_np_ETPOS["alt-std"], color="blue", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["lat-std"], color="green", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["lon-std"], color="red", linewidth=0.4)


    # ax1.plot(X_stand, Y_stand)
    #ax2.plot(np.round(std_ETPOS_PPP[52000:55000], 4))
    """
    ax3.plot(X_ETPOS, Y_ETPOS)
    ax3.plot(X_PPP, Y_PPP)
    # print(np.mean(std_ETPOS_stand))
    fig.show()
    fig.savefig('Accurasy.png')

    # """
    # from pandas_geojson import to_geojson, write_geojson
    # import pandas as pd
    # latlon = {'lat':sbet_np_ETPOS['lat'], 'lon': sbet_np_ETPOS['lon']}
    # data = pd.DataFrame(latlon)
    # #data = pd.read_csv('Test.csv')
    # geo_json = to_geojson(df=data, lat='lat', lon='lon',
    #                       properties=[])
    # write_geojson(geo_json, filename='random.geojson', indent=4)
    # """

    """
    Per Helges kode test
    """
    """
    import pyproj
    print(pyproj.datadir.get_user_data_dir())
    from pyproj import Transformer, transform
    import pandas as pd
    X_ref2 = 3149785.9652
    Y_ref2 = 598260.8822
    Z_ref2 = 5495348.4927
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    current_epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    transformer = Transformer.from_crs(4936, 25832)
    X,Y,Z_UTM, epoch = transformer.transform(X_ref2, Y_ref2, Z_ref2, current_epoch)

    WGS84 = [sbet_np_PPP['lat'][0], sbet_np_PPP['lon'][0], sbet_np_PPP['alt'][0]]
    transformer = Transformer.from_crs(7912, 25832)
    X_84,Y_84,Z_84, epoch = transformer.transform(WGS84[0], WGS84[1], WGS84[2], current_epoch)
    EUREF89 = [sbet_np_PPP['lat'][0], sbet_np_PPP['lon'][0], sbet_np_PPP['alt'][0]]
    transformer = Transformer.from_crs(4258, 25832)
    X_89,Y_89,Z_89, epoch = transformer.transform(EUREF89[0], EUREF89[1], EUREF89[2], current_epoch)

    res = np.array([X_84, Y_84, Z_84]) - np.array([X_89,Y_89,Z_89])
    """
