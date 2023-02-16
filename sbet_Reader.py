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
    from pyproj import Proj
    transformer = Proj.from_crs(4326, 25832)
    import matplotlib.pyplot as plt

    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84-smrmsg.out"
    sbet_filename_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-LAZ\\sbet--UTCtime-Lillehammer_211021_3_7-LC_PPP-PPP-WGS84-smrmsg.out"
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
    X_ETPOS, Y_ETPOS = transformer.transform(sbet_np_ETPOS['lat'],sbet_np_ETPOS['lon'])
    X_PPP, Y_PPP = transformer.transform(sbet_np_PPP['lat'],sbet_np_PPP['lon'])
    X_stand, Y_stand = transformer.transform(sbet_np_stand['lat'],sbet_np_stand['lon'])

    std_ETPOS_PPP = np.sqrt((X_PPP-X_ETPOS)**2 + (Y_PPP-Y_ETPOS)**2)
    av = []
    for test in std_ETPOS_PPP:
        if test > 1.0:
            test = np.mean(av)
        av.append(test)
    # std_ETPOS_stand = np.sqrt((X_stand-X_ETPOS)**2 + (Y_stand-Y_ETPOS)**2)
    #plt.plot(std_ETPOS_PPP, color = "blue")
    #print(np.mean(std_ETPOS_PPP))
    import seaborn as sns

    fig, (ax1) = plt.subplots(1,1)
    fig.set_size_inches(30, 18.5, forward=True)
    ax1.set_ylim([-0.01,0.05])
    ax1.axhline(y=0.0, color='b', linestyle='-')
    ax1.plot(smrmsg_np_ETPOS["time"], smrmsg_np_ETPOS["alt-std"], color="blue", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["time"], smrmsg_np_ETPOS["lat-std"], color="green", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["time"], smrmsg_np_ETPOS["lon-std"], color="red", linewidth=0.4)
    ax1.grid(axis = 'y')
    fig.show()
    fig.savefig('Accurasy_ETPOS.png')

    fig, (ax1) = plt.subplots(1,1)
    fig.set_size_inches(30, 18.5, forward=True)
    ax1.set_ylim([-0.01,1])
    ax1.axhline(y=0.0, color='b', linestyle='-')
    ax1.plot(smrmsg_np_PPP["time"], smrmsg_np_PPP["alt-std"], color="blue", linewidth=0.4)
    ax1.plot(smrmsg_np_PPP["time"], smrmsg_np_PPP["lat-std"], color="green", linewidth=0.4)
    ax1.plot(smrmsg_np_PPP["time"], smrmsg_np_PPP["lon-std"], color="red", linewidth=0.4)
    ax1.grid(axis = 'y')
    fig.show()
    fig.savefig('Accurasy_PPP.png')

    fig, (ax2) = plt.subplots(1, 1)
    fig.set_size_inches(30, 18.5, forward=True)
    ax2.plot(X_PPP-X_ETPOS,linewidth=0.4)
    ax2.plot(Y_PPP - Y_ETPOS, linewidth=0.4)
    ax2.grid(axis='y')
    fig.savefig('std.png')


    fig, (ax1,ax2,ax3,ax4) = plt.subplots(4, 1)
    fig.set_size_inches(30, 18.5, forward=True)
    ax1.plot(smrmsg_np_ETPOS["alt-std"], color="blue", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["lat-std"], color="green", linewidth=0.4)
    ax1.plot(smrmsg_np_ETPOS["lon-std"], color="red", linewidth=0.4)


    # ax1.plot(X_stand, Y_stand)
    ax2.plot(std_ETPOS_PPP)
    ax3.plot(X_ETPOS, Y_ETPOS)
    ax4.plot(X_PPP, Y_PPP)
    # print(np.mean(std_ETPOS_stand))
    fig.show()
    fig.savefig('Accurasy.png')
    """
    from pandas_geojson import to_geojson, write_geojson
    import pandas as pd
    latlon = {'lat':sbet_np_ETPOS['lat'], 'lon': sbet_np_ETPOS['lon']}
    data = pd.DataFrame(latlon)
    #data = pd.read_csv('Test.csv')
    geo_json = to_geojson(df=data, lat='lat', lon='lon',
                          properties=[])
    write_geojson(geo_json, filename='random.geojson', indent=4)
    """