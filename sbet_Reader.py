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

        ("lat-std", np.float64),  # meter

        ("lon-std", np.float64),  # meter

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

    sbet_ETPOS = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032.out"
    smrmsg_ETPOS = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-TC_PPK - SBS-WGS84-UTC-10Hz-Lidar-1.743 0.044 -0.032-smrmsg.out"
    sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-WGS84-UTC-Lidar-10Hz-1.743 0.044 -0.032-smrmsg.out"
    smrmsg_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84-smrmsg.out"
    sbet_filename_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Standalone-LAZ\\sbet-UTCtime-211021-Lillehammer-standalone-RT - PPP-WGS84.out"

    FROM_CRS = 4326 #WGS84
    TO_CRS =  25832 #UTM32
    sbet_np_ETPOS, smrmsg_np_ETPOS  =read_sbet(sbet_ETPOS, smrmsg_ETPOS)
    sbet_np_PPP, smrmsg_np_PPP  =read_sbet(sbet_PPP, smrmsg_PPP)

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
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    current_epoch = [epoch] * sbet_np_ETPOS['lat'].size
    transformer = Transformer.from_crs(4937, 5972)

    X_ETPOS_ITRF14, Y_ETPOS_ITRF14,Z_ETPOS_ITRF14,epoch = transformer.transform(sbet_np_ETPOS['lat'],sbet_np_ETPOS['lon'],sbet_np_ETPOS['alt'],current_epoch)

    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    current_epoch = [epoch] * sbet_np_PPP['lat'].size
    transformer = Transformer.from_crs(7912, 5972)
    X_PPP_ITRF14, Y_PPP_ITRF14, Z_PPP_ITRF14, epoch = transformer.transform(sbet_np_PPP['lat'], sbet_np_PPP['lon'], sbet_np_PPP['alt'], current_epoch)
    PPP_trajectory_ITRF14 = np.array([X_PPP_ITRF14, Y_PPP_ITRF14, Z_PPP_ITRF14]).T
    ETPOS_trajectory_ITRF14 = np.array([X_ETPOS_ITRF14, Y_ETPOS_ITRF14, Z_ETPOS_ITRF14]).T
    st = ETPOS_trajectory_ITRF14 - PPP_trajectory_ITRF14
    Y = ETPOS_trajectory_ITRF14 - PPP_trajectory_ITRF14
    print(np.mean(np.sqrt(st[:,0]**2+st[:,1]**2)))

    # np.save("pros_data\\PPP_traj_ITRF14.npy", PPP_trajectory_ITRF14)
    # np.save("pros_data\\ETPOS_trajectory_ITRF14.npy", ETPOS_trajectory_ITRF14)
    print("whop done")
