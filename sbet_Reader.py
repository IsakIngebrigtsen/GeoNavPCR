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
    sbet_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-EUREF89-UTC-Lidar-10Hz-1.743 0.044 -0.032.out"
    smrmsg_PPP = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\Lillehammer_211021_3_7-LC_PPP - SBS-EUREF89-UTC-Lidar-10Hz-1.743 0.044 -0.032-smrmsg.out"
    smrmsg_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\realtime-sbet-output-UTC-1000.out"
    sbet_filename_stand = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Sbet\\realtime-sbet-output-UTC-1000-smrmsg.out"

    FROM_CRS = 4326 #WGS84
    TO_CRS =  25832 #UTM32
    sbet_np_ETPOS, smrmsg_np_ETPOS  = read_sbet(sbet_ETPOS, smrmsg_ETPOS)
    # sbet_np_ETPOS, smrmsg_np_ETPOS  =read_sbet(sbet_PPP, smrmsg_PPP)

    # sbet_np_stand, smrmsg_np_stand = read_sbet(sbet_filename_stand, smrmsg_stand)
    # lat = np.mean(smrmsg_np["lat-std"])
    # lon = np.mean(smrmsg_np["lon-std"])
    # alt = np.mean(smrmsg_np["lon-std"])
    # time = np.mean(smrmsg_np["time"])
    # print(lat,lon,alt,time)
    gpsweek = filename2gpsweek("C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\PPP-Standalone-PCAP\\OS-1-128_992035000186_1024x10_20211021_200041.pcap")
    ar_from = 0
    ar_to = 19200
    # Round 1 0 9660
    # Round 2 10000 19200
    mean_lat = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['lat-std'][ar_from:ar_to]**2)/(len(smrmsg_np_ETPOS['lat-std'][ar_from:ar_to]))), 3)
    mean_lon = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['lon-std'][ar_from:ar_to]**2)/(len(smrmsg_np_ETPOS['lon-std'][ar_from:ar_to]))), 3)
    mean_alt = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['alt-std'][ar_from:ar_to]**2)/(len(smrmsg_np_ETPOS['alt-std'][ar_from:ar_to]))), 3)
    plt.style.use('fivethirtyeight')
    plt.rcParams.update({'font.size': 18})
    fig, ax = plt.subplots(figsize=(20, 10))
    # ax.set_facecolor('#eafff5')
    fig.suptitle('INS standard deviation, Processed with ETPOS, Round 1, and Round 2',fontsize=24)
    ax.set_title('The ETPOS solution from Round 1 is used to georeference the source point cloud',fontsize=16)
    ax.plot(smrmsg_np_ETPOS['time'][ar_from:ar_to]-smrmsg_np_ETPOS['time'][ar_from], smrmsg_np_ETPOS['lat-std'][ar_from:ar_to], linewidth=2.7, label=f'Latitude std, mean:{mean_lat} m')
    ax.plot(smrmsg_np_ETPOS['time'][ar_from:ar_to]-smrmsg_np_ETPOS['time'][ar_from], smrmsg_np_ETPOS['lon-std'][ar_from:ar_to], linewidth=2.7, label=f'Longitude std, mean:{mean_lon} m ')
    ax.plot(smrmsg_np_ETPOS['time'][ar_from:ar_to]-smrmsg_np_ETPOS['time'][ar_from], smrmsg_np_ETPOS['alt-std'][ar_from:ar_to], linewidth=2.7, label=f'Altitude std, mean:{mean_alt} m')
    ax.axvline(x=960, color='black', linestyle='--', label = f"Line of where Round 1 stop, and Round 2 start", linewidth=2.2)
    ax.set_ylabel('Distance (meter)')
    ax.set_xlabel('Time (second)')
    ax.set_ylim([-0.002, 0.03])
    ax.set_xlim([-40, 2040])
    # ax.grid()
    # get the legend object
    leg = ax.legend()

    # change the line width for the legend
    for line in leg.get_lines():
        line.set_linewidth(4.0)
    # ax.grid(linestyle='-', linewidth=0.5)
    plt.show()
    fig.savefig('ETPOS_std_Truetraj.png')
    """
    mean_lat = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['lat-std']**2)/(len(smrmsg_np_ETPOS['lat-std'])-1)), 3)
    mean_lon = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['lon-std']**2)/(len(smrmsg_np_ETPOS['lon-std'])-1)), 3)
    mean_alt = np.round(np.sqrt(np.sum(smrmsg_np_ETPOS['alt-std']**2)/(len(smrmsg_np_ETPOS['alt-std'])-1)), 3)
    plt.style.use('bmh')
    fig, ax = plt.subplots(figsize=(20, 10))
    # ax.set_facecolor('#eafff5')
    ax.set_title('INS standard deviation, after processing with PPP')
    ax.plot(smrmsg_np_ETPOS['time'], smrmsg_np_ETPOS['lat-std'], linewidth=0.2, label=f'Latitude std, mean:{mean_lat} m')
    ax.plot(smrmsg_np_ETPOS['time'], smrmsg_np_ETPOS['lon-std'], linewidth=0.2, label=f'longitude std, mean:{mean_lon} m ')
    ax.plot(smrmsg_np_ETPOS['time'], smrmsg_np_ETPOS['alt-std'], linewidth=0.2, label=f'altitude std, mean:{mean_alt} m')
    ax.set_ylabel('meter')
    ax.set_xlabel('Time in seconds')
    ax.set_ylim([-0.005,1])
    ax.legend()
    # ax.grid(linestyle='-', linewidth=0.5)
    plt.show()
    fig.savefig('PPP_std.png')
    """

    # sbet_PPP['lat'] = sbet_PPP['lat'] * 180 / np.pi
    # sbet_PPP['lon'] = sbet_PPP['lon'] * 180 / np.pi
    # sbet_filename_stand['lat'] = sbet_filename_stand['lat'] * 180 / np.pi
    # sbet_filename_stand['lon'] = sbet_filename_stand['lon'] * 180 / np.pi

    doy = pd.Period("2021-10-21", freq="H").day_of_year
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    sbet_np_ETPOS, smrmsg_np_ETPOS  = read_sbet(smrmsg_stand, sbet_filename_stand)
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    transformer = Transformer.from_crs(4327, 5972)
    true_X = sbet_np_ETPOS['lat']*180 / np.pi
    true_Y = sbet_np_ETPOS['lon'] * 180 / np.pi
    X_ETPOS_EUREF89, Y_ETPOS_EUREF89, Z_ETPOS_EUREF89, epoch = transformer.transform(true_X, true_Y, sbet_np_ETPOS['alt'], current_epoch)
    ETPOS_trajectory_EUREF89 = np.array([X_ETPOS_EUREF89[ar_from:ar_to], Y_ETPOS_EUREF89[ar_from:ar_to], Z_ETPOS_EUREF89[ar_from:ar_to]]).T

    # epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    # current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    # current_epoch = [epoch] * sbet_np_ETPOS['lat'].size
    # transformer = Transformer.from_crs(4937, 5972)
    # X_PPP_ITRF14, Y_PPP_ITRF14, Z_PPP_ITRF14, epoch = transformer.transform(sbet_PPP['lat'], sbet_PPP['lon'], sbet_PPP['alt'], current_epoch)
    # PPP_trajectory_ITRF14 = np.array([X_PPP_ITRF14[0:9660], Y_PPP_ITRF14[0:9660], Z_PPP_ITRF14[0:9660]]).T

    plt.plot(ETPOS_trajectory_EUREF89[:,0], ETPOS_trajectory_EUREF89[:,1])
    # target = np.load("pros_data\\Standalone\\Round2_fulltraj_corrected_outliers\\target_coord_2023-03-23_0626.npy")
    #plt.plot(target[:,0],target[:,1])
    plt.show()
    sbet_np_ETPOS, smrmsg_np_ETPOS  = read_sbet(smrmsg_stand, sbet_filename_stand)
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    transformer = Transformer.from_crs(7912, 5972)
    true_X = sbet_np_ETPOS['lat']*180 / np.pi
    true_Y = sbet_np_ETPOS['lon'] * 180 / np.pi
    X_ETPOS_ITRF14, Y_ETPOS_ITRF14, Z_ETPOS_ITRF14, epoch = transformer.transform(true_X, true_Y, sbet_np_ETPOS['alt'], current_epoch)

    ETPOS_trajectory_ITRF14 = np.array([X_ETPOS_ITRF14[ar_from:ar_to], Y_ETPOS_ITRF14[ar_from:ar_to], Z_ETPOS_ITRF14[ar_from:ar_to]]).T


    sbet_np_ETPOS_true, smrmsg_np_ETPOS  =read_sbet(sbet_ETPOS, smrmsg_ETPOS)
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS_true['lat'].size)]
    transformer = Transformer.from_crs(4937, 5972)
    true_X = sbet_np_ETPOS_true['lat']*180 / np.pi
    true_Y = sbet_np_ETPOS_true['lon'] * 180 / np.pi
    X_ETPOS_EUREF89, Y_ETPOS_EUREF89, Z_ETPOS_EUREF89, epoch = transformer.transform(true_X, true_Y, sbet_np_ETPOS_true['alt'], current_epoch)

    # X_PPP_ITRF14, Y_PPP_ITRF14, Z_PPP_ITRF14, epoch = transformer.transform(sbet_PPP['lat'], sbet_PPP['lon'], sbet_PPP['alt'], current_epoch)
    # PPP_trajectory_ITRF14 = np.array([X_PPP_ITRF14[0:9660], Y_PPP_ITRF14[0:9660], Z_PPP_ITRF14[0:9660]]).T
    ETPOS_trajectory_EUREF89_truetraj = np.array([X_ETPOS_EUREF89[ar_from:ar_to], Y_ETPOS_EUREF89[ar_from:ar_to], Z_ETPOS_EUREF89[ar_from:ar_to]]).T

    sbet_np_ETPOS, smrmsg_np_ETPOS  = read_sbet(sbet_PPP, smrmsg_PPP)
    epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    current_epoch = [epoch for k in range(sbet_np_ETPOS['lat'].size)]
    transformer = Transformer.from_crs(4937, 5972)
    true_X = sbet_np_ETPOS['lat']*180 / np.pi
    true_Y = sbet_np_ETPOS['lon'] * 180 / np.pi
    X_PPP_ITRF14, Y_PPP_ITRF14, Z_PPP_ITRF14, epoch = transformer.transform(true_X, true_Y, sbet_np_ETPOS['alt'], current_epoch)

    PPP_trajectory_ITRF14 = np.array([X_PPP_ITRF14[ar_from:ar_to], Y_PPP_ITRF14[ar_from:ar_to], Z_PPP_ITRF14[ar_from:ar_to]]).T


    #st = ETPOS_trajectory_ITRF14 - ETPOS_trajectory_EUREF89
    #Y = ETPOS_trajectory_ITRF14 - PPP_trajectory_ITRF14
    #print(np.mean(np.sqrt(st[:,0]**2+st[:,1]**2)))
    np.mean(ETPOS_trajectory_EUREF89_truetraj[1000:1500,0:2]-ETPOS_trajectory_EUREF89[2000:3000:2,0:2])

    plt.plot(ETPOS_trajectory_ITRF14[2000:2500:2,0],ETPOS_trajectory_ITRF14[2000:2500:2,1], label = "7912", )
    plt.plot(ETPOS_trajectory_EUREF89_truetraj[1000:1250,0],ETPOS_trajectory_EUREF89_truetraj[1000:1250,1], label = "Isaks 4937")
    plt.plot(ETPOS_trajectory_EUREF89[2000:2500:2,0],ETPOS_trajectory_EUREF89[2000:2500:2,1],':', label = "43264937",)
    plt.plot(PPP_trajectory_ITRF14[1000:1250,0],PPP_trajectory_ITRF14[1000:1250,1], label = "Isaks 7912")

    plt.legend()
    plt.show()
    # np.save("pros_data\\PPP_traj_ITRF14.npy", PPP_trajectory_ITRF14)
    # np.save("pros\\erlend_test.npy", ETPOS_trajectory_EUREF89)
    # print("whop done")
    import numpy as np
    import time
    import logging as log
    import datetime
    import pandas as pd
    from pyproj import Transformer
    doy = pd.Period("2021-10-21", freq="H").day_of_year
    current_epoch = int(2021) + int(doy) / 365  # Current Epoch ex: 2021.45
    transformer = Transformer.from_crs(7912, 5972)
    true_X = 61.11310191
    true_Y = 10.4773705
    true_z = 294.1767326304689
    X, Y, Z, epoch = transformer.transform(true_X, true_Y, true_z, current_epoch)
    print(X,Y,Z)
    transformer = Transformer.from_crs(4937, 5972)
    X, Y, Z, epoch = transformer.transform(true_X, true_Y, true_z, current_epoch)
    print(X,Y,Z)