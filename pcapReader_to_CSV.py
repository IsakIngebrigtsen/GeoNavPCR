import os
import ouster.client as client
from ouster import pcap
from itertools import islice
import numpy as np
from typing import Tuple, List
import matplotlib.pyplot as plt
# Configure PCAP and JSON file paths
pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data\\OS-1-128_992035000186_1024x10"
pcap_file = pathBase + ".pcap"
meta = pathBase + ".json"
field_names = ""
field_fmts = ""
dual = False
# Read the metadata from the JSON file.
with open(meta, 'r') as f:
    metadata = client.SensorInfo(f.read())

# Open the LIDAR data source from the PCAP file
source = pcap.Pcap(pcap_file, metadata)


# Read the 50th LIDAR frame
# with closing(client.Scans(source)) as scans:
  #  scan = nth(scans, 2)

# Create a function that translates coordinates to a plottable coordinate system
xyzlut = client.XYZLut(source.metadata)
#xyz = xyzlut(scan)


def get_fields_info(scan: client.LidarScan) -> Tuple[str, List[str]]:
    field_names = 'TIMESTAMP (ns)'
    field_fmts = ['%d']
    for chan_field in scan.fields:
        field_names += f', {chan_field}'
        if chan_field in [client.ChanField.RANGE, client.ChanField.RANGE2]:
            field_names += ' (mm)'
        field_fmts.append('%d')
    field_names += ', X (mm), Y (mm), Z (mm)'
    field_fmts.extend(3 * ['%d'])
    if dual:
        field_names += ', X2 (mm), Y2 (mm), Z2 (mm)'
        field_fmts.extend(3 * ['%d'])
    return field_names, field_fmts

num = 50

# create an iterator of LidarScans from pcap and bound it if num is specified
scans = iter(client.Scans(source))
if num:
    scans = islice(scans, num)

for idx, scan in enumerate(scans):
    print(idx)
    # initialize the field names for csv header
    if not field_names or not field_fmts:
        field_names, field_fmts = get_fields_info(scan)

    # copy per-column timestamps for each channel
    timestamps = np.tile(scan.timestamp, (scan.h, 1))

    # grab channel data
    fields_values = [scan.field(ch) for ch in scan.fields]

    # use integer mm to avoid loss of precision casting timestamps
    xyz = (xyzlut(scan.field(client.ChanField.RANGE)) * 1000).astype(
        np.int64)

    if dual:
        xyz2 = (xyzlut(scan.field(client.ChanField.RANGE2)) * 1000).astype(
            np.int64)

        # get all data as one H x W x num fields int64 array for savetxt()
        frame = np.dstack((timestamps, *fields_values, xyz, xyz2))

    else:
        # get all data as one H x W x num fields int64 array for savetxt()
        frame = np.dstack((timestamps, *fields_values, xyz))

    # not necessary, but output points in "image" vs. staggered order
    frame = client.destagger(metadata, frame)

    # write csv out to file

    csv_base = 10
    csv_ext = "csv"
    csv_dir = 'C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data'
    csv_path = os.path.join(csv_dir, f'{csv_base}_{idx:06d}.{csv_ext}')
    print(f'write frame #{idx}, to file: {csv_path}')

    header = '\n'.join([f'frame num: {idx}', field_names])

    np.savetxt(csv_path,
               frame.reshape(-1, frame.shape[2]),
               fmt=field_fmts,
               delimiter=',',
               header=header)


"""Load the data back into numpy arrays"""

import numpy as np

# read array from CSV
frame = np.loadtxt('C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data\\10_000000.csv', delimiter=',')
extractedData = frame[:, [5, 6, 7]]
# convert back to "fat" 2D image [H x W x num_fields] shape
frame = frame.reshape((128, -1, frame.shape[1]))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(extractedData[:, 0], extractedData[:, 1], extractedData[:, 2], c=extractedData[:, 2], alpha=1, vmin=0, vmax=5000)
ax.set_xlim(-50000,50000)
ax.set_ylim(-50000,50000)
ax.set_zlim(0,20000)
plt.show()
