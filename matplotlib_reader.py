
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import ouster.pcap as pcap
import ouster.client as client

from contextlib import closing
from more_itertools import nth, islice_extended
from itertools import islice

# Configure PCAP and JSON file paths
pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data\\OS-1-128_992035000186_1024x10"
pcap_file = pathBase + ".pcap"
meta = pathBase + ".json"

# Read the metadata from the JSON file.
with open(meta, 'r') as f:
    metadata = client.SensorInfo(f.read())

# Open the LIDAR data source from the PCAP file
source = pcap.Pcap(pcap_file, metadata)

# Read the 50th LIDAR frame
with closing(client.Scans(source)) as scans:
    scan = nth(scans, 1)

# Create a function that translates coordinates to a plottable coordinate system
xyzlut = client.XYZLut(source.metadata)
xyz = xyzlut(scan)

# Create a 3D matplotlib plot showing the data
[x, y, z] = [c.flatten() for c in np.dsplit(xyz, 3)]
ax = plt.axes(projection='3d')
r = 20
ax.set_xlim3d([-r, r])
ax.set_ylim3d([-r, r])
ax.set_zlim3d([-r/2, r/2])
plt.axis('off')
z_col = np.minimum(np.absolute(z), 5)
ax.scatter(x, y, z, c=z_col, s=0.2)
plt.show()


