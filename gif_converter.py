
fp_in = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\master_code\\Master_thesis\\img_Pointcloud\\"
fp_out = "file_10_frame_0_196_uncorrected.gif"
import sys
sys.path.insert(0, "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\master_code\\Master_thesis\\img_Pointcloud")
import os
file_names = sorted((fn for fn in os.listdir(fp_in) if fn.endswith('.png')))

import matplotlib.animation as animation
import matplotlib.pyplot as plt

# Create new figure for GIF
fig, ax = plt.subplots()

# Adjust figure so GIF does not have extra whitespace
fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
ax.axis('off')
ims = []

for file in file_names:
    im = ax.imshow(plt.imread(fp_in+file), animated = True)
    ims.append([im])

ani = animation.ArtistAnimation(fig, ims, interval=200)
ani.save('us_population.gif')


