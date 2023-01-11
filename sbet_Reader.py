import numpy as np
import PDAL

def read_sbet(sbet_filename, smrmsg) -> np.array:
    start_sbet = timer()

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

        ("alt-std", np.float64),

        ("roll-std", np.float64),

        ("pitch-std", np.float64),

        ("yaw-std", np.float64),

        ("unknown1", np.float64),  # akselerasjon std.dev?

        ("unknown2", np.float64),

        ("unknown3", np.float64)

    ]

    sbet_np = np.fromfile(sbet_filename, dtype=np.dtype(sbet_record_types))

    smrmsg_np = np.fromfile(smrmsg, dtype=np.dtype(smrmsg_record_types))

    log.debug("Time read sbet: {:.3f}".format(timer() - start_sbet))

    return sbet_np, smrmsg_np


if __name__ == "__main__":
    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84 (3).out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84-smrmsg (2).out"
    sbet_np, smrmsg_np =read_sbet(sbet_filename, smrmsg)


