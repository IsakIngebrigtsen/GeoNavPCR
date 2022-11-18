import os
from ouster import client
import ouster.sdk.examples.pcap as pc
import ouster.pcap as pcap


if __name__ == "__main__":

    pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data\\OS-1-128_992035000186_1024x10"

    pcap_file = pathBase + ".pcap"
    meta = pathBase + ".json"
    with open(meta, 'r') as f:
        metadata = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_file, metadata)

    test = pc.pcap_to_ply(source, metadata, num=3, ply_dir='.', ply_base='ply_out', ply_ext='ply')




