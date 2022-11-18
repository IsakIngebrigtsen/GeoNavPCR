from ICP_Point import get_frame

pathBase = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\data\\OS-1-128_992035000186_1024x10"

pcap_file = pathBase + ".pcap"
meta = pathBase + ".json"
accumulatedTime = 0.0
source = get_frame(pcap_file, meta, 90)
source_1 = get_frame(pcap_file, meta, 91)
source = source.reshape((-1, 3))
source_1 = source_1.reshape((-1, 3))

