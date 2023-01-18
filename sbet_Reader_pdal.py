def read_sbet_pdal(sbet_file, smrmsg_file, FROM_CRS, TO_CRS):
    """

    Reads SBET and smrms with PDAL

    Merge to one df

    Rename and transform coordinates

    """

    pipeline_sbet = pdal.Reader.sbet(filename=sbet_file).pipeline()

    pipeline_sbet.execute()

    sbet_file_np = pipeline_sbet.arrays

    pipeline_smrmsg = pdal.Reader.smrmsg(filename=smrmsg_file).pipeline()

    pipeline_smrmsg.execute()

    smrmsg_file_np = pipeline_smrmsg.arrays

    sbet_df = pd.DataFrame(sbet_file_np[0])

    smrmsg_df = pd.DataFrame(smrmsg_file_np[0])

    sbet_df = sbet_df.merge(smrmsg_df)

    transformer = pyproj.Transformer.from_crs(FROM_CRS, TO_CRS)

    sbet_df["East"], sbet_df["North"], sbet_df['Height'] = transformer.transform(sbet_df["Y"].values,
                                                                                 sbet_df["X"].values,
                                                                                 sbet_df["Z"].values)

    sbet_df = sbet_df.rename(columns={"X": "Lon", "Y": "Lat", "Z": "Height ell"})

    return sbet_df


if __name__ == "__main__":
    import pdal
    import pyproj
    import pandas as pd

    sbet_filename = "C:\\Users\\isakf\\Documents\\1_Geomatikk\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84.out"
    smrmsg = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Lillehammer_211021_3_7-sbet-200Hz-WGS84-smrmsg.out"

    FROM_CRS = 4326  # WGS84
    TO_CRS = 25832  # UTM32

    sbet_df = read_sbet_pdal(sbet_filename, smrmsg, FROM_CRS, TO_CRS)

    sbet_df.to_csv("C:\\Users\\isakf\\Documents\\1_Geomatikk\Master\\Data\\Sbet_Data.txt", sep=',', mode='w')
