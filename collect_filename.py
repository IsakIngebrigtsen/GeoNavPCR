def get_files(from_file_number=2, number_of_files=1, folder="Round1"):
    # Collects a number of files from the referance folder, max number of files are 44
    import os

    folder_round1 = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Raw_Frames_Round_1"
    folder_round2 = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Raw_Frames_Round_2"

    if folder == "Round1":
        folder_dir = folder_round1
    else:
        folder_dir = folder_round2

    files_list = os.listdir(folder_dir)
    y = []

    for name in files_list:
        x = name.split("_")
        if 'json' in x[4]:
            continue
        y.append(x[4].split(".")[0])

    if from_file_number > len(y):
        return print("There are not that many files")

    if from_file_number + number_of_files > len(y):
        number_of_files = len(y)-from_file_number
        print(f'There are not enough files to pick, so it got reduced to {number_of_files}')
        return y[from_file_number:from_file_number+number_of_files]

    return y[from_file_number:from_file_number+number_of_files]


if __name__ == "__main__":

    from absolute_navigator_ICP import read_laz
    file_name = get_files(7, 5, folder = "Round1")
    # h = 0
    # for k in range(0,198,5):
    #     h += 1
    # for files in file_name:  # For loop that goes through the PCAP files, and the corresponding laz files.
        # Source_init is the raw laz file corresponding to the PCAP file
    #     source_init = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ\\1024x10_20211021_" + files + ".laz"
        # Transformes the Laz file into Open 3d point cloud.
    #     pc_raw_laz = read_laz(source_init)

