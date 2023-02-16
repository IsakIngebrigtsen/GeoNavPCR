def get_files(from_file_number = 2, number_of_files = 1):
    # Collects a number of files from the referance folder, max number of files are 44
    import os
    folder = "C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-PCAP"
    files = os.listdir(folder)
    y = []

    for name in files:
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

    file_name = get_files(9, 10)




