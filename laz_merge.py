import sys
import traceback
import laspy
import os

try:
    print('Running Merge LAS')

    # This is the las file to append to.  DO NOT STORE THIS FILE IN THE SAME DIRECTORY AS BELOW...
    out_las = 'C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\1024x10_20211021_194456.laz'
    # this is a directory of las files
    inDir = 'C:\\Users\\isakf\\Documents\\1_Geomatikk\\Master\\Data\\Referansepunktsky-LAZ'


    def append_to_las(in_laz, out_las):
        with laspy.open(out_las, mode='a') as outlas:
            with laspy.open(in_las) as inlas:
                for points in inlas.chunk_iterator(2_000_000):
                    outlas.append_points(points)


    for (dirpath, dirnames, filenames) in os.walk(inDir):
        for inFile in filenames:
            if inFile.endswith('.laz'):
                in_las = os.path.join(dirpath, inFile)
                append_to_las(in_las, out_las)

    print('Finished without errors - merge_LAS.py')
except:
    tb = sys.exc_info()[2]
    tbinfo = traceback.format_tb(tb)[0]
    print('Error in append las')
    print("PYTHON ERRORS:\nTraceback info:\n" + tbinfo + "\nError     Info:\n" + str(sys.exc_info()[1]))