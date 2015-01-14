#!/usr/bin/python

import glob;
import os;

command = r'"C:\Program Files\VCG\MeshLab\meshlabserver.exe"';

output_directory = "obj"

filenamelist = glob.glob("*.ply");
filenamelist.sort();

for filename in filenamelist:
	newfilename = filename.replace(".ply",".obj");
	newfilename = "patch_" + newfilename;
	os.system( command + " -i " + filename + " -o " + output_directory + "/" + newfilename );
	print(newfilename);