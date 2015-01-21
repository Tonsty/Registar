#!/usr/bin/python

import glob;
import os;

command = r'"C:\Program Files\VCG\MeshLab\meshlabserver.exe"';
output_directory = "ply";

filenamelist = glob.glob("*.obj");
filenamelist.sort();

for filename in filenamelist:
	newfilename = filename.replace(".obj",".ply");
	os.system( command + " -i " + filename + " -o " + output_directory + "/" + newfilename );
#	print newfilename;
