#!/usr/bin/python

import glob;
import os;

command = r'"C:\Program Files\VCG\MeshLab\meshlabserver.exe"';
script_file = r"E:\Registar-master\Scripts\armadillo_remove_edge_too_long_face.mlx";
options = "vn vc";

output_directory = "ml_remove_edge_too_long_face";

filenamelist = glob.glob("*.ply");
filenamelist.sort();

for filename in filenamelist:
#	newfilename = filename.replace(".ply","_ml_remove_edge_too_long_face.ply");
	os.system( command + " -i " + filename + " -o " + output_directory + "/" + filename + " -s " + script_file + " -om " + options );
#	print newfilename;