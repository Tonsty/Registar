#!/usr/bin/python

import glob;
import os;

command = "/home/yiztang/Downloads/software/MeshLab/MeshLabSrc_AllInc_v133/meshlab/src/distrib/meshlabserver";
script_file = "/home/yiztang/Registar-master/Scripts/happy_remove_edge_too_long_face.mlx";
options = "vn";

output_directory = "ml_remove_edge_too_long_face";

filenamelist = glob.glob("*.ply");
filenamelist.sort();

for filename in filenamelist:
#	newfilename = filename.replace(".ply","_ml_remove_edge_too_long_face.ply");
	os.system( command + " -i " + filename + " -o " + output_directory + "/" + filename + " -s " + script_file + " -om " + options );
#	print newfilename;