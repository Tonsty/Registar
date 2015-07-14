#!/usr/bin/python

import os;

command = "..\\HMRR.exe";

#output file

output_info_file = "bunny_ouput_info";

#input file

scans_file = "./bunny.scans";
# .scans file
# 0.ply
# 1.ply
# ...
links_file = "./bunny.links";
# .links file
# 0 6
# 0 9
# ...
loops_file = "./bunny.loops";
# .loops file
# 0 1 5
# 0 1 9
# 0 5 6
# ...

#parameter settings

#--ipr : initial pairwise registration 
#--il  : incremental loop refine
#--gr  : global refine or global pair refine
algorithm_options = "--ipr --il";
#algorithm_options = "--ipr --il"; #only incremental loop refine
#algorithm_options = "--ipr --gr"; #only global pair refine
#algorithm_options = "--gr"; #only global refine

#number of iterations of pairwise registration
pi_max     = 100;
pi_min     = 80;
#threshold distance in pairwise registration
distance   = 0.01;
#threshold angle in pairwise registration
angle      = 60;

#number of iterations of pairwise refine in incremental loop refine
pi_num     = 30;

#mumber of iterations in global refine
gi_max     = 60;
gi_min     = 30;

pr_options = "--pi_max " + str(pi_max) + " --pi_min " + str(pi_min) + " --distance " + str(distance) + " --angle " + str(angle);
il_options = "--pi_num " + str(pi_num);
gr_options = "--gi_max " + str(gi_max) + " --gi_min " + str(gi_min);

input_files= scans_file + " " + links_file + " " + loops_file + " ";
options    = algorithm_options + " " + pr_options + " " + il_options + " " + gr_options;

command_total = command + " " +  input_files + " " + options + " > " + output_info_file;
print(command_total);

output_info_file_object = open(output_info_file,"w");
output_info_file_object.write(command_total + "\n\n");
output_info_file_object.close();

os.system(command + " " +  input_files + " " + options + " >> " + output_info_file);
