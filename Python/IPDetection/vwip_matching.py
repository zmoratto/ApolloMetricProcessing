#!/usr/bin/env python

import os, glob, subprocess, sys, time;
from multiprocessing import Pool

def job_func(cmd):
    os.system(cmd);
    return cmd;

if __name__ == '__main__':

    # Handling command line input
    if (len(sys.argv) < 2 ):
        print "ERROR:"
        print sys.argv[0] + " requires at least two inputs. First is the working directory. Everything else is the number of nodes avialable for processing. You the user should provide this by `cat $PBS_NODEFILE`\n";
        sys.exit();

    my_node_number = 0;
    name_proc = subprocess.Popen("hostname",shell=True,stdout=subprocess.PIPE);
    my_node_name = name_proc.stdout.readline().strip();

    print "My node name is: "+my_node_name;

    for i in range(2,len(sys.argv)):
        if ( sys.argv[i] == my_node_name ):
            my_node_number = i-2;

    print "My node number is: "+str(my_node_number);

    task_number = my_node_number;
    number_of_tasks = len(sys.argv)-2;

    os.chdir(sys.argv[1]);

    # Determing number of files
    files = glob.glob("*.cub");
    files = sorted(files);
    max_matches = 100

    # Building all permutations
    pool = Pool(processes=8)

    for i in range(0, len(files)):
        match_cmd = []
        if ( os.path.exists("fin_iteration_"+str(i)+".txt" ) ):
            continue
        for j in range(i+1,len(files)):

            # Checking if output already exists
            idx = files[i].rfind(".")
            match_file = files[i][:idx] + "__"+files[j][:idx]+".match"

            if not os.path.exists(match_file):
        
                # Making Command
                cmd = "/u/zmoratto/projects/Sandbox/OverlapCheckMatch/overlap_check_match "+files[i]+" "+files[j]+" -m "+str(max_matches)
                match_cmd.append(cmd)

        results = [pool.apply_async(job_func, (cmd,)) for cmd in match_cmd]
        for result in results:
            result.get() # No timeout
        os.system( "touch fin_iteration_"+str(i)+".txt" )

    #min_index = int(len(match_cmd)*task_number/number_of_tasks);
    #max_index = int(len(match_cmd)*(task_number+1)/number_of_tasks);

    #print "Min index: "+str(min_index)
    #print "Max index: "+str(max_index)
    print "Finished";
    
