#!/usr/bin/env python

import os, glob, subprocess, sys, time
from multiprocessing import Pool

def job_func(cmd):
    print "Running: "+cmd+"\n";
    os.system(cmd);
    return cmd;

if __name__ == '__main__':
    if ( len(sys.argv) < 3 ):
        print "ERROR:"
        print sys.argv[0] + " requires at least two inputs. First is the working directory. Everything else is the number of nodes avialable for processing. You the user should provide this by `cat $PBS_NODEFILE`\n";
        sys.exit();

    # Discover my node number
    my_node_number = 0;
    name_proc = subprocess.Popen("hostname",shell=True,stdout=subprocess.PIPE);
    my_node_name = name_proc.stdout.readline().strip();

    print "My node name is: "+my_node_name;

    for i in range(2,len(sys.argv)):
        if ( sys.argv[i] == my_node_name ):
            my_node_number = i-2;

    print "My node number is: "+str(my_node_number);

    # Allow time for the jobs to start on all nodes
    time.sleep(1);

    task_number = my_node_number;
    number_of_tasks = len(sys.argv)-2;
    print "Assigned task "+str(task_number)+"/"+str(number_of_tasks)
    name_proc = subprocess.Popen("which stereo",shell=True,stdout=subprocess.PIPE);
    print "My stereo is: "+name_proc.stdout.readline().strip();

    os.chdir(sys.argv[1]);

    # Building Stereo/DEM/Hillshade/Color Commands
    stereo_cmds = []
    point2dem_cmds = []
    hillshade_cmds = []
    colormap_cmds = [];
    files = glob.glob("*.cub");
    files.sort();

    # Determing task range
    min_index = int((len(files)-1)*task_number/number_of_tasks);
    max_index = int((len(files)-1)*(task_number+1)/number_of_tasks);

    for i in range(min_index,max_index):
        startidx = files[i].find("-M-");
        endidx = files[i].find(".");
        diff = int(files[i+1][startidx+3:endidx])-int(files[i][startidx+3:endidx]);
        if ( diff > 1 ):
            continue;

        cam_indx1 = files[i].find(".") # Isis_adjust is not as smart
        cam_indx2 = files[i+1].find(".")
        cam1 = files[i][:cam_indx1] + ".isis_adjust"
        cam2 = files[i+1]:cam_indx2] + ".isis_adjust"
        id1 = (files[i].split("-M-")[1]).split(".")[0]
        id2 = (files[i+1].split("-M-")[1]).split('.')[0]
        output_dir = id1 + "_" + id2 + "-stereo"
        output_prefix = id1 + "_" + id2
        full_prefix = output_dir + "/" + output_prefix

        # Double checking output file exists
        if (not(os.path.exists(output_dir))):
            print "Creating "+output_dir
            os.system("mkdir "+output_dir)

        # Double checking that stereo was not perform correctly before
        if (os.path.exists(full_prefix+"-completed.txt")):
            print "Found finished stereo - skipping"
        else:
            # Building stereo command
            stereo_cmd = "stereo "+files[i]+" "+files[i+1]+" "+cam1+" "+cam2+" "+full_prefix+" -e 4 && touch "+full_prefix+"-completed.txt";
            stereo_cmds.append(stereo_cmd);

        # Building point2dem command
        if (os.path.exists(full_prefix+"-DEM.tif")):
            print "Found finished dem - skipping"
        else:
            point2dem_cmd = "point2dem " + full_prefix + "-PC.tif --xyz -r moon --default-value -10000";
            point2dem_cmds.append(point2dem_cmd);

        # Building hillshade command
        if (os.path.exists(full_prefix+"-DEM_HILLSHADE.tif")):
            print "Found finished hillshade - skipping"
        else:
            hillshade_cmd = "hillshade " + full_prefix + "-DEM.tif --nodata-value -10000";
            hillshade_cmds.append(hillshade_cmd);

        # Building colormap command
        if (os.path.exists(full_prefix+"-DEM_CMAP.tif")):
            print "Found finished colormap - skipping"
        else:
            colormap_cmd = "colormap " + full_prefix + "-DEM.tif --nodata-value -10000 -s " + full_prefix + "-DEM_HILLSHADE.tif --min -8499 --max 10208"
            colormap_cmds.append(colormap_cmd);

    # Creating work pool and processing
    pool = Pool(processes=8)

    print "--- Node "+str(my_node_number)+" processing STEREO ---\n";
    results = [pool.apply_async(job_func, (cmd,)) for cmd in stereo_cmds]
    for result in results:
        result.get() # No timeout

    print "--- Node "+str(my_node_number)+" processing POINT2DEM ---\n";
    results = [pool.apply_async(job_func, (cmd,)) for cmd in point2dem_cmds]
    for result in results:
        result.get() # No timeout

    print "--- Node "+str(my_node_number)+" processing HILLSHADE ---\n";
    results = [pool.apply_async(job_func, (cmd,)) for cmd in hillshade_cmds]
    for result in results:
        result.get() # No timeout

    print "--- Node "+str(my_node_number)+" processing COLORMAP ---\n";
    results = [pool.apply_async(job_func, (cmd,)) for cmd in colormap_cmds]
    for result in results:
        result.get() # No timeout

    print "--- Node "+str(my_node_number)+" finished\n"
