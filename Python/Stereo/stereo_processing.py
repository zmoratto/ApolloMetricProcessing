#!/usr/bin/env python

# This is meant for the Harpertown Cores. You should be spawning 4 processes that all think they can have 4 threads. Each process must be limited to 2GB of RAM.

import os, glob, subprocess, sys, time
from multiprocessing import Pool

def job_func(cmd):
    print "Running: "+cmd;
    os.system(cmd);
    return cmd;

if __name__ == '__main__':
    if ( len(sys.argv) < 3 ):
        print "ERROR:"
        print sys.argv[0] + " requires at least two inputs. First is the working directory. Everything else is the number of nodes avialable for processing. You the user should provide this by `cat $PBS_NODEFILE`\n";
        sys.exit();

    # Discover my job number
    job_number = int(sys.argv[2])
    number_of_jobs = int(sys.argv[3])

    # Discover my node number
    my_node_number = 0;
    name_proc = subprocess.Popen("hostname",shell=True,stdout=subprocess.PIPE);
    my_node_name = name_proc.stdout.readline().strip();

    for i in range(4,len(sys.argv)):
        if ( sys.argv[i] == my_node_name ):
            my_node_number = i-4;

    print my_node_name+": node number is: "+str(my_node_number);

    # Allow time for the jobs to start on all nodes
    time.sleep(1);

    task_number = my_node_number;
    number_of_tasks = len(sys.argv)-4;
    print my_node_name+": assigned task "+str(task_number)+"/"+str(number_of_tasks)+" of Job "+str(job_number)+"/"+str(number_of_jobs)
    name_proc = subprocess.Popen("which stereo",shell=True,stdout=subprocess.PIPE);
    print my_node_name+": My stereo is: "+name_proc.stdout.readline().strip();

    os.chdir(sys.argv[1]);

    # Building Stereo/DEM/Hillshade/Color Commands
    stereo_cmds = []
    point2dem_cmds = []
    hillshade_cmds = []
    colormap_cmds = []
    orthoproject_cmds = []
    files = glob.glob("*.cub");
    files.sort();

    # Determing job's rang
    jobs_min_index = int((len(files)-1)*float(job_number)/float(number_of_jobs))
    jobs_max_index  = int((len(files)-1)*float(job_number+1)/float(number_of_jobs))
    tasks_todo = jobs_max_index - jobs_min_index;

    # Determing task range
    min_index = int((tasks_todo)*task_number/number_of_tasks);
    max_index = int((tasks_todo)*(task_number+1)/number_of_tasks);

    print my_node_name+": My index range: "+str(jobs_min_index+min_index)+" - "+str(jobs_min_index+max_index)

    for i in range(jobs_min_index+min_index,jobs_min_index+max_index):
        startidx = files[i].find("-M-");
        endidx = files[i].find(".");
        diff = int(files[i+1][startidx+3:endidx])-int(files[i][startidx+3:endidx]);
        if ( diff > 1 ):
            continue;

        cam1 = files[i][:files[i].find(".")] + ".isis_adjust"
        cam2 = files[i+1][:files[i+1].find(".")] + ".isis_adjust"
        cam1_alt = files[i][:files[i].rfind(".")] + ".isis_adjust"
        cam2_alt = files[i+1][:files[i+1].rfind(".")] + ".isis_adjust"

        id1 = (files[i].split("-M-")[1]).split(".")[0]
        id2 = (files[i+1].split("-M-")[1]).split('.')[0]
        output_dir = id1 + "_" + id2 + "-stereo"
        output_prefix = id1 + "_" + id2
        full_prefix = output_dir + "/" + output_prefix

        # Double checking output file exists
        if (not(os.path.exists(output_dir))):
            os.system("mkdir "+output_dir)

        # Double checking that stereo was not perform correctly before
        if (os.path.exists(full_prefix+"-completed.txt")):
            print "Found finished stereo - skipping"
        else:
            if ( os.path.exists(cam1) and os.path.exists(cam2) ):
                # Building stereo command
                stereo_cmd = "stereo "+files[i]+" "+files[i+1]+" "+cam1+" "+cam2+" "+full_prefix+" && touch "+full_prefix+"-completed.txt";
                stereo_cmds.append(stereo_cmd);
            elif (os.path.exists(cam1_alt) and os.path.exists(cam2_alt)):
                # Building stereo command
                stereo_cmd = "stereo "+files[i]+" "+files[i+1]+" "+cam1_alt+" "+cam2_alt+" "+full_prefix+" && touch "+full_prefix+"-completed.txt";
                stereo_cmds.append(stereo_cmd);
            else:
                # Building stereo command
                stereo_cmd = "stereo "+files[i]+" "+files[i+1]+" "+full_prefix+" && touch "+full_prefix+"-completed.txt";
                stereo_cmds.append(stereo_cmd);

        # Building point2dem command
        if (os.path.exists(full_prefix+"-DEM.tif")):
            print "Found finished dem - skipping"
        else:
            point2dem_cmd = "point2dem " + full_prefix + "-PC.tif --xyz -r moon --default-value -32767 --cache-dir .";
            point2dem_cmds.append(point2dem_cmd);

        # Building hillshade command
        if (os.path.exists(full_prefix+"-DEM_HILLSHADE.tif")):
            print "Found finished hillshade - skipping"
        else:
            hillshade_cmd = "hillshade " + full_prefix + "-DEM.tif --nodata-value -32767";
            hillshade_cmds.append(hillshade_cmd);

        # Building colormap command
        if (os.path.exists(full_prefix+"-DEM_CMAP.tif")):
            print "Found finished colormap - skipping"
        else:
            colormap_cmd = "colormap " + full_prefix + "-DEM.tif --nodata-value -32767 -s " + full_prefix + "-DEM_HILLSHADE.tif --min -8499 --max 10208"
            colormap_cmds.append(colormap_cmd);

        # Building orthoproject command
        if (os.path.exists(full_prefix+"-DRG.tif")):
            print "Found finish drg - skipping"
        else:
            if ( os.path.exists(cam1) ):
                # PPD should be 4096 in release and should use original images
                orthoproject_cmd = "orthoproject " + full_prefix + "-DEM.tif " + files[i] + " " + cam1 + " " + full_prefix +  "-DRG.tif --nodata -32768 --ppd 256"
                orthoproject_cmds.append( orthoproject_cmd );
            elif ( os.path.exists(cam1_alt) ):
                # PPD should be 4096 in release and should use original images
                orthoproject_cmd = "orthoproject " + full_prefix + "-DEM.tif " + files[i] + " " + cam1_alt + " " + full_prefix +  "-DRG.tif --nodata -32768 --ppd 256"
                orthoproject_cmds.append( orthoproject_cmd );
            else:
                # PPD should be 4096 in release and should use original images
                orthoproject_cmd = "orthoproject " + full_prefix + "-DEM.tif " + files[i] + " " + full_prefix +  "-DRG.tif --nodata -32768 --ppd 256"
                orthoproject_cmds.append( orthoproject_cmd );

    # Creating work pool and processing
    pool = Pool(processes=4)

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

    #print "--- Node "+str(my_node_number)+" processing ORTHOPROJECT ---\n";
    #results = [pool.apply_async(job_func, (cmd,)) for cmd in orthoproject_cmds]
    #for result in results:
    #    result.get() # No timeout

    print "--- Node "+str(my_node_number)+" finished\n"
