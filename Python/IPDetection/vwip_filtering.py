#!/usr/bin/python

import os, glob, subprocess, sys, time;

job_pool = [];

def add_job( cmd, num_working_threads=8):
    if ( len(job_pool) >= num_working_threads):
        job_pool[0].wait();
        job_pool.pop(0);
    print cmd;
    job_pool.append( subprocess.Popen(cmd, shell=True) );

def wait_on_jobs():
    while len(job_pool) > 0:
        job_pool[0].wait();
        job_pool.pop(0);

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

# Allow time for the jobs to start on all nodes
time.sleep(2);

task_number = my_node_number;
number_of_tasks = len(sys.argv)-2;

os.chdir(sys.argv[1]);

files = glob.glob("*.cub");
files = sorted(files);

min_index = int(len(files)*task_number/number_of_tasks);
max_index = int(len(files)*(task_number+1)/number_of_tasks);

print "Min index: "+str(min_index)
print "Max index: "+str(max_index)

print "VWIP Filtering"
for i in range( min_index, max_index ):
    cube = files[i];
    idx = cube.rfind(".");
    prefix = cube[:idx];
    cmd = "/u/zmoratto/projects/Sandbox/OverlapCheckMatch/vwip_filter "+prefix+".cub "+prefix+".vwip ";
    add_job(cmd);
wait_on_jobs();

print "Finished";
    
