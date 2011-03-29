#!/usr/bin/env python

import os, glob, subprocess, sys, time, zmq
from multiprocessing import Process, Value

class ProcessingJob:
    # status = 0,1,2 {not performed, assigned, done}
    def __init__(self, cmd):
        self.cmd = cmd;
        self.status = 0;

def write_jobs_status(jobs):
    f = open('job_server.log', 'w')
    for job in jobs:
        f.write(job.cmd+"#!#"+str(job.status)+"\n")
    f.close()

def find_jobs(working_dir):
    cube_files = glob.glob(working_dir+"/*.cub")
    cube_files.sort()
    jobs = []

    for i in range(0,len(cube_files)-1):
        print '%04.2f%%\r' % (float(i)/float(len(cube_files))*100.0),
        sys.stdout.flush()
        startidx = cube_files[i].find("-M-");
        endidx = cube_files[i].find(".");
        diff = int(cube_files[i+1][startidx+3:endidx])-int(cube_files[i][startidx+3:endidx]);
        if ( diff > 1 ):
            continue;

        cam1 = cube_files[i][:cube_files[i].rfind(".")] + ".isis_adjust"
        cam2 = cube_files[i+1][:cube_files[i+1].rfind(".")] + ".isis_adjust"

        id1 = (cube_files[i].split("-M-")[1]).split(".")[0]
        id2 = (cube_files[i+1].split("-M-")[1]).split('.')[0]
        output_dir = id1 + "_" + id2 + "-stereo"
        output_prefix = id1 + "_" + id2
        full_prefix = output_dir + "/" + output_prefix

        # Double checking output dir exists
        if (not(os.path.exists(working_dir+"/"+output_dir))):
            os.system("mkdir "+working_dir+"/"+output_dir)

        # Double checking that stereo was not perform correctly before
        if (os.path.exists(working_dir+"/"+full_prefix+"-completed.txt") or
            os.path.exists(working_dir+"/"+full_prefix+"-corr-completed.txt") ):
            continue;
        else:
            stereo_pos_args = ""
            if (os.path.exists(cam1) and os.path.exists(cam2)):
                stereo_pos_args = cube_files[i]+" "+cube_files[i+1]+" "+cam1+" "+cam2
            else:
                continue
            stereo_cmd = "stereo_tri "+stereo_pos_args+" "+full_prefix+" && touch "+full_prefix+"-corr-completed.txt";
            jobs.append( ProcessingJob( stereo_cmd ) )

    print '\n\n'
    return jobs



if __name__ == '__main__':

    jobs = [];
    working_dir = "/u/zmoratto/nobackup/Moon/Apollo17_cubes"

    # Figuring out what jobs we have to do
    jobs = find_jobs( working_dir )

    read_head = 0

    # Spawn ZMQ
    print "Hello, I am your server"
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5566")

    write_jobs_status(jobs)
    while True:
        message = socket.recv()
        if ( message == "NJ" ):
            if ( read_head < len(jobs) ):
                while ( jobs[read_head].status != 0 and read_head < len(jobs) ):
                    read_head = read_head + 1
                if ( read_head >= len(jobs) ):
                    socket.send("FN")
                    continue
                socket.send( str(read_head)+","+jobs[read_head].cmd )
                jobs[read_head].status=1
                read_head = read_head+1
            else:
                socket.send("FN")
            write_jobs_status(jobs)
        elif ( message.find("FN") == 0 ):
            index = int(message[message.find(",")+1:])
            socket.send("Thanks")
            jobs[index].status=2
            write_jobs_status(jobs)
        elif ( message.find("Failed") == 0 ):
            index = int(message[message.find(",")+1:])
            socket.send("I understand")
            jobs[index].status=0
            read_head=0
            write_jobs_status(jobs)
        elif ( message == "QUIT" ):
            socket.send("Okay")
            break
        elif ( message == "STAT" ):
            completed_count = 0
            assigned_count = 0
            for job in jobs:
                if ( job.status == 2 ):
                    completed_count = completed_count + 1
                if ( job.status == 1 ):
                    assigned_count = assigned_count + 1
            socket.send("Assigned "+str(assigned_count)+"/"+str(len(jobs))+" jobs, Completed "+str(completed_count)+"/"+str(len(jobs))+" jobs")
        elif ( message == "PWD" ):
            socket.send(working_dir)
        else:
            print "Unknown message: "+message
            socket.send("What?")
