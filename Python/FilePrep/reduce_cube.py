#!/usr/bin/env python

import os, optparse, glob, subprocess, sys, glob;
from multiprocessing import Pool;

def job_func(cmd):
    print "\n\n-> Running: " + cmd + "\n\n";
    os.system(cmd);
    return cmd;

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main():

    usage = "usage: reduce_script_v2.py apollo_cubes "
    parser = optparse.OptionParser(usage=usage)
    parser.set_defaults(threads=2);
    parser.add_option("-t","--threads",dest="threads",
                      help="Number of threads to use.")

    (options, args) = parser.parse_args()

    scale_cmds = [];
    stretch_cmds = [];

    for cube in args:
        # Scaling
        idx = cube.rfind("/");
        prefix = cube[idx+1:];
        prefix = prefix.split(".")[0];
        if ( os.path.exists(prefix+".lev0.cub") ):
            continue;
        cmd = "reduce from="+cube+" to="+prefix+".lev0.cub lscale=4 sscale=4";
        print cmd
        scale_cmds.append(cmd);

    pool = Pool(processes=int(options.threads));
    results = [pool.apply_async(job_func, (cmd,)) for cmd in scale_cmds]
    for result in results:
        result.get(timeout=3600)

    for cube in args:
        # Stretch
        idx = cube.rfind("/");
        prefix = cube[idx+1:];
        prefix = prefix.split(".")[0];

        if ( os.path.exists(prefix+".lev1.cub") ):
            continue;

        # Getting Low
        cmd = "percent from="+prefix+".lev0.cub p=0.1";
        p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
        cmd_return = p.stdout.readlines();
        low_value = "";
        for line in cmd_return:
            if line.find("Value") >= 0:
                low_value = line.split()[2];

        cmd = "percent from="+prefix+".lev0.cub p=99.5";
        p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
        cmd_return = p.stdout.readlines();
        hi_value = "";
        for line in cmd_return:
            if line.find("Value") >= 0:
                hi_value = line.split()[2];

        print "Low Value: "+low_value;
        print "Hi Value:  "+hi_value;

        cmd = "stretch from="+prefix+".lev0.cub to="+prefix+".lev1.cub+16bit+0:65535 p=\""+low_value+":1 "+hi_value+":65534\" null=0 lis=0 lrs=0 his=65535 hrs=65535";
        stretch_cmds.append(cmd);

    results = [pool.apply_async(job_func, (cmd,)) for cmd in stretch_cmds]
    for result in results:
        result.get(timeout=3600)

    print "Finished:";
    return 0;

if __name__ == "__main__":
    sys.exit(main());
