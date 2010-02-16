#!/usr/bin/env python

import os, optparse, glob, subprocess, sys, string, csv;

# Global Variables
job_pool = [];
job_pool_file = [];
g_write_log = False;
g_modify = False;
log = 0;

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def add_spice_job( cmd, job_file, num_working_threads=4):
    if ( len(job_pool) >= num_working_threads ):
        job_pool[0].wait();
        cmd_return = job_pool[0].stdout.readline();
        cmd_err_return = job_pool[0].stderr.readline();
        if cmd_return.find("Kernels") < 0 or cmd_err_return.find("**") >= 0:
            print "Spice failed on: "+job_pool_file[0];
            if g_write_log:
                log.write(job_pool_file[0]+", SPICE FAILED\n");
            if g_modify:
                os.system("rm "+job_pool_file[0]);
        else:
            if g_write_log:
                log.write(job_pool_file[0]+", SPICE SUCCESS\n");
        job_pool.pop(0);
        job_pool_file.pop(0);
    print cmd;
    job_pool.append( subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE) );
    job_pool_file.append(job_file);

def wait_on_spice_jobs():
    while len(job_pool) > 0:
        job_pool[0].wait();
        cmd_return = job_pool[0].stdout.readline();
        cmd_err_return = job_pool[0].stderr.readline();
        if cmd_return.find("Kernels") < 0 or cmd_err_return.find("**") >= 0:
            print "Spice failed on: "+job_pool_file[0];
            if g_write_log:
                log.write(job_pool_file[0]+", SPICE FAILED\n");
            if g_modify:
                os.system("rm "+job_pool_file[0]);
        else:
            if g_write_log:
                log.write(job_pool_file[0]+", SPICE SUCCESS\n");
        job_pool.pop(0);
        job_pool_file.pop(0);

def main():
    try:
        try:
            usage = "usage: spice_att_del.py [--help][--threads] apollo.cub-files "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(modify=False)
            parser.set_defaults(write_log=False)
            parser.set_defaults(threads=4);
            parser.add_option("-m","--modify", action="store_true",
                              dest="modify", help="Will delete if spice does not attach")
            parser.add_option("-l","--log", action="store_true",
                              dest="write_log", help="Will write a captain's log")
            parser.add_option("-t","--threads",dest="threads",
                              help="Number of threads to use.")

            (options, args) = parser.parse_args()

            if not args: parser.error("need .cub files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        print "Number of threads to use: "+str(options.threads);

        if options.modify:
            print "Modify is ON";
            global g_modify
            g_modify = True;

        if options.write_log:
            print "Writing a Log.";
            global g_write_log
            g_write_log = True
            global log
            log = open("spice_att_del.log", 'w');

        for cube in args:
            cmd = "spiceinit from="+cube;
            add_spice_job(cmd,cube,options.threads);
        wait_on_spice_jobs();

        if options.write_log:
            log.close();

    except Usage, err:
        print >>sys.stderr, err.msg
        return 2

    except Exception, err:
        sys.stderr.write( str(err)+'\n')
        return 1

if __name__ == "__main__":
    sys.exit(main());

