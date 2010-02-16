#!/usr/bin/env python

import os, optparse, glob, subprocess, sys, string, time;
from datetime import datetime, date, time, timedelta;

job_pool = [];

class Usage(Exception):
    def __init__(self,msg):
        self.msg = msg

def main():
    try:
        try:
            usage = "angle_detect_del [--help] apollo.cub-files "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(modify=False)
            parser.set_defaults(write_log=False)
            parser.add_option("-m", "--modify", action="store_true",
                              dest="modify",
                              help="Will write in correct time.")
            parser.add_option("-l", "--log", action="store_true",
                              dest="write_log",
                              help="Will write a log of findings.")

            (options, args) = parser.parse_args()

            if not args: parser.error("need .CUB files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        log = 0;
        if  options.write_log:
            log = open("angle_detect_del.log", 'w');

        for cube in args:

            cmd = "caminfo from="+cube+" to=temp.txt";
            os.system(cmd);

            caminfo = open("temp.txt",'r');
            cmd_return = caminfo.readlines();

            angle = "";
            for line in cmd_return:
                if line.find("EmissionAngle") >= 0:
                    angle = line.split()[2];

            if float(angle) >= 5:
                print cube+" - Emission Angle too large";
                if options.write_log:
                    log.write(cube+", E="+angle+" Too Large!\n");
                if options.modify:
                    os.system("rm "+cube);
            else:
                print cube+" - Good";
                if options.write_log:
                    log.write(cube+", E="+angle+"\n");

        if options.write_log:
            log.close();

        print "Finished"
        return 0

    except Usage, err:
        print >>sys.stderr, err.msg
        return 2

    except Exception, err:
        sys.stderr.write( str(err) + '\n' )
        return 1

if __name__ == "__main__":
    sys.exit(main())
