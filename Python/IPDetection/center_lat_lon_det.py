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
            usage = "center_lat_lon_det [--help] apollo.cub-files "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(write_log=False)
            parser.add_option("-l", "--log", action="store_true",
                              dest="write_log",
                              help="Will write a log of findings.")

            (options, args) = parser.parse_args()

            if not args: parser.error("need .CUB files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        log = 0;
        if  options.write_log:
            log = open("center_lat_lon_det.log", 'w');

        for cube in args:

            cmd = "caminfo from="+cube+" to=temp.txt";
            os.system(cmd);

            caminfo = open("temp.txt",'r');
            cmd_return = caminfo.readlines();

            ctr_lat = "";
            ctr_lon = "";
            for line in cmd_return:
                if line.find("CenterLatitude") >= 0:
                    ctr_lat = line.split()[2];
                if line.find("CenterLongitude") >= 0:
                    ctr_lon = line.split()[2];

            print cube+" - "+ctr_lat+" "+ctr_lon
            if options.write_log:
                log.write(cube+", "+ctr_lat+", "+ctr_lon+"\n");

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
