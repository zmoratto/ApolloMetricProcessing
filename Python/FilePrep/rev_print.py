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
            usage = "rev_print [--help] apollo.cub-files "
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
            log = open("rev_print.log", 'w');

        for cube in args:

            # Extract REV file
            cmd = "catlab from="+cube;
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readlines();
            rev = "";
            for line in cmd_return:
                if line.find("_M_REV") >= 0:
                    num = line.split("_M_REV")[1];
                    idx = num.rfind(".");
                    rev = num[:idx];
                    continue;
            print cube+" - REV "+rev;

            if options.write_log:
                log.write(cube+", REV "+rev+"\n");

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
