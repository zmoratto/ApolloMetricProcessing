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
            usage = "time_check_modify [--help] apollo.cub-files "
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
            log = open("time_check_modify.log", 'w');

        for cube in args:

            # Extract time from Code::Time
            cmd = "catlab from="+cube;
            p = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE);
            cmd_return = p.stdout.readlines();
            group = "";
            time_code = "";
            time_instrument = "";
            for line in cmd_return:
                if line.find("Group = ") >= 0:
                    group = line.split()[2];
                if line.find(" StartTime ") >= 0:
                    if ( group == "Code" ):
                        time_code = line.split()[2];
                    if ( group == "Instrument" ):
                        time_instrument = line.split()[2];
            time_instrument = time_instrument[:23];
            time_code = time_code[:23];

            if time_instrument == "" or time_code == "":
                print cube+" has no timestamps!";
                if options.write_log:
                    log.write(cube+", NO TIMESTAMPS IN INSTRUMENT OR CODE GROUPS\n");
                continue;

            # Clean up inability to stay withn reasonable values;
            if ( int(time_code[11:13]) > 24 ):
                print "\tBefore: "+time_code;
                diff = int(time_code[11:13]) - 24;
                print "\t\tDiff: "+str(diff)
                print "\t\tDate Corr:"+str(int(time_code[8:10])+1)
                day_correction = str(int(time_code[8:10])+1);
                if len(day_correction) == 1:
                    day_correction = "0"+day_correction
                time_code = time_code[:8]+day_correction+time_code[10:];
                if (len(str(diff)) == 2):
                    time_code = time_code[:11]+str(diff)+time_code[13:];
                else:
                    time_code = time_code[:11]+"0"+str(diff)+time_code[13:];
                print "\tChanged to: "+time_code;
            if ( int(time_code[14:16]) > 60 ):
                diff = int(time_code[14:16])-60;
                new_hour = str(int(time_code[11:13])+1);
                if (len(new_hour) ==2):
                    time_code = time_code[:11]+new_hour+time_code[13:];
                else:
                    time_code = time_code[:11]+"0"+new_hour+time_code[13:];
                if (len(str(diff)) == 2 ):
                    time_code = time_code[:14]+str(diff)+time_code[16:];
                else:
                    time_code = time_code[:14]+"0"+str(diff)+time_code[16:];
                print "\tChanged to: "+time_code;
            if ( int(time_code[17:19]) > 60 ):
                diff = str(int(time_code[17:19]) - 60);
                new_minute = str(int(time_code[14:16])+1);
                if (len(new_minute) == 2):
                    time_code = time_code[:14]+new_minute+time_code[16:];
                else:
                    time_code = time_code[:14]+"0"+new_minute+time_code[16:];
                if (len(diff) == 2):
                    time_code = time_code[:17]+diff+time_code[19:];
                else:
                    time_code = time_code[:17]+"0"+diff+time_code[19:];
                print "\tChanged to: "+time_code;

            if ( len(time_instrument) == 22 ):
                time_instrument = time_instrument+"0";
            if ( len(time_instrument) == 21 ):
                time_instrument = time_instrument+"00";
            if ( len(time_code) == 22 ):
                time_code = time_code+"0";
            if ( len(time_code) == 21 ):
                time_code = time_code+"00";

            if ( len(time_instrument) < 23 and len(time_code) == 23 ):
                print cube+" only has code timestamp!";
                print "Length: "+str(len(time_code))
                t_code = datetime.strptime(time_code+"000","%Y-%m-%dT%H:%M:%S.%f");
                if options.write_log:
                    log.write(cube+", ONLY HAS CODE GROUP\n");
                if options.modify:
                    correction = t_code.strftime("%Y-%m-%dT%H:%M:%S.%f");
                    correction = correction[:23];
                    print " -> Correcting to: "+correction;
                    cmd = "editlab from="+cube+" option=modkey grpname=Instrument keyword=StartTime value="+correction;
                    print cmd;
                    os.system(cmd);
                continue;
            if ( len(time_instrument) == 23 and len(time_code) < 23 ):
                print cube+" has no code group";
                if options.write_log:
                    log.write(cube+", NO CODE GROUP\n");
                continue;

            t_instrument = datetime.strptime(time_instrument+"000","%Y-%m-%dT%H:%M:%S.%f");
            t_code = datetime.strptime(time_code+"000","%Y-%m-%dT%H:%M:%S.%f");


            if options.write_log:
                log.write(cube+", GOOD: DIFF="+str((t_code-t_instrument))+"\n");

            if (not options.modify):
                print cube;
                print "   Instrument Time: "+str(t_instrument);
                print "         Code Time: "+str(t_code);
                print "              DIFF: "+str((t_code-t_instrument));
                
            if ( abs(t_code-t_instrument) > timedelta(minutes=30) ):
                print cube+" has issues!";
                if options.modify:
                    correction = t_code.strftime("%Y-%m-%dT%H:%M:%S.%f");
                    correction = correction[:23];
                    print " -> Correcting to: "+correction;
                    cmd = "editlab from="+cube+" option=modkey grpname=Instrument keyword=StartTime value="+correction;
                    print cmd;
                    os.system(cmd);

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
