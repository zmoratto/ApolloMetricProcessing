#!/usr/bin/env python

import os, optparse, glob, subprocess, sys, string, csv;

job_pool = [];
output_name = "footprint.kml";

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def add_job( cmd, num_working_threads=4):
    if ( len(job_pool) >= num_working_threads):
        job_pool[0].wait();
        job_pool.pop(0);
    print cmd;
    job_pool.append( subprocess.Popen(cmd, shell=True) );

def wait_on_jobs():
    while len(job_pool) > 0:
        job_pool[0].wait();
        job_pool.pop(0);

def main():
    try:
        try:
            usage = "usage: produce_footprints.py [--help] apollo.cub-files "
            parser = optparse.OptionParser(usage=usage);
            parser.set_defaults(threads=8);
            parser.add_option("-t","--threads",dest="threads",
                              help="Number of threads to use.")

            (options, args) = parser.parse_args()

            if not args: parser.error("need .tif files")

        except optparse.OptionError, msg:
            raise Usage(msg)

        print "Using "+str(options.threads)+" threads";

        # Attaching footprint data

        for image in args:
            cmd = "footprintinit from="+image+" sinc=200 linc=200";
            add_job( cmd, int(options.threads) );
        wait_on_jobs();

        # Extracting into GML
        for image in args:
            output = image.split(".")[0]+".gml";
            cmd = "isis2gml from="+image+" to="+output+" label=Polygon";
            add_job( cmd, int(options.threads) );
        wait_on_jobs();

        # Converting to KML
        for image in args:
            kml = image.split(".")[0]+".kml";
            gml = image.split(".")[0]+".gml";
            cmd = "ogr2ogr -f \"KML\" "+kml+" "+gml;
            add_job( cmd, int(options.threads) );
        wait_on_jobs();

        # Cleaning up GML
        os.system("rm *.gml");
        os.system("rm *.gfs");

        # Create intro to output KML
        output_f = open(output_name, 'w');
        output_f.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        output_f.write("<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\" hint=\"target=moon\">\n");
        output_f.write("<Document>\n");
        output_f.write("\t<name>footprint.kml</name>\n");
        output_f.write("\t<Style id=\"sn_ylw-pushpin\">\n");
        output_f.write("\t\t<LineStyle>\n");
        output_f.write("\t\t\t<width>2</width>\n");
        output_f.write("\t\t</LineStyle>\n");
        output_f.write("\t\t<PolyStyle>\n");
        output_f.write("\t\t\t<fill>0</fill>\n");
        output_f.write("\t\t</PolyStyle>\n");
        output_f.write("\t</Style>\n");
        output_f.write("\t<Style id=\"sh_ylw-pushpin\">\n");
        output_f.write("\t\t<IconStyle>\n");
        output_f.write("\t\t\t<scale>1.2</scale>\n");
        output_f.write("\t\t</IconStyle>\n");
        output_f.write("\t\t<LineStyle>\n");
        output_f.write("\t\t\t<width>2</width>\n");
        output_f.write("\t\t</LineStyle>\n");
        output_f.write("\t\t<PolyStyle>\n");
        output_f.write("\t\t\t<fill>0</fill>\n");
        output_f.write("\t\t</PolyStyle>\n");
        output_f.write("\t</Style>\n");
        output_f.write("\t<StyleMap id=\"msn_ylw-pushpin\">\n");
        output_f.write("\t\t<Pair>\n");
        output_f.write("\t\t\t<key>normal</key>\n");
        output_f.write("\t\t\t<styleUrl>#sn_ylw-pushpin</styleUrl>\n");
        output_f.write("\t\t</Pair>\n");
        output_f.write("\t\t<Pair>\n");
        output_f.write("\t\t\t<key>highlight</key>\n");
        output_f.write("\t\t\t<styleUrl>#sh_ylw-pushpin</styleUrl>\n");
        output_f.write("\t\t</Pair>\n");
        output_f.write("\t</StyleMap>\n");

        # Open KMLs and reduce
        for image in args:
            kml = image.split(".")[0]+".kml";
            name = image.split(".")[0];
            t = image.rfind("/");
            name = name[t+1:];

            print "Adding: "+name;

            input_f = open(kml,'r');

            data = input_f.readline();
            while ( data.find("<coordinates>") < 1 ):
                data = input_f.readline();
            data = data.split("<coordinates>")[1];
            data = data.split("</coordinates>")[0];
            latlons = data.split(" ");

            # Intro for coordinate
            output_f.write("\t<Placemark>\n");
            output_f.write("\t\t<name>"+name+"</name>\n");
            output_f.write("\t\t<styleUrl>#msn_ylw-pushpin</styleUrl>\n");
            output_f.write("\t\t<ExtendedData>\n");
            output_f.write("\t\t\t<SchemaData schemaUrl=\"#multi_polygon\">\n");
            output_f.write("\t\t\t\t<SimpleData name=\"ID\">Polygon</SimpleData>\n");
            output_f.write("\t\t\t</SchemaData>\n");
            output_f.write("\t\t</ExtendedData>\n");
            output_f.write("\t\t\t<MultiGeometry><Polygon><outerBoundaryIs><LinearRing><coordinates>\n");

            # Write coordinates
            for i in range(0, len(latlons), len(latlons)/64):
                output_f.write(latlons[i]+" ");

            #Writing outro for coordinates
            output_f.write("</coordinates>\n");
            output_f.write("</LinearRing></outerBoundaryIs></Polygon></MultiGeometry></Placemark>\n");


        # Cleanup of small kml
        for image in args:
            kml = image.split(".")[0]+".kml";
            os.system("rm "+kml);

        # Close out in output KML
        output_f.write("</Document>");
        output_f.write("</kml>");

        print "Finished"
        return 0

    except Usage, err:
        print >>sys.stderr, err.msg
        return 2

    except Exception, err:
        sys.stderr.write( str(err)+'\n')
        return 1

if __name__ == "__main__":
    sys.exit(main());
