#!/usr/bin/env python

import os,  sys, math;
from math import cos, sin;

def angle_diff( lat_s, lat_f, lon_s, lon_f ):
    lon_diff = lon_f-lon_s;
    sin_lon_diff = sin(lon_diff);
    cos_lon_diff = cos(lon_diff);

    cos_lat_f = cos(lat_f)
    sin_lat_f = sin(lat_f)
    cos_lat_s = cos(lat_s)
    sin_lat_s = sin(lat_s)
            
    top_left = cos_lat_f*sin_lon_diff;
    top_right = cos_lat_s*sin_lat_f-sin_lat_s*cos_lat_f*cos_lon_diff
    top = math.sqrt(top_left*top_left+top_right*top_right)
    bot = sin_lat_s*sin_lat_f+cos_lat_s*cos_lat_f*cos_lon_diff;

    adiff = math.atan2(top,bot)
    return adiff

def main():
    file_names = [];
    lats = [];
    lons = [];

    f = open("center_lat_lon_det.log",'r')
    for line in f:
        seg = line.split(",");
        file_names.append(seg[0]);
        lats.append(float(seg[1]));
        lons.append(float(seg[2]));
    f.close();

    out = open("image_Match.pair",'w')

    for i in range(0,len(file_names)):
        for j in range(i+1,len(file_names)):
            lat_s = math.radians( lats[i] );
            lat_f = math.radians( lats[j] );
            lon_s = math.radians( lons[i] );
            lon_f = math.radians( lons[j] );
            
            angle = math.degrees(angle_diff(lat_s, lat_f, lon_s, lon_f));
            if ( angle < 6 ):
                out.write(file_names[i]+", "+file_names[j]+"\n");

    out.close();

if __name__ == "__main__":
    sys.exit(main())
