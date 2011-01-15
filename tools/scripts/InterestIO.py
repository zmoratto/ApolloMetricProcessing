#!/opt/local/bin/python2.6

# This is just tools for reading and writing VW match files

from numpy import array
from struct import *
from copy import copy

class ip:
    def __init__(self, array_v, scale_v):
        self.location = array_v
        self.scale = scale_v
    def __add__(self,other):
        return ip(self.location+other,self.scale)
    def __iadd__(self,other):
        self.location = self.location + other
    def __sub__(self,other):
        return ip(self.location-other,self.scale)
    def __isub__(self,other):
        self.location = self.location - other
    def __mul__(self,other):
        return ip(self.location*other,self.scale*other)
    def __imul__(self,other):
        self.location = self.location * other
        self.scale = self.scale * other
    def __div__(self,other):
        return ip(self.location/other,self.scale/other)
    def __idiv__(self,other):
        self.location = self.location / other
        self.scale = self.scale / other
    def __getitem__(self,key):
        return self.location[key]
    def __setitem__(self,key,value):
        self.location[key] = value
    def __str__(self):
        return "Location: "+str(self.location)+"\nScale: "+str(self.scale)

def read_ip( file ):
    # x = 4f,   y = 4f, ix = 4i, iy = 4i
    # ori = 4f, s = 4f, in = 4f, bool = 1
    # oc = 4u, sc = 4u, size = 8u, float array
    ip_front_raw = file.read(29)
    ip_back_raw  = file.read(16)
    ip_front = unpack('ffiifff?',ip_front_raw)
    ip_back  = unpack('IIQ',ip_back_raw)
    file.read(ip_back[2]*4)
    return ip(array([ip_front[0], ip_front[1]]), ip_front[5])

def write_ip( file, ip ):
    ip_front_raw = pack('ffiifff?', ip[0], ip[1], int(ip[0]), int(ip[1]),
                        0, ip.scale, 0, 1 )
    ip_back_raw = pack('IIQ',0,0,0)
    file.write( ip_front_raw )
    file.write( ip_back_raw )

def read_match_file( filename ):
    ip1 = []
    ip2 = []
    print "Reading match file: ", filename
    file = open(filename,"rb")
    ip1_size = unpack('Q',file.read(8))[0]
    ip2_size = unpack('Q',file.read(8))[0]
    try:
        for i in range(0,ip1_size):
            ip1.append(read_ip(file))
        for i in range(0,ip2_size):
            ip2.append(read_ip(file))
    finally:
        file.close()
    return ip1, ip2

def write_match_file( filename, ip1, ip2 ):
    print "Writing match file: ", filename
    file = open(filename,"wb")
    file.write( pack('Q',len(ip1)) )
    file.write( pack('Q',len(ip2)) )
    for ip in ip1:
        write_ip( file, ip )
    for ip in ip2:
        write_ip( file, ip )
    file.close()

