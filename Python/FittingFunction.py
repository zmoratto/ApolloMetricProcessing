#!/opt/local/bin/python2.6

# This is just a bunch of tools for solving for fitting transforms

from numpy import *
from InterestIO import write_match_file
import os, subprocess

def solve_euclidean(meas1, meas2):
    mean1 = array(meas1).sum(0) / 2.0
    mean2 = array(meas2).sum(0) / 2.0

    H = array([meas1[0]-mean1]).transpose()*(meas2[0]-mean2)
    H = H + array([meas1[1]-mean1]).transpose()*(meas2[1]-mean2)

    U, S, Vt = linalg.svd(H)
    rotation = dot(transpose(Vt),transpose(U))
    translation = mean2-dot(rotation,mean1)
    output = identity(3,float)
    output[0:2,0:2] = rotation
    output[0:2,2] = translation
    return output

def solve_affine(meas1, meas2):
    y = zeros((len(meas1)*2,1),float)
    A = zeros((len(meas1)*2,6),float)

    for i in range(0,len(meas1)):
        for j in range(0,2):
            row = i*2+j
            A[row,0+j*3:2+j*3] = meas1[i]
            A[row,2+j*3] = 1
            y[row] = meas2[i][j]

    x = linalg.lstsq(A,y)[0]

    solution = identity(3,float)
    solution[0,0:3] = x[0:3,0]
    solution[1,0:3] = x[3:6,0]
    return solution

def solve_homography(meas1, meas2):
    write_match_file("tmp.match",meas1,meas2)
    p = subprocess.Popen("homography_fit tmp.match",
                         shell=True,stdout=subprocess.PIPE);
    cmd_return = p.stdout.readline().strip()
    text = cmd_return[cmd_return.find(":")+13:].strip("((").strip("))").replace(")(",",").split(",")
    solution = identity(3,float)
    solution[0,0:3] = [float(text[0]), float(text[1]), float(text[2])]
    solution[1,0:3] = [float(text[3]), float(text[4]), float(text[5])]
    solution[2,0:3] = [float(text[6]), float(text[7]), float(text[8])]
    return solution

