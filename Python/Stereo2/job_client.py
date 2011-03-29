#!/usr/bin/env python

import os, glob, subprocess, sys, time, zmq, signal
from multiprocessing import Pool

socket = []
index = 0

def signal_handler(signal, frame):
    global socket, index
    print "Exit requested"
    socket.send("Failed,"+str(index))
    message = socket.recv()
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    url = sys.argv[1]
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect( url )

    # Test connection with server
    socket.send("PING")
    message = socket.recv()
    if ( message != "What?" ):
        print "Failed to establish contact"
        exit()

    # Change to current working directory
    socket.send("PWD")
    message = socket.recv()
    print "Working Directory: " + message
    os.chdir(message)

    # Start performing work
    while True:
        socket.send("NJ")
        message = socket.recv()
        if ( message == "FN" ):
            print "No Jobs Available"
            break
        comma = message.find(",")
        cmd = message[comma+1:]
        print cmd

        os.system(cmd)

        index = int(message[:comma])
        socket.send("FN,"+str(index))
        message = socket.recv()

    print "job_client.py clean exit"
