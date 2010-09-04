#!/usr/bin/env python

import os, glob, subprocess, sys, time, zmq
from multiprocessing import Process
from time import sleep

# Use this executable with a :
# deploy_plate.py `cat $PBS_NODEFILE`

rabbitmq_go = False
finish_count = 1

def server_process( socket ):
    while True:
        message = socket.recv()
        print "Has message: "+message
        if ( message == "Awake?" ):
            if ( rabbitmq_go ) :
                socket.send("Yes")
            else:
                socket.send("No")
        elif ( message == "Finished" ):
            finish_count = finish_count + 1
        else:
            print "Unknown message: "+message

def cmd_process( cmd ):
    print "Running "+cmd
    os.system( cmd )

if __name__ == '__main__':

    # Error check
    if ( len(sys.argv) < 2 ):
        print "ERROR:"
        print "Missing input that list other nodes available\n";
        sys.exit();

    # Discover my node number
    hostname_proc = subprocess.Popen("hostname",shell=True,stdout=subprocess.PIPE)
    hostname = hostname_proc.stdout.readline().strip()
    print "My hostname: " + hostname
    servername = sys.argv[1]
    print "My server's name: " + servername

    # Load up the number of nodes
    node_number = 0
    number_of_nodes = len(sys.argv)-1;
    for i in range(1,len(sys.argv)):
        if ( sys.argv[i] == hostname ):
            node_number = i - 1
    print "Node number: " + str(node_number)

    # Spawn ZMQ
    context = zmq.Context()
    socket = []    # Connection to outside world
    server_p = []  # Process handling server responses
    rabbit_p = []  # Process running rabbitmq-server
    index_p  = []  # Process running index_server
    if ( node_number == 0 ) :
        print "I AM SERVER "+servername
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5566")

        # Start spawning the server
        server_p = Process(target=server_process, args=(socket,))
        rabbit_p = Process(target=cmd_process, args=("rabbitmq-server",))
        index_p  = Process(target=cmd_process, args=("index_server .",))
        server_p.start()
        rabbit_p.start()
        sleep(2)
        index_p.start()
        sleep(0.5)
        rabbitmq_go = True

    else:
        print "I AM CLIENT "+hostname
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://"+servername+":5566")

        response = False
        while not response:
            socket.send("Awake?")
            message = socket.recv()
            print "Response: "+message
            response = ( message == "Yes" )


    # Starting image2plate jobs
    print hostname + " Running jobs!!!!!!!!!!"
    #os.system("echo *.tif | xargs -n1 echo | awk '{if(NR%"+str(number_of_nodes)+"=="+str(node_number)+"){print}}' | xargs -n1 -P 8 image2plate -m equi --file tif -o pf://"+servername+"/index/test.plate")

    # Phone home and say I'm finished
    if ( node_number != 0 ):
        socket.send("Finished")

    # Clean up
    if ( node_number == 0 ):
        while ( finish_count != number_of_nodes ):
            print "finish_count = "+str(finish_count)
            sleep(2)
        server_p.terminate()
        rabbit_p.terminate()
        index_p.terminate()

    print "SCRIPT FINISHED"
