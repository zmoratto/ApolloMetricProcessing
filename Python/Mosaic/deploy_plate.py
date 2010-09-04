#!/usr/bin/env python

import os, glob, subprocess, sys, time, zmq
from multiprocessing import Process, Value
from time import sleep

# Use this executable with a :
# deploy_plate.py `cat $PBS_NODEFILE`

def server_process(rabbitmq_go,finish_count):
    print "I AM SERVER "+servername
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5566")

    while True:
        message = socket.recv()
        if ( message == "Awake?" ):
            if ( rabbitmq_go.value > 0 ) :
                socket.send("Yes")
            else:
                socket.send("No")
        elif ( message == "Finished" ):
            finish_count.value = finish_count.value + 1
            socket.send("Thanks")
        else:
            print "Unknown message: "+message
            socket.send("What?")

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
        if ( hostname.find(sys.argv[i]) >= 0 ):
            node_number = i - 1
    print "Node number: " + str(node_number)

    # Spawn ZMQ
    socket = []
    server_p = []  # Process handling server responses
    rabbit_p = []  # Process running rabbitmq-server
    index_p  = []  # Process running index_server
    rabbitmq_go = Value('i',0)
    finish_count = Value('i',1)
    if ( node_number == 0 ) :
        # Start spawning the server
        server_p = Process(target=server_process, args=(rabbitmq_go,finish_count,))
        #rabbit_p = Process(target=cmd_process, args=("rabbitmq-server",))
        #index_p  = Process(target=cmd_process, args=("index_server .",))
        server_p.start()
        #rabbit_p.start()
        sleep(2)
        #index_p.start()
        sleep(0.5)
        rabbitmq_go.value = 1

    else:
        print "I AM CLIENT "+hostname
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://"+servername+":5566")

        response = False
        while not response:
            sleep(0.5)
            socket.send("Awake?")
            message = socket.recv()
            response = ( message == "Yes" )


    # Starting image2plate jobs
    print hostname + " Running jobs!!!!!!!!!!"
    #os.system("echo *.tif | xargs -n1 echo | awk '{if(NR%"+str(number_of_nodes)+"=="+str(node_number)+"){print}}' | xargs -n1 -P 8 image2plate -m equi --file tif -o pf://"+servername+"/index/test.plate")

    # Phone home and say I'm finished
    if ( node_number != 0 ):
        socket.send("Finished")

    # Clean up
    if ( node_number == 0 ):
        while ( finish_count.value != number_of_nodes ):
            sleep(2)
        server_p.terminate()
        #rabbit_p.terminate()
        #index_p.terminate()

    print "SCRIPT FINISHED"
