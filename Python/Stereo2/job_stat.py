#!/usr/bin/env python
import zmq, sys

if __name__ == '__main__':
    url = sys.argv[1]
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect( url )

    socket.send("STAT")
    message = socket.recv()
    print message
