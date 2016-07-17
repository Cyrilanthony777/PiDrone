#/usr/bin/env python

import numpy as np
import cv2
import os
import socket

# Change eth0 to wlan0 if you are using wifi

f = os.popen('ifconfig eth0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
ip=f.read()
IP_ADDR = ip
PORT = 5010
udp = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp.bind((IP_ADDR,PORT))
udp.settimeout(0.01)


help_message = '''
Code Customized by Cyril Anthony for Multirotor application

'''

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lx =0
    ly = 0

    os.system('cls' if os.name == 'nt' else 'clear')
    for tx in fx:
        lx = tx+lx
    lx = lx/300

    for ty in fy:
        ly = ty+ly
    ly=ly/300

    txStr = "FLOW,"+str(lx)+","+str(ly)
    try:
        udp.sendto(txStr,("192.168.2.1",5010))
    except:
        print "Unable to send Data to Server"

    print lx
    print ly
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


if __name__ == '__main__':
    import sys
    print help_message
    try: fn = sys.argv[1]
    except: fn = 0

    cam = cv2.VideoCapture(fn)
    cam.set(3,320)
    cam.set(4,240)
    ret, prev = cam.read()
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()

    while True:
        ret, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(prevgray, gray, 0.5, 3, 15, 3, 5, 1.2, 0)
        #print flow
        prevgray = gray
        #exit()

        cv2.imshow('flow', draw_flow(gray, flow))
        
        ch = 0xFF & cv2.waitKey(5)
        if ch == 27:
            break
        
    cv2.destroyAllWindows()

