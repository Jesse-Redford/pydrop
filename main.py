import cv2
import time
import serial
import keyboard
import numpy as np
import serial.tools.list_ports

"""
List of common Gcode commands
https://marlinfw.org/meta/gcode/

issues installing opencv try
pip install opencv-python==4.3.0.36


p - turn recording on and off
i - zoom in
o - zoom out
l - pan left
r - pan right
u - pan up
d - pan down

h - home
space - disable steppers / enable steppers
up - move z up 
down - move z down
left - bring y forward
right - bring y back
+ - dispense water
- - suck water

x - terminate program
"""


class PyDrop():

    def __init__(self,port='/COM1',baudrate=115200):
        self.ser = serial.Serial(port, baudrate)
        time.sleep(1)
        self.terminate = False
        self.enabled = False
        self.record = False
        self.zoom = 100
        self.pan_left_right = 0
        self.pan_up_down = 0
        self.count = 0

    # function for sending gcode commands to 3d printer
    def command(self,command):
        self.ser.write(str.encode(command)); print( "Sending: " + command )
        time.sleep(.1)

        #while True:
        #    line = self.ser.readline(); print(line)
        #    if line == b'ok\n':
        #        self.ser.flush()
        #        break

    def onkeypress(self,event):

        if event.name == 'h':
            self.command("G28\r\n"); print('Homing Machine')

        elif event.name == 'up':
            self.command("G1 Z5\r\n"); print('Moving Z up')

        elif event.name == 'down':
            self.command("G1 Z-5\r\n"); print('Moving Z down')

        elif event.name == 'left':
            self.command("G1 Y5\r\n"); print('Moving Y forward')

        elif event.name == 'right':
            self.command("G1 Y-5\r\n"); print('Moving Y backward')

        elif event.name == '+':
            self.command("G1 E5\r\n"); print('Dispensing droplet')

        elif event.name == '-':
            self.command("G1 E-5\r\n"); print('Removing droplet')

        elif event.name == 'space':
            if self.enabled:
                self.command("M18\r\n"); print('Disabling Steppers')
                self.enabled = False
            else:
                self.command("M17\r\n");  print('Enabling Steppers')
                self.enabled = True

        elif event.name == 'p':
            if self.record == False:
                self.record = True; print('Recording Enabled')
            else:
                self.record = False; print('Recording Disabled')

        elif event.name =='i':
            self.zoom -= 1

        elif event.name =='o':
            self.zoom += 1

        elif event.name =='l':
            self.pan_left_right -= 1

        elif event.name =='r':
            self.pan_left_right += 1

        elif event.name == 'u':
            self.pan_up_down -= 1

        elif event.name == 'd':
            self.pan_up_down += 1

        elif event.name == 'x':
            self.terminate = True; print('Terminating Program')

        elif event.name == 'c':
            self.count += 1; print('Saving Image')
            cv2.imwrite("frame%d.jpg" % self.count, self.image)

        else:
            pass

    def run(self):

        # intalized gcode commands
        self.command("M107\r\n")    # turn off fan
        self.command("M302 S0\r\n") # always allow cold extrustion

        self.command("G91\r\n")  # set relative positioning
        #self.command("G90\r\n")  # set absolute positioning

        # intalize hook for keyboard commands
        keyboard.on_press(self.onkeypress)

        # intiate video stream & file for recording
        cap = cv2.VideoCapture(0)
        video_out = cv2.VideoWriter('output.mp4v', # name of file
                              cv2.VideoWriter_fourcc('H', '2', '6', '5'), #codec
                              20.0, # frames per second
                              (640, 480)) # frame size
        while True:

            try:
                # Read Camera
                ret, frame = cap.read()
                # Covert color image to grayscale
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # pan / zoom according to keyboard commands
                h = int(frame.shape[0]/2) +  self.pan_up_down
                w = int(frame.shape[1]/2) +  self.pan_left_right
                cropped = frame[h-self.zoom:h+self.zoom, w-self.zoom:w+self.zoom,]
                self.image = cv2.resize(cropped,(640, 480),interpolation=cv2.INTER_NEAREST)

                # show frame and cropped image side by side
                cv2.imshow('Crop', np.concatenate((frame,self.image), axis=1))

                # save the current frames if recording is enabled
                if self.record:
                    video_out.write(self.image)

                # Introduce 20 milisecond delay. press q to exit.
                if cv2.waitKey(20) == ord('q'):
                    break

                # wait for user to push x to terminate program
                if self.terminate:
                    print('Terminating Program')
                    break

            except:
                break

        # When everything done, release the video capture object
        cap.release()


        # relase video writer
        video_out.release()

        # Closes all the frames
        cv2.destroyAllWindows()


# Get list of connected COM Ports
for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
        print("{}: {} [{}]".format(port, desc, hwid))


pydrop = PyDrop(port='/COM1',baudrate=115200)
pydrop.run()

