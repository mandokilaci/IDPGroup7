import cv2
import numpy as np

import socket
import time
import threading

UDP_IP = "10.248.194.40"  # IP address of phone
#UDP_IP = "129.169.213.25"  # IP address of arduino

IP_ADDR = "10.248.223.89"  # IP address of pc
UDP_PORT = 2390 # UDP port for communication

class Image:
    def __init__(self):
        
        #self.img = src
        #self.shape = src.shape
        self.centers = []
        self.robot_center = []

        print('Initialising camera')
        self.cap = cv2.VideoCapture('http://localhost:8081/stream/video.mjpeg')
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 128)
        self.cap.set(cv2.CAP_PROP_SATURATION, 128)
        print('Camera initialised')

    def get_table_roi(self, frame):
        """Crops original frame and returns frame of only the competition table"""

        #Custom roi hardcoded for table 1 to avoid detecting the floor
        table_mask = np.zeros(frame.shape, dtype=np.uint8)
        roi_corners = np.array([[(230,117),(300,90), (600, 40),(700,45),(940,50), (1000, 300),(1040,800), (750,870),(270, 880),(220,500)]], dtype=np.int32)
        # fill the ROI so it doesn't get wiped out when the mask is applied
        channel_count = frame.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,)*channel_count
        cv2.fillPoly(table_mask, roi_corners, ignore_mask_color)

        frame = cv2.bitwise_and(frame, table_mask)

        return frame
    
    def get_block_centers(self, frame, minArea = 30, threshold_val = 100):
        """Appends the centers of the 4 smallest red contours, which correspond to the blocks' centers, to self.centers
        and returns image with bounding boxes drawn"""
        
        #frame = cv2.imread('2Capture2.png')


        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        frame = self.get_table_roi(frame)

        # Threshold the HSV image to get only red colours
        mask1 = cv2.inRange(hsv, (0, 70, 20), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 70, 20), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)


        kernel = np.ones((5, 5), 'int')
        dilated = cv2.dilate(mask, kernel)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        ret, thrshed = cv2.threshold(cv2.cvtColor(res, cv2.COLOR_BGR2GRAY), threshold_val, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thrshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        suitable_contours = []


        #Retain suitable contours that are larger than minimum area and of the right dimensions
        for i in range(len(contours)):
            # Contour area is taken
            #area = cv2.contourArea(contours[i])
            (x, y, w, h) = cv2.boundingRect(contours[i])
            if w*h > minArea and w < 40 and h < 40:
                
                suitable_contours.append(contours[i])
        
        block_contours = sorted(suitable_contours, key=cv2.contourArea)
        block_contours = block_contours[:4]   #4 smallest contours corresponding to blocks' contours
    
        for j in range(len(block_contours)):
            (x, y, w, h) = cv2.boundingRect(block_contours[j])
            img = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            x_c = x + w*0.5
            y_c = y + h*0.5
            self.centers.append((x_c,y_c))

        return img

    def get_robot_center(self, frame, threshold_val = 100):
        """Appends the center of green contour, which correspond to the robot's center, to self.robot_center
        and returns image with contour drawn"""
        
        #frame = cv2.imread('2Capture2.png')

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        frame = self.get_table_roi(frame)

        # Threshold the HSV image to get only green colours
        
        mask = cv2.inRange(hsv, (36, 25, 25), (86, 255,255))


        kernel = np.ones((5, 5), 'int')
        dilated = cv2.dilate(mask, kernel)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        ret, thrshed = cv2.threshold(cv2.cvtColor(res, cv2.COLOR_BGR2GRAY), threshold_val, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thrshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


        green_contours = sorted(contours, key=cv2.contourArea)
        robot_contour = green_contours[-1]  #Assumes that robot contour is the largest green contour
        M = cv2.moments(robot_contour)
        x_c = int(M["m10"] / M["m00"])
        y_c = int(M["m01"] / M["m00"])
        self.robot_center.append((x_c,y_c))
        img = cv2.drawContours(frame, robot_contour, -1, (255, 0, 0), 2)
        
        return img


    def capture(self):
        """Returns an OpenCV frame (numpy array) of the table"""

        _, frame = self.cap.read()
        return frame

    
    def show_frame(self, frame):
        """Shows the frame/ drawn contours on computer screen"""

        cv2.imshow('frame', frame)
        cv2.waitKey(25)

    def process_image(self, minArea = 30, threshold_val = 100):
        """Captures image frame and performs image processing"""

        frame = self.capture()
        block_img = self.get_block_centers(frame, minArea, threshold_val)
        robot_img = self.get_robot_center(frame, threshold_val)
        #self.show_frame(img)

    def shutdown(self):
        """Shuts down OpenCV"""

        self.cap.release()
        cv2.destroyAllWindows()


def send_centers(sock, centers):
    """Send coordinates of blocks' centers to Arduino"""

    for coordinate in centers:
        time.sleep(0.1)
        message = str(coordinate)
        print(message)
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

def send_robot_center(sock, robot_center):
    """Send coordinates of robot center to Arduino"""

    time.sleep(0.1)
    message = "r" + str(robot_center)
    print(message)
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))


class ReceiveThread (threading.Thread):
   def __init__(self, threadID, name, sock):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.socket = sock
   def run(self):
      print("Starting " + self.name)
      while True:
        receive_data(self.socket)
      print("Ending " + self.name)

class CvThread (threading.Thread):
   def __init__(self, threadID, name, sock):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.socket = sock
   def run(self):
      print("Starting " + self.name)
      img = Image()
      img.process_image(minArea = 30, threshold_val = 100)
      centers = img.centers
      print(centers)
      send_centers(self.socket, centers)
      robot_center = img.robot_center
      print("r"+str(robot_center))
      send_robot_center(self.socket, robot_center)
      img.shutdown()
      print("Ending " + self.name)

def receive_data(sock):
    """Receives data from the Arduino and only breaks out of this when
    b'Connection established' is received"""
    data, addr = sock.recvfrom(1024)  # buffer size of 1024 bytes
    print("Recieved: "+str(data)+" from "+str(addr))
    if(data==b'block centers'):
        cvThread=CvThread(1,"CvThread", sock)
        cvThread.start()
        

def send_data(sock):
    line=input()
    sock.sendto(line.encode(), (UDP_IP, UDP_PORT))
    print(line+" has been sent")
    

if __name__ == '__main__':
    print("main")
    #init socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP_ADDR, UDP_PORT))
    receiveThread=ReceiveThread(1,"ReceiveThread", sock)
    receiveThread.start()
    while True:
        send_data(sock)
        print("data")

