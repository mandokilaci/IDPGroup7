import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from collections import Counter
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

        print('Initialising camera')
        self.cap = cv2.VideoCapture('http://localhost:8081/stream/video.mjpeg')
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 128)
        self.cap.set(cv2.CAP_PROP_SATURATION, 128)
        print('Camera initialised')
    
    def get_clusters(self, frame, reference_RGB, tolerance_value = 100):
        """Gets all the available clusters in the LHS of the image 
        and returns all the coordinates of the cluster data points"""

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    #convert BGR to RGB
        #points = np.zeros_like(img)
        #x_list = []
        #y_list = []
        data = []

        for ij in np.ndindex(img.shape[:2]):
            mae = abs(reference_RGB[0]-img[ij][0]) + abs(reference_RGB[1]-img[ij][1]) + abs(reference_RGB[2]-img[ij][2])
            if mae < tolerance_value and ij[1] < (img.shape[1])*0.5:
                #x_list.append(ij[1])
                #y_list.append(-ij[0])
                #points[ij[0]][ij[1]] = img[ij]
                data.append([ij[1],-ij[0]])


        return data



    def get_block_centers(self, data):
        """Assumes that there are 6 clusters and sorts the clusters based on number of labels and 
        return the centers of the 4 smallest clusters which correspond to the blocks"""
        
        kmeans_model = KMeans(n_clusters = 6, random_state = 12)
        kmeans_model.fit(data)
        lab = Counter(kmeans_model.labels_)
        lab_list = sorted(lab, key = lab.get, reverse = False)
        lab_list = lab_list[:4]

        for i in range(len(lab_list)):
            self.centers.append(kmeans_model.cluster_centers_[lab_list[i]])

    def capture(self):
        """Returns an OpenCV frame (numpy array) of the table"""

        _, frame = self.cap.read()
        return frame

    
    def show_frame(self, frame):
        """Shows the captured frame on computer screen"""

        cv2.imshow('frame', frame)
        cv2.waitKey(25)

    def process_image(self, reference_RGB = [230,50,80], tolerance_value = 100):
        """Captures image frame and performs image processing"""

        frame = self.capture()
        data = self.get_clusters(frame, reference_RGB, tolerance_value)
        self.get_block_centers(data)

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
      img.process_image(reference_RGB = [230,50,80], tolerance_value =  100)
      centers = img.centers
      print(centers)
      send_centers(self.socket, centers)
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


        