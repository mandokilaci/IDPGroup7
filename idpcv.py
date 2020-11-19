import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from collections import Counter
import socket
import time

UDP_IP = ""  # IP address of Arduino
IP_ADDR = ""  # IP address of pc
UDP_PORT = 8080 # UDP port for communication

class Image:
    def __init__(self):
        
        #self.img = src
        #self.shape = src.shape
        self.centers = []

        print('Initialising camera')
        self.cap = cv2.VideoCapture(1)
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



'''

def visualize(file_name):
    """Purely for testing and visualization. 
    Returns a matplotlib plot to visualize the positions of the blocks and red delivery targets"""
    
    src = cv2.imread(file_name)
    image = Image(src)
    x_list, y_list, data = image.get_clusters(reference_RGB=[230,50,80])
    image.get_block_centers(data)

    for coordinate in image.centers:
        print(str(coordinate))
    #print(image.centers)
    #print(image.centers.size)

    fig, ax = plt.subplots(figsize =(8,6))
    ax.plot(x_list, y_list, 'o', c = 'red')
    ax.set_xlim(0,image.shape[1])
    ax.set_ylim(-image.shape[0],0)
    plt.show()
'''

def send_centers(sock, centers):
    """Send coordinates of blocks' centers to Arduino"""

    for coordinate in centers:
        time.sleep(0.1)
        message = str(coordinate)
        print(message)
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

def receive_data(sock):
    """Receives data from the Arduino and only breaks out of this when
    b'Connection established' is received"""
    
    while True:
        data, addr = sock.recvfrom(1024)  # buffer size of 1024 bytes
        print(data)
        if data == b'Connection established':
            break



    

    
        

if __name__ == '__main__':

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP_ADDR, UDP_PORT))
    receive_data(sock)

    img = Image()
    img.process_image(reference_RGB = [230,50,80], tolerance_value =  100)
    centers = img.centers
    print(centers)
    send_centers(sock, centers)

    img.shutdown()



    #while img.cap.isOpened() and (not keyboard.is_pressed('c')):

    
    #visualize("2Capture2.png")
    
    
    




