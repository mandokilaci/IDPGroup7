import cv2
import numpy as np
import copy

class Img:
    def __init__(self, img):
        self.img = img
        self.centres = []
        

    def draw_axis(self, p, q, colour, scale=0.3):
        """Draws axes of objects"""

        angle = np.arctan2(p[1] - q[1], p[0] - q[0]) #radians
        angle_deg = angle * 180 / np.pi  #degrees

        hypotenuse = np.sqrt((p[1] - q[1]) ** 2 + (p[0] - q[0]) ** 2)
        q = np.zeros_like(q, dtype=np.int64)

        #Lengthen the arrow by a factor of scale
        q[0] = int(p[0] - scale * hypotenuse * np.cos(angle))
        q[1] = int(p[1] - scale * hypotenuse * np.sin(angle))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

        #Create the arrow hooks
        p[0] = int(q[0] + 9 * np.cos(angle + np.pi / 4))
        p[1] = int(q[1] + 9 * np.sin(angle + np.pi / 4))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

        p[0] = int(q[0] + 9 * np.cos(angle - np.pi / 4))
        p[1] = int(q[1] + 9 * np.sin(angle - np.pi / 4))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

    def get_orientation(self, pts):   
        """Gets orientation of an object bounded by set of points"""
        
        #Construct a buffer used by the PCA analysis
        data_pts = np.squeeze(np.array(pts, dtype=np.float64))

        #Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)

        #Store the centre of the object
        cntr = np.array([int(mean[0, 0]), int(mean[0, 1])])
        self.centres.append(cntr)

        #Draw the principal components
        cv2.circle(self.img, (cntr[0], cntr[1]), 3, (255, 0, 255), 1)
        p1 = cntr + 0.02 * eigenvectors[0] * eigenvalues[0]
        p2 = cntr - 0.02 * eigenvectors[1] * eigenvalues[1]
        self.draw_axis(copy.copy(cntr), p1, (0, 255, 0), 10)
        self.draw_axis(copy.copy(cntr), p2, (255, 255, 0), 50)

        return np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

   

    def draw_contours(self):
        """Finds contours and draws them in self.img"""

        #Convert image to grayscale
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        #Convert image to binary
        _, bw = cv2.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

        #Find all the contours 
        _, contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            
            #Ignore contours that are too small or too large
            if area < 1e2 or 1e5 < area:
                continue

            #Draw each contour only for visualisation purposes
            cv2.drawContours(self.img, contours, i, (0, 0, 255), 2)

            #Find the orientation of each shape
            self.get_orientation(contours[i])

    def display(self):
        """Displays image with contours drawn in"""

        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.imshow("output", self.img)
        cv2.waitKey()

    def get_centres(self):
        """Returns coordinates of centres of each contour"""

        return np.array(self.centres)



def process_image():
    """Performs image processing and returns the processed image object"""

    #src = cv2.imread(file_path)    (<--- insert file path to image here)
    image = Img(copy.copy(src))
    image.draw_contours()
    return image

if __name__ == '__main__':
    
    image = process_image()
    centres = image.get_centres()
    print(centres)
    image.display()
    
    