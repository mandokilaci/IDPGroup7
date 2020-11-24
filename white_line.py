import cv2
import numpy as np
import math

if __name__ == "__main__":
    frame = cv2.imread('Capture2.png')
    


    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    table_array = [(230,117),(300,90), (600, 40),(700,45),(940,50), (1000, 300),(870,800), (750,870),(270, 880),(220,500)]
    #og_table_array = [(230,117),(300,90), (600, 40),(700,45),(940,50), (1000, 300),(1040,800), (750,870),(270, 880),(220,500)]
    array1 = [(550, 50), (780,50),(830,650), (800,830), (550, 850)]
    

        
    # Threshold the HSV image to get only cell colours
    
    table_mask = np.zeros(frame.shape, dtype=np.uint8)
    roi_corners = np.array([table_array], dtype=np.int32)
    array1_corners = np.array([array1], dtype = np.int32)

    # fill the ROI so it doesn't get wiped out when the mask is applied
    channel_count = frame.shape[2]  # i.e. 3 or 4 depending on your image
    ignore_mask_color = (255,)*channel_count
    cv2.fillPoly(table_mask, [roi_corners, array1_corners], ignore_mask_color)

    # apply the mask
    frame = cv2.bitwise_and(frame, table_mask)
    mask = cv2.inRange(hsv, (0, 0, 240), (255, 255, 255))
    
        

    kernel = np.ones((5, 5), 'int')
    dilated = cv2.dilate(mask, kernel)
        # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)
    ret, thrshed = cv2.threshold(cv2.cvtColor(res, cv2.COLOR_BGR2GRAY), 100, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(thrshed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


    for i in range(len(contours)):
        # Contour area is taken
        img = cv2.drawContours(frame, contours, i, (0, 255, 0), 2)
        
    
    
    
    print(contours)
    cv2.imshow("output", img)
    cv2.waitKey()


    