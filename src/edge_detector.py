import cv2
print(cv2.__path__)
import numpy as np


imPath = ""  # <----- image path


def imageResize(orgImage, resizeFact):
    # dim = (int(orgImage.shape[1]*resizeFact),       
    #        int(orgImage.shape[0]*resizeFact))  # w, h
    # return cv2.resize(orgImage, dim, cv2.INTER_AREA)
    return orgImage

img = imageResize(cv2.imread(imPath), 0.5)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

print(gray.shape)
                  
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)) # <-----                                                                             
morphed = cv2.dilate(gray, kernel, iterations=1)

thresh = cv2.inRange(morphed,  155, 255)  # to pick only black squares

# find canny edge
edged_wide = cv2.Canny(thresh, 10, 200, apertureSize=3)
# cv2.waitKey(0)

# find Contours
contours, hierarchy = cv2.findContours(
    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # cv2.CHAIN_APPROX_NONE stores all coords unlike SIMPLE, cv2.RETR_EXTERNAL


cntImg = img.copy()

minArea, maxArea = 2000, 9000

valid_cnts = []
for c in contours:
    area = cv2.contourArea(c)
    if area > minArea and area < maxArea:
        valid_cnts.append(c)

        # draw centers 
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(cntImg, (cX, cY), 5, (0, 0, 255), -1)


cv2.drawContours(cntImg, valid_cnts, -1, (0, 255, 0), 2)

cv2.imwrite('../output/threshold.png', thresh)
cv2.imwrite('../output/morphed.png', morphed)
cv2.imwrite('../output/canny.png', edged_wide)
cv2.imwrite('../output/contour.png', cntImg)
# cv2.waitKey(0)

# cv2.destroyAllWindows()