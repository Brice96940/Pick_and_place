import numpy as np
import cv2 as cv
from util import get_limits

class red_detect:
    def __init__(self):

        self.red = [0,0,255]
        self.color_info = (0, 255, 255)
    
    def Opencv_red(self,frame ):
        #cap = cv.VideoCapture(0)
        """""
        if not cap.isOpened():
            print("Cannot open camera")
            exit()"""
        
        while True:
            # Capture frame-by-frame
            #et, frame = cap.read()
            # if frame is read correctly ret is True
            #if not ret:
            #    print("Can't receive frame (stream end?). Exiting ...")
            #    break
            # Our operations on the frame come here
            #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            #charger une image en mode hsv
            image_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            #cv.inRange(image_hsv, lower_couleur, upper_couleur)
            #creer une image binaire en fonction de la couleur choisie
            Lower_red,upper_red = get_limits(self.red)
            maskred = cv.inRange(image_hsv ,Lower_red ,upper_red )
            
            if Lower_red[0] == 0 :
                Lower_red1 = np.array([168,100,100])
                upper_red1 = np.array([180,255,255])
                masked1 =cv.inRange(image_hsv ,Lower_red1 ,upper_red1 )
            else:
                masked1 = maskred

            kernel = np.ones((5,5), np.uint8)


            #eroder l'image
            #binary_image = cv.erode(maskred,kernel,iterations = 1)

            #optimisation de la binarisation
            #ret,binnary_image = cv.threshold(maskred, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

            #recuperer la couleur rouge 
            result_red = cv.bitwise_or(maskred ,masked1)

            #ouverture 
            binary_image = cv.morphologyEx(result_red , cv.MORPH_OPEN,kernel)

            #determination des contours 
            element = cv.findContours(binary_image,cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[-2]

            #determination du barycentre

            if len(element) > 0:
                c = max(element, key = cv.contourArea)
                ((x,y), rayon) = cv.minEnclosingCircle(c)

                if rayon > 30:
                    cv.circle(binary_image, (int(x),int(y)), int(rayon), self.color_info, 2)
                    cv.circle(frame, (int(x),int(y)), 5, self.color_info, 10)
                    cv.line(frame, (int(x),int(y)), (int(x)+150,int(y)), self.color_info, 2)
                    cv.putText (frame, "objet rouge",(int(x)+10, int(y)-10), cv.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv.LINE_AA)
            

            # Display the resulting frame
            cv.imshow('binary_image', binary_image)
            cv.imshow('frame', frame)
            if cv.waitKey(1) == ord('q'):
                break
        
# When everything done, release the capture
#cap.release()
#cv.destroyAllWindows()