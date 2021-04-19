#!/usr/bin/env python
# coding: utf-8

# In[ ]:





# In[5]:

import math
import numpy as np
import cv2
import rospy
import geometry_msgs.msg
#from rospy_tutorials.msg import Floats



def display(im, bbox):
    n = len(bbox)
    for j in range(n):
        cv2.line(im, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)
 
    # Display results
    cv2.imshow("Results", im)
# több kamera esetén a szám jelöli a kamerát - ha -1 van ott akkor az utolsó kamera lesz akitiválva

pub = rospy.Publisher('saf/qrkam',geometry_msgs.msg.Point, queue_size=10)
rospy.init_node('kam', anonymous=True)

cap = cv2.VideoCapture(0)

szamol=0
# végtelen ciklus inítása a képek beolvasására
# fúj fúj végtelen ciklus
while(True):
    # a videocapture read függvényével megkaphatjuk a képet
    ret, frame = cap.read()
    
    # ha kiíratjuk háthatjuk, hogy a ret azt adja meg, hogy sikerült-e olvasni
    # print(ret)q
    
    # Múlt órai átalakítás
    qrDecoder = cv2.QRCodeDetector()
 
    # Detect and decode the qrcode
    data,bbox,rectifiedImage = qrDecoder.detectAndDecode(frame)
    if len(data)>0 and szamol%20 ==0 :
        #print("Decoded Data : {}".format(data))
        n = len(bbox)
        for j in range(n):
            cv2.line(frame, tuple(bbox[j][0]), tuple(bbox[ (j+1) % n][0]), (255,0,0), 3)
        print(bbox)
        x1=bbox[0,0]
        x1=x1[0]
        x2=bbox[1,0]
        x2=x2[0]
        x3=bbox[2,0]
        x3=x3[0]
        y1=bbox[0,0]
        y1=y1[1]
        y2=bbox[1,0]
        y2=y2[1]
        y4=bbox[3,0]
        y4=y4[1]
       
        ter=(x2-x1)*(y4-y1)
        kul= (110*110 - ter)   #ha nagyobb, tehát közelebb van, akkor negatív!!!
        kp=geometry_msgs.msg.Point()
        kp.x=x1+(x3-x1)/2
        kp.y=y4+(y2-y4)/2
        kp.z=kul
        #kp=np.array([x1+(x3-x1)/2, y4+(y2-y4)/2])
        #print(kp.x, kp.y,kp.z)
       # pub.publish(kp)   

        tav=geometry_msgs.msg.Point()
        tav.x=kp.x-320
        tav.y=kp.y-(475/2)
        if kul>0:
            tav.z=math.sqrt(kul)
            
        else:
            tav.z=-math.sqrt(-kul)
        
        kp.z=tav.z
        print(tav.x, tav.y, tav.z)
        pub.publish(tav)
        
    szamol=szamol+1

        #display(inputImage, bbox)
        #rectifiedImage = np.uint8(rectifiedImage);
        #cv2.imshow("Rectified QRCode", rectifiedImage);
        #ok = tracker.init(frame, bbox)
    cv2.imshow('frame',frame)

  # az OpenCV beépített megjelenítőjét használhatjuk, ezzel a legkönnyebb megnézni.
    
    
    # várunk egy 'q'-ra a kilépéshez a ciklusból
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# A kamerát elengedi, nem fog a led mellette tovább világítani.q
cap.release()
# Ha sok ablak lenne, ez a leghasznosabb parancs 
# Érdemes megpróbálni lekapcsolni kézzel az ablakokat még futás közben - elég nehéz
cv2.destroyAllWindows()


# In[4]:





# In[ ]:




