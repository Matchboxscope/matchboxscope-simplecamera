import threading
import requests
import time
import cv2
import urllib.request
import numpy as np

class ESP32Microscope(object):
    
    def __init__(self, baseHost=None):
        self.streamRunning = False
        if baseHost is None:
            self.baseHost = "http://192.168.2.201"
        else:
            self.baseHost = baseHost

    def setParameter(self, uid, value):
        url = f"{self.baseHost}/control?var={uid}&val={value}"
        try:
            response = requests.get(url, timeout=.50)
            return response.text
        except Exception as e:
            print(e)
            return -1

          # prints the response content as text

    def setLED(self, value=0):
        uid = "lamp"
        self.setParameter(uid, value)
        
    def startStream(self):
        print("Starting Stream")
        if not self.streamRunning:
            self.captureThread = threading.Thread(target=self.generateFrames)
            self.captureThread.start()
            
    def stopStream(self):
        self.streamRunning = False
        self.captureThread.join()
        
    def generateFrames(self):
        self.streamRunning = True
        url = self.baseHost+":81"
        stream = urllib.request.urlopen(url, timeout=5)
        bytesJPEG = bytes()
        while self.streamRunning:
            
            bytesJPEG += stream.read(1024)
            print(bytesJPEG)
            a = bytesJPEG.find(b'\xff\xd8')
            b = bytesJPEG.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytesJPEG[a:b+2]
                bytesJPEG = bytesJPEG[b+2:]
                frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                print("Returning frame")
                yield frame
    
    def returnFrames(self, callBackFct = None):
        for frame in self.generateFrames():
            if callBackFct is None:
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
            else:
                callBackFct(frame)
                
                
    

mESP = ESP32Microscope()
mESP.setLED(50)
time.sleep(1)
mESP.setLED(0)

mESP.startStream()
mESP.returnFrames()