import threading
import requests
import time
import cv2
import urllib.request

class ESP32Microscope(object):
    
    def __init__(self, baseHost=None):
        self.streamRunning = False
        if baseHost is None:
            self.baseHost = "http://192.168.137.87"
        else:
            self.baseHost = baseHost

    def setParameter(self, uid, value):
        url = f"{self.baseHost}/control?var={uid}&val={value}"
        response = requests.get(url, timeout=.10)

        return response.text  # prints the response content as text

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
        url = self.baseHost+":81/view"
        stream = urllib.request.urlopen(url, timeout=5)
        bytes = bytes()
        while self.streamRunning:
            bytes += stream.read(1024)
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
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