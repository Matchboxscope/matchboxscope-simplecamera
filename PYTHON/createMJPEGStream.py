import cv2
import numpy as np
import time

# Define the dimensions of the images
image_width = 640
image_height = 480

from flask import Flask, Response

app = Flask(__name__)

# Define the MJPEG stream properties
stream_port = 8080
#http://10.101.1.127:8080/stream.mjpg
stream_address = f"http://localhost:{stream_port}/stream.mjpg"

# Start capturing and streaming
print(f"Streaming MJPEG at {stream_address}")


def generate_frames():
    while True:
            
        # You can modify this part to generate different images
        # For now, let's create a simple red-blue gradient
        gradient = np.uint8(np.random.randn(image_height, image_width, 3)*255)
        
        # Encode the image to JPEG format
        ret, jpeg = cv2.imencode('.jpg', gradient)

        if not ret:
            print("Failed to encode the frame.")
            break

        # Convert the JPEG image to bytes
        jpeg_bytes = jpeg.tobytes()

        # Write the MJPEG boundary and image bytes to the stream
        stream_frame = (b'--myboundary\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
        
        # Simulate delay between frames (adjust as needed)
        time.sleep(0.1)
        # Yield the frame as bytes with the MJPEG boundary
        yield (b'--myboundary\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')

# Route for streaming MJPEG
@app.route('/stream.mjpg')
def stream():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=myboundary')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, threaded=True)