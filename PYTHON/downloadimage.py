import requests
from PIL import Image
import numpy as np

url = "http://192.168.43.146/capture?_cb=test"  # Replace with the actual URL

response = requests.get(url)

if response.status_code == 200 and response.headers.get('content-type') == 'image/jpeg':
    with open("image.jpg", "wb") as f:
        f.write(response.content)
    print("Image downloaded successfully as 'image.jpg'")
    
    # Open the image using PIL
    pil_image = Image.open("image.jpg")
    
    # Convert the PIL image to a numpy array
    image_array = np.array(pil_image)
    
    print("Image converted to a numpy array")
else:
    print("Failed to download the image")
