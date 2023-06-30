import requests

def perform_ota_update(IP_ADDRESS,file_path):
    url = 'http://'+IP_ADDRESS+':82/update'  # Replace with your ESP32's IP address
    
    # Upload the file to the ESP32 OTA server
    with open(file_path, 'rb') as file:
        files = {'update': file}
        response = requests.post(url, files=files)
        
        if response.status_code == 200:
            print('File updated successfully')
            return True
        else:
            print('File upload failed')
            return False


# Example usage
IP_ADDRESS = "192.168.0.108"
binary_file_path = '/Users/bene/Dropbox/Dokumente/Promotion/PROJECTS/matchboxscope-simplecamera/.pio/build/seeed_xiao_esp32s3/firmware.bin'
perform_ota_update(IP_ADDRESS,binary_file_path)
