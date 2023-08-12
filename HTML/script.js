function calculateBase64Length(width, height) {
    const numBytes = width * height;
    let base64Length = Math.ceil((numBytes * 4) / 3);
    base64Length = base64Length + (4 - base64Length % 4) % 4;
    return base64Length +2;
}

function base64ToImage(base64String) {
    // Convert base64 string to byte array
    const byteCharacters = atob(base64String);
    const byteNumbers = new Array(byteCharacters.length);
    for (let i = 0; i < byteCharacters.length; i++) {
        byteNumbers[i] = byteCharacters.charCodeAt(i);
    }
    const byteArray = new Uint8Array(byteNumbers);
    
    return byteArray;
}

async function readFromDevice(device, width, height, timeout = 2000) {
    const length = calculateBase64Length(width, height);
    const reader = device.readable.getReader();

    let accumulatedData = "";
    let timeoutId;

    const timeoutPromise = new Promise((_, reject) => {
        timeoutId = setTimeout(() => reject(new Error("Read timeout")), timeout);
    });

    try {
        while (accumulatedData.length < length) {
            const result = await Promise.race([reader.read(), timeoutPromise]);
            clearTimeout(timeoutId); // Clear the timeout every successful read
            
            // Append the newly read data to the accumulated data
            accumulatedData += new TextDecoder().decode(result.value);
        }
        
        reader.releaseLock();
        return accumulatedData;
    } catch (error) {
        reader.releaseLock();
        throw error;
    }
}



// Attempts to connect to a USB device.
async function connectToUSBDevice(manufacturer = "Espressif") {
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 2000000 });
    return port;
}

// Sends an initialization command to the device.
async function initCam(device) {
    device.write(new TextEncoder().encode('t10\n'));
    await new Promise(resolve => setTimeout(resolve, 50)); // Sleep for 50ms
}

// Reads image data from the device.
async function readImageData(device, timeout = 2000) {
    const reader = device.readable.getReader();

    let timeoutId;
    const timeoutPromise = new Promise((_, reject) => {
        timeoutId = setTimeout(() => reject(new Error("Read timeout")), timeout);
    });

    try {
        const result = await Promise.race([reader.read(), timeoutPromise]);
        clearTimeout(timeoutId);
        reader.releaseLock();
        return new TextDecoder().decode(result.value);
    } catch (error) {
        reader.releaseLock();
        throw error;
    }
}

document.getElementById('connectButton').addEventListener('click', async () => {
    let serialDevice;
    let waitForNextFrame = true;
    let lastFrameTimestamp = Date.now();

    try {
        serialDevice = await connectToUSBDevice();
        if (!serialDevice) return;

        const writer = serialDevice.writable.getWriter();

        while (true) {
            try {
                await writer.write(new TextEncoder().encode(' \n'));
                await new Promise(resolve => setTimeout(resolve, 50));

                //writer.releaseLock();

                //const base64Image = await readImageData(serialDevice);
                width = 320;
                height = 240;
                // For example, reading an image of width 320 and height 240
                const base64Image = await readFromDevice(serialDevice, 320, 240);
                console.log(`Received: ${base64Image}`);

                const imageBytes = base64ToImage(base64Image);

                // You now have a Uint8Array representing the image. 
                // If you want to display this image, you can convert it to a blob and set it as a src of an img tag
                const blob = new Blob([imageBytes], { type: "image/png" }); // Assuming it's a PNG image. Adjust if necessary.
                const imageUrl = URL.createObjectURL(blob);
                document.getElementById('serialImage').src = imageUrl;            
                //await new Promise(resolve => setTimeout(resolve, 500));
                //const imageElement = document.getElementById('serialImage');
                //imageElement.src = `data:image/png;base64,${base64Image}`;

                if (waitForNextFrame) {
                    waitForNextFrame = false;
                } else {
                    const currentTimestamp = Date.now();
                    console.log(`framerate: ${1000 / (currentTimestamp - lastFrameTimestamp)}`);
                    lastFrameTimestamp = currentTimestamp;
                }
            } catch (e) {
                console.error("Error:", e);
                //await writer.write(new TextEncoder().encode('r\n'));
                //await new Promise(resolve => setTimeout(resolve, 500));
                //await initCam(serialDevice);
                waitForNextFrame = true;
            }
        }
    } catch (e) {
        console.error("Failed to connect:", e);
    }
});
