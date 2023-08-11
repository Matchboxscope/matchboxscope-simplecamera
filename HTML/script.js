document.getElementById('connectButton').addEventListener('click', () => {
    let serialDevice;
    let waitForNextFrame = true;
    let lastFrameTimestamp = Date.now();

    async function connectToUSBDevice(manufacturer = "Espressif") {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 2000000});
        return port;
    }

    async function initCam(device) {
        device.write(new TextEncoder().encode('t10\n'));
        await new Promise(resolve => setTimeout(resolve, 50)); // Sleep for 50ms
    }

    async function readImageData(device) {
        // Adjust these numbers based on your specific base64 size calculation
        const base64Length = Math.ceil((320 * 240 * 4) / 3); 
        const reader = device.readable.getReader();
        const { value } = await reader.read();
        reader.releaseLock();
        const base64Data = new TextDecoder().decode(value);

        return base64Data;
    }

    async function main() {
        serialDevice = await connectToUSBDevice();
        if (!serialDevice) return;

        const writer = serialDevice.writable.getWriter();

        //await initCam(serialDevice);
        while (true) {
            try {
                await writer.write(new TextEncoder().encode('\n'));
                await new Promise(resolve => setTimeout(resolve, 50));

                writer.releaseLock();
                
                const base64Image = await readImageData(serialDevice);
                const imageElement = document.getElementById('serialImage');
                imageElement.src = `data:image/png;base64,${base64Image}`;

                if (waitForNextFrame) {
                    waitForNextFrame = false;
                } else {
                    const currentTimestamp = Date.now();
                    console.log(`framerate: ${1000 / (currentTimestamp - lastFrameTimestamp)}`);
                    lastFrameTimestamp = currentTimestamp;
                }
            } catch (e) {
                console.error("Error:", e);
                await serialDevice.write(new TextEncoder().encode('r\n'));
                await new Promise(resolve => setTimeout(resolve, 500));
                await initCam(serialDevice);
                waitForNextFrame = true;
            }
        }
    }

    main();
});
