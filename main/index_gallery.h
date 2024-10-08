const uint8_t index_gallery_html[] = R"=====(
    <!DOCTYPE html>
<html>
<head>
    <title>File Downloads</title>
    <!-- Add Bootstrap CSS (You can use CDN links or local files) -->
    <!-- Example with CDN links -->
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css">
</head>
<body>
    <div class="container mt-5">
        <h1 class="mb-4">Files Available for Download</h1>
        <ul id="fileList" class="list-group"></ul>
    </div>

    <!-- Add Bootstrap JS (You can use CDN links or local files) -->
    <!-- Example with CDN links -->
    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        document.addEventListener('DOMContentLoaded', function () {
            const currentURL = window.location.href;
            const baseURL = currentURL.split("/")[2]; // Get the base URL

            fetch('http://' + baseURL + '/files')
                .then(response => response.text())
                .then(data => {
                    const fileList = data.split('\n');
                    const fileListElement = document.getElementById('fileList');

                    fileList.forEach(file => {
                        if (file.trim() !== '') {
                            const listItem = document.createElement('li');
                            listItem.classList.add('list-group-item');

                            const downloadLink = document.createElement('a');
                            downloadLink.href = 'http://' + baseURL + "/downloadfile?filename=" + file + ".jpg";
                            downloadLink.textContent = file;
                            listItem.appendChild(downloadLink);
                            fileListElement.appendChild(listItem);
                        }
                    });
                })
                .catch(error => {
                    console.error('Error fetching file list:', error);
                });
        });
    </script>
</body>
</html>
)=====";

size_t index_gallery_html_len = sizeof(index_gallery_html)-1;

