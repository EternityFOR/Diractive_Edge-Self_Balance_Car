import os
from http.server import SimpleHTTPRequestHandler, HTTPServer

PORT = 8080

FIRMWARE_DIR = r"D:\6.Programming\1.Project_softwares\1.Personal_Projects\Self_Balance_Car\1.Project_Data\Self_Balance_Car_ESP32S3_N8R8\build"

os.chdir(FIRMWARE_DIR)

class OTARequestHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        print(f"Serving OTA request for: {self.path}")
        super().do_GET()

server_address = ('', PORT)
httpd = HTTPServer(server_address, OTARequestHandler)
print(f"Serving OTA updates from {FIRMWARE_DIR} on port {PORT}")
print(f"Access your firmware at http://<your_ip_address>:{PORT}/smart_car.bin")
httpd.serve_forever()
