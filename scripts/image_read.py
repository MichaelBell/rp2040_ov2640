import socketserver
from PIL import Image
import struct
import math

image_count = 0

def make_image(raw):
    img = Image.new('RGB', (352,288))
    width, height = img.size
    data = img.load()

    for y in range(height):
        for x in range(width):
            idx = y * width + x
            v = struct.unpack('<H', raw[2*idx:2*(idx+1)])[0]

            r, g, b = v >> (5 + 6), (v >> 5) & 0b111111, v & 0b11111 

            r = math.floor(r / 0x1f * 0xff)
            g = math.floor(g / 0x3f * 0xff)
            b = math.floor(b / 0x1f * 0xff)
            data[x, y] = (r, g, b)

    global image_count
    img.save("image{}.bmp".format(image_count))
    image_count += 1

class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def setup(self):
        print("Connection from {}".format(self.client_address[0]))
        self.image_data = b""

    def handle(self):
        # self.request is the TCP socket connected to the client
        while True:
            self.data = self.request.recv(1024)
            if len(self.data) == 0: break
            print("{} wrote {} bytes".format(self.client_address[0], len(self.data)))
            self.image_data += self.data
            self.request.send(b"OK")
            
    def finish(self):
        make_image(self.image_data)

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 4242

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()