import socketserver
from PIL import Image
import struct
import math

image_count = 0

def make_image_rgb565(raw):
    img = Image.new('RGB', (1600,1200))
    width, height = img.size
    data = img.load()

    try:
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                v = struct.unpack('<H', raw[2*idx:2*(idx+1)])[0]

                r, g, b = v >> (5 + 6), (v >> 5) & 0b111111, v & 0b11111 

                r = math.floor(r / 0x1f * 0xff)
                g = math.floor(g / 0x3f * 0xff)
                b = math.floor(b / 0x1f * 0xff)
                data[x, y] = (r, g, b)
    except struct.error:
        print("Incomplete image")

    global image_count
    img.save("image{}.png".format(image_count))
    image_count += 1

def make_image_yuv(raw):
    img = Image.new('YCbCr', (1600,1200))
    width, height = img.size
    data = img.load()

    try:
        for y in range(height):
            for x in range(width//2):
                idx = y * width//2 + x
                y0 = raw[4*idx + 0]
                u = raw[4*idx + 1]
                y1 = raw[4*idx + 2]
                v = raw[4*idx + 3]

                data[x*2, y] = (y0, u, v)
                data[x*2+1, y] = (y1, u, v)
    except IndexError:
        print("Incomplete image")

    global image_count
    img.convert('RGB').save("image{}.png".format(image_count))
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
            #print("{} wrote {} bytes".format(self.client_address[0], len(self.data)))
            self.image_data += self.data
            #try:
            #    self.request.send(b"OK")
            #except ConnectionAbortedError:
            #    pass
            
    def finish(self):
        print("Writing image {}".format(image_count))
        make_image_rgb565(self.image_data)

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 4242

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
