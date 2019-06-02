import logging
import sys
import SocketServer
import json
import Adafruit_BBIO.ADC as ADC
import time
import signal

pins = ["P9_39", "P9_40", "P9_38", "P9_37"]

class SharpRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, request, client_address, server):
        self.logger = logging.getLogger("SharpRequestHandler")
        self.logger.debug('__init__')
        SocketServer.BaseRequestHandler.__init__(self, request, client_address, server)

    def setup(self):
        self.logger.debug('setup')
        return SocketServer.BaseRequestHandler.setup(self)

    def handle(self):
        ADC.setup()
        self.logger.debug('handle')
        data = self.request.recv(1024)
        self.logger.debug('recv() -> "%s"', data)
        values = {
            "timestamp": int(round(time.time() * 1000))
        }
        for pin in pins:
            value = ADC.read(pin)*1.8
            values[pin] = value
        self.request.send(json.dumps(values))

    def finish(self):
        self.logger.debug('finish')
        return SocketServer.BaseRequestHandler.finish(self)

class SharpServer(SocketServer.TCPServer):

    def __init__(self, server_adress, handler_class=SharpRequestHandler):
        self.logger = logging.getLogger("SharpServer")
        self.logger.debug('__init__')
        SocketServer.TCPServer.__init__(self, server_adress, handler_class)
        return

    def server_activate(self):
        self.logger.debug('server_activate')
        SocketServer.TCPServer.server_activate(self)
        return

    def serve_forever(self, **kwargs):
        self.logger.debug('waiting for request')
        self.logger.info('Handling requests, press <Ctrl-C> to quit')
        while True:
            self.handle_request()
        return

    def handle_request(self):
        self.logger.debug('handle_request')
        return SocketServer.TCPServer.handle_request(self)

    def verify_request(self, request, client_address):
        self.logger.debug('verify_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.verify_request(self, request, client_address)

    def process_request(self, request, client_address):
        self.logger.debug('process_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.process_request(self, request, client_address)

    def server_close(self):
        self.logger.debug('server_close')
        return SocketServer.TCPServer.server_close(self)

    def finish_request(self, request, client_address):
        self.logger.debug('finish_request(%s, %s)', request, client_address)
        return SocketServer.TCPServer.finish_request(self, request, client_address)

    def close_request(self, request_address):
        self.logger.debug('close_request(%s)', request_address)
        return SocketServer.TCPServer.close_request(self, request_address)

if __name__ == '__main__':
    import threading
    forever = threading.Event()

    def signal_handler(signal, frame):
        print("Exiting")
        forever.set()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)


    address = ('192.168.7.2', 25560)
    server = SharpServer(address)
    ip, port = server.server_address

    t = threading.Thread(target=server.serve_forever)
    t.setDaemon(True)
    t.start()

    forever.wait()