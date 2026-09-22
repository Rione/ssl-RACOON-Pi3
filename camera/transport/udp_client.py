"""UDP client that publishes detection JSON to the Go receiver."""

import socket

from camera import debug
from camera.settings import UDP_CAMERA_PORT


class UDPClient:
    def __init__(self, host="127.0.0.1", port=UDP_CAMERA_PORT):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        debug.log(f"UDP Client initialized for {self.host}:{self.port}")

    def send(self, data_json_string):
        if not data_json_string:
            debug.log("UDP Send Error: No data to send.")
            return False
        try:
            self.socket.sendto(data_json_string.encode("utf-8"), (self.host, self.port))
            return True
        except socket.error as e:
            debug.log(f"UDP Send Error: {e}")
            return False
        except Exception as e:
            debug.log(f"An unexpected error occurred during UDP send: {e}")
            return False

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None
            debug.log("UDP Client socket closed.")
