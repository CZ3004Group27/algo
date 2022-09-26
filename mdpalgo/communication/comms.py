"""
Testing Algorithm Socket Only - Not used in production.
"""
import socket
import struct
# from misc.config import FORMAT, ALGO_SOCKET_BUFFER_SIZE, WIFI_IP, PORT

# Wifi IP: RPI's ip address -> 192.168.12.12
# Port: 5050

FORMAT = "UTF-8"
ALGO_SOCKET_BUFFER_SIZE = 1024

RPI_IP = "192.168.27.27"
TEST_IP = "127.0.0.1" # Use this for easier testing RPi integration without RPi

WIFI_IP = RPI_IP
PORT = 25565


class AlgoClient:

    def __init__(self, server_ip=WIFI_IP, server_port=PORT) -> None:
        print("[Algo Client] Initialising Algo Client.")
        self.client_socket = None
        self.server_address = (server_ip, server_port)
        print("[Algo Client] Client has been initilised.")

    def connect(self) -> bool:
        try:
            # Connect to RPI via TCP
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((WIFI_IP, PORT))
            print(f"[Algo Client] Client successfully connected to Server at {self.server_address}.")
            return True
        except Exception as error:
            print(f'[Algo] Failed to connect to Algorithm Server at {self.server_address}')
            print(f"[Error Message]: {error}")
            return False

    def disconnect(self) -> bool:
        try:
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None
            return True
        except Exception as error:
            print(f'[Algo] Failed to disconnect from Algorithm Server at {self.server_address}')
            print(f"[Error Message]: {error}")
            return False

    def recv(self) -> str:
        try:
            # Decode : Converting from Byte to UTF-8 format.
            raw_bytes = self.receive_message_with_size()
            if raw_bytes is not None:
                message = self.decode(raw_bytes)
                print(f'[Algo] Received Message from Algo Server: {message}')
                return message
            return None
        except Exception as error:
            print("[Algo] Failed to receive message from Algo Server.")
            print(f"[Error Message]: {error}")
            raise error

    def send(self, message):
        try:
            print(f'[Algo] Message to Algo Server: {message}')
            self.send_message_with_size(self.encode(message))

        except Exception as error:
            print("[Algo] Failed to send to Algo Server.")
            print(f"[Error Message]: {error}")
            raise error

    def encode(self, message: str) -> bytes:
        """Encode a message to a byte array"""
        return message.encode(FORMAT)

    def decode(self, raw_bytes: bytes) -> str:
        """Decode a byte array to a string message"""
        return raw_bytes.strip().decode(FORMAT)

    def send_message_with_size(self, data: bytes):
        """Send (size, data) where size is size in bytes of data"""
        number_of_bytes = len(data)
        packet_length = struct.pack("!I", number_of_bytes)
        packet_length += data
        self.client_socket.sendall(packet_length)

    def receive_message_with_size(self):
        """Receive the raw bytes from the message"""
        try:
            data = self.client_socket.recv(4) # read the first 4 bytes (data length)
            if len(data) == 0:
                return None
            else:
                number_of_bytes = struct.unpack("!I", data)[0] # convert 4 bytes into integer
                received_packets = b''
                bytes_to_receive = number_of_bytes
                while len(received_packets) < number_of_bytes:
                    packet = self.client_socket.recv(bytes_to_receive)
                    bytes_to_receive -= len(packet)
                    received_packets += packet
                return received_packets
        except socket.timeout:
            return None
        except:
            return None


# Standalone testing.
if __name__ == '__main__':
    WIFI_IP = TEST_IP # use the testing IP
    client = AlgoClient()
    connect_status = client.connect()
    assert (connect_status) # if the server is up, this should be true

    # import the libraries for parsing messages
    from mdpalgo.communication.message_parser import MessageParser, MessageType
    parser = MessageParser()

    while True:
        message = input("[Client] Input message to send to server: ")
        client.send(message)
        received = client.recv()

        # test the photo data
        message_data = parser.parse(received)
        if message_data["type"] == MessageType.IMAGE_TAKEN:
            import cv2
            img = message_data["data"]["image"]
            cv2.imwrite("catimage.jpg", img)
            cv2.imshow("preview", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
