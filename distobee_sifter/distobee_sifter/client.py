import socket
from distobee_sifter.sifter_motors_command import SifterMotorsCommand

class Client:
    def __init__(self, host, port=6000):
        self.host = host
        self.port = port

    def send_command(self, command: SifterMotorsCommand):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((self.host, self.port))
            json_data = command.to_json()
            print(f"Sending: {json_data}")
            sock.sendall(json_data.encode())
            response = sock.recv(1024)
            print(f"Received: {response.decode()}")


if __name__ == "__main__":
    client = Client()
    command = SifterMotorsCommand(brush_pwm=50, vibration=False)
    client.send_command(command)
