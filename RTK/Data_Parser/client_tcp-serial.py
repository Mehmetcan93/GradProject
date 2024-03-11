import serial
import socket
import threading
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='provides communication between two serial ports over TCP/IP network (Client Side)')
parser.add_argument('-i', '--server-ip', required=True, help='IP of the TCP server (e.g. 192.168.1.77)')
parser.add_argument('-s', '--serial-port', required=True, help='Serial port for the device (e.g. /dev/ttyUSB0)')
parser.add_argument('-b', '--baud-rate', type=int, required=True, help='Baud rate for the serial port (e.g. 115200)')

args = parser.parse_args()

# Parameters for the physical serial port
SERIAL_PORT = args.serial_port  # This should be your actual serial port
BAUD_RATE = args.baud_rate
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1

# Parameters for the TCP client
TCP_IP = args.server_ip
TCP_PORT = 2101
BUFFER_SIZE = 4096

# Create and configure the physical serial port
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUD_RATE,
    bytesize=DATA_BITS,
    parity=PARITY,
    stopbits=STOP_BITS,
    timeout=1
)

# Function to handle data transfer from serial port to TCP
def serial_to_tcp(tcp_socket):
    while True:
        data = ser.read(BUFFER_SIZE)
        if data:
            print("serial2tcp: ", data)
            tcp_socket.send(data)

# Function to handle data transfer from TCP to serial port
def tcp_to_serial(tcp_socket):
    while True:
        data = tcp_socket.recv(BUFFER_SIZE)
        if data:
            print("tcp2serial: ", data)
            ser.write(data)

try:
    # Create a TCP client connection to the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    print(f"Connected to server at {TCP_IP}:{TCP_PORT}")

    # Start the data transfer between the two interfaces
    t1 = threading.Thread(target=serial_to_tcp, args=(s,))
    t2 = threading.Thread(target=tcp_to_serial, args=(s,))
    
    t1.daemon = True
    t2.daemon = True

    t1.start()
    t2.start()

    while True:
        pass

except KeyboardInterrupt:
    print("\nDisconnecting from server...")
    s.close()
    ser.close()
