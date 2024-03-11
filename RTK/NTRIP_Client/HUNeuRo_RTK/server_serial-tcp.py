import serial
import serial.tools.list_ports
import socket
import threading
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='provides communication between two serial ports over TCP/IP network (Server Side)')
parser.add_argument('-b', '--baud-rate', type=int, required=True, help='Baud rate for the serial port (e.g. 115200)')

args = parser.parse_args()

# Parameters for the virtual serial port
BAUD_RATE = args.baud_rate
DATA_BITS = 8
PARITY = 'N'
STOP_BITS = 1

# Find available serial ports
ports = serial.tools.list_ports.comports()
print("Available serial ports:")
for port, desc, hwid in sorted(ports):
    print(f"{port}: {desc}")

# Prompt user to select the serial port
serial_port = input("Enter the serial port for the device (e.g. /dev/ttyUSB0): ")

# Parameters for the TCP server
TCP_IP = '0.0.0.0'
TCP_PORT = 2101
BUFFER_SIZE = 4096

# Create and configure the virtual serial port
ser = serial.Serial(
    port=serial_port,
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
    # Create the TCP server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print(f"Listening for incoming connections on {TCP_IP}:{TCP_PORT}")

    conn, addr = s.accept()
    print(f"Connection from: {addr}")

    # Start the data transfer between the two interfaces
    t1 = threading.Thread(target=serial_to_tcp, args=(conn,))
    t2 = threading.Thread(target=tcp_to_serial, args=(conn,))
    
    t1.daemon = True
    t2.daemon = True

    t1.start()
    t2.start()

    while True:
        pass

except KeyboardInterrupt:
    print("\nShutting down server...")
    s.close()
    ser.close()

