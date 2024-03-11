from RTK_client import process_serial_data
import serial, json, argparse

def my_callback(parsed_data):
    print(json.dumps(parsed_data, indent=4))



# Parse command-line arguments
parser = argparse.ArgumentParser(description='Read data from a serial device and publish to ROS topics.')
parser.add_argument('-s', '--serial-port', required=False, help='Serial port for the device (e.g. /dev/ttyUSB0)', default='/dev/ttyUSB0')
parser.add_argument('-b', '--baud-rate', type=int, required=False, help='Baud rate for the serial port (e.g. 115200)', default=115200)

args = parser.parse_args()

ser = serial.Serial(args.serial_port, baudrate=args.baud_rate)
process_serial_data(ser, my_callback)
