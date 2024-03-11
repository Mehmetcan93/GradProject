
# TCP-Serial Bridge

This project provides a two-way communication bridge between a virtual serial port and a physical serial port over a TCP/IP network. It consists of two main scripts: a server script and a client script.

## Virtual Serial Tools

Before you proceed, if you're using the server script to connect to a virtual serial port, you'll need tools to create such ports:

### Windows:
- **com0com**: A free and open-source tool that allows you to create virtual serial port pairs. [Download and installation guide](http://com0com.sourceforge.net/)

### Linux:
- **socat**: A versatile tool that establishes two bidirectional byte streams and transfers data between them. Install it using your package manager, for example:
```
sudo apt-get install socat
```
Then, to create a virtual serial port pair:
```
socat PTY,link=/tmp/virtualcom0 PTY,link=/tmp/virtualcom1
```

### macOS:
- **socat**: The same tool used in Linux is available for macOS. You can install it using Homebrew:
```
brew install socat
```
Then, create virtual serial port pairs similarly to the Linux method mentioned above.

## Server (server_serial-tcp.py)

The server script creates a TCP server that listens for incoming connections. It reads data from a virtual serial port and forwards it to a connected client over TCP. Similarly, it can receive data from a connected client and write it to the virtual serial port.

### Usage:

```
python3 server_serial-tcp.py -s <virtual_serial_port> -b <baud_rate>
```

**Arguments:**
- `-s` or `--serial-port`: The virtual serial port (e.g., COM11 or /tmp/virtualcom1).
- `-b` or `--baud-rate`: Baud rate for the virtual serial port (e.g., 115200).

**Example:**

```
python3 server_serial-tcp.py -s COM11 -b 115200
```

**Note:** Ensure the port on your router is correctly forwarded if accessing over the public internet. The default TCP port in the script is set to 2101.

## Client (client_tcp-serial.py)

The client script connects to the TCP server. It establishes a connection between an actual physical serial port and the server. This allows for data to be relayed between the physical serial port and the virtual serial port (through the server).

### Usage:

```
python3 client_tcp-serial.py -i <server_ip> -s <physical_serial_port> -b <baud_rate>
```

**Arguments:**
- `-i` or `--server-ip`: IP address of the TCP server (e.g., 192.168.1.77).
- `-s` or `--serial-port`: Physical serial port for the device (e.g., COM3).
- `-b` or `--baud-rate`: Baud rate for the physical serial port (e.g., 115200).

**Example:**

```
python3 client_tcp-serial.py -i 192.168.1.77 -s COM3 -b 115200
```

## Notes

- Ensure you have the required dependencies installed, notably `pyserial`.
- Exposing a server to the public internet carries inherent risks. Ensure you have adequate security measures in place.
