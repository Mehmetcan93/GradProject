import serial
import struct

data_fields = {
    0x0001: ("Acceleration(XYZ)", 12, "fff"),
    0x0002: ("Gyro(XYZ)", 12, "fff"),
    0x0004: ("Magnetometer(XYZ)", 12, "fff"),
    0x0008: ("Latitude-Longitude-Elevation", 12, "fff"),
    0x0010: ("Roll-Pitch-Yaw", 12, "fff"),
    0x0020: ("Pressure-Temperature-BaroHeight", 12, "fff"),
    0x0040: ("Time", 4, "f"),
    0x0080: ("Status", 1, "B"),
    0x0100: ("Velocity", 12, "fff"),
    0x0200: ("Scaled-LLh", 12, "iii"),
    0x1000: ("Low-Rate-Messages", 12, "")
}
def bytes_to_int(byte_array):
    padded_byte_array = b'\x00' + byte_array
    return struct.unpack('>I', padded_byte_array)[0]


def bytearray_to_hex(byte_array):
    if isinstance(byte_array, str):

        return ''.join('{:02x}'.format(ord(char)) for char in byte_array)
    elif not isinstance(byte_array, bytearray):

        raise ValueError("Invalid input type. Expected 'str' or 'bytearray'.")


    return ''.join('{:02x}'.format(byte) for byte in byte_array)


def process_serial_data(ser, callback):

    while True:


        # Buffer to hold the current message
        message = bytearray()

        # Read until we find a preamble
        byte = ser.read(1)

        while byte != b'\x3D':
            byte = ser.read(1)


        # We found a preamble, so start a new message
        message += byte


        # Read Message ID (3 bytes)
        for _ in range(3):
            message += ser.read(1)

        # Ignore the last byte of Message ID, as it's set to 0
        message_id = bytes_to_int(message[1:4])

        data = {
            'Preamble': bytearray_to_hex(message[:1]),
            'Message ID': bytearray_to_hex(message[1:4]),
        }

        # Parse data fields according to message_id
        data_start = 4
        for bit_position, (field_name, field_length, field_format) in data_fields.items():
            if message_id & bit_position:
                data_end = data_start + field_length
                field_data = ser.read(field_length)
                message += field_data


                # Unpack the data according to the provided format
                if field_format:
                    field_data_hex = bytearray_to_hex(field_data)
                    unpacked_data = struct.unpack('<' + field_format, field_data_hex.decode('hex'))
                else:

                    field_data_hex = bytearray_to_hex(field_data)
                    unpacked_data = field_data_hex

                data[field_name] = unpacked_data

                data_start = data_end

        # Read CRC (2 bytes)
        for _ in range(2):
            message += ser.read(1)

        data['CRC'] = bytearray_to_hex(message[-2:])
        #print(data)

        # Call the callback with the parsed data
        callback(data)
