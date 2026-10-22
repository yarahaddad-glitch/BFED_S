# import struct
# import serial


# COM_PORT = "COM3"
# SLAVE_ID = 1

# # Mitsubishi labels skip 8 and 9: X7 is followed by X10.
# INPUT_NAMES = tuple([f"X{i}" for i in range(8)] + [f"X1{i}" for i in range(6)])
# OUTPUT_NAMES = tuple([f"Y{i}" for i in range(8)] + ["Y10", "Y11"])


# def crc16(data):
#     """Calculate the CRC-16 used by Modbus RTU."""
#     crc = 0xFFFF
#     for byte in data:
#         crc ^= byte
#         for _ in range(8):
#             if crc & 1:
#                 crc = (crc >> 1) ^ 0xA001
#             else:
#                 crc >>= 1
#     return crc


# def read_exact(port, length):
#     """Read the requested number of bytes or report a timeout."""
#     received = bytearray()
#     while len(received) < length:
#         chunk = port.read(length - len(received))
#         if not chunk:
#             break
#         received.extend(chunk)
#     if len(received) != length:
#         raise RuntimeError(
#             f"Timeout: received {len(received)} of {length} expected bytes"
#         )
#     return bytes(received)


# def read_bits(port, function, quantity):
#     """Read coils (function 1) or discrete inputs (function 2)."""
#     request = struct.pack(">BBHH", SLAVE_ID, function, 0, quantity)
#     request += struct.pack("<H", crc16(request))

#     port.reset_input_buffer()
#     port.write(request)
#     port.flush()

#     data_byte_count = (quantity + 7) // 8
#     response = read_exact(port, 3 + data_byte_count + 2)

#     if response[0] != SLAVE_ID or response[1] != function:
#         raise RuntimeError("Unexpected Modbus response")
#     if response[2] != data_byte_count:
#         raise RuntimeError("Unexpected number of data bytes")
#     if crc16(response[:-2]) != struct.unpack("<H", response[-2:])[0]:
#         raise RuntimeError("Modbus CRC check failed")

#     packed = response[3:-2]
#     return tuple(
#         bool(packed[index // 8] & (1 << (index % 8)))
#         for index in range(quantity)
#     )


# with serial.Serial(
#     port=COM_PORT,
#     baudrate=57600,
#     bytesize=serial.EIGHTBITS,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     timeout=0.5,
# ) as plc:
#     # Function 2 reads the 14 physical inputs X0-X7 and X10-X15.
#     inputs = read_bits(plc, function=2, quantity=14)

#     # Function 1 reads the 10 physical outputs Y0-Y7 and Y10-Y11.
#     outputs = read_bits(plc, function=1, quantity=10)


# print("Inputs : " + "  ".join(
#     f"{name}={int(value)}" for name, value in zip(INPUT_NAMES, inputs)
# ))
# print("Outputs: " + "  ".join(
#     f"{name}={int(value)}" for name, value in zip(OUTPUT_NAMES, outputs)
# ))