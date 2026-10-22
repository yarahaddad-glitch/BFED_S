# """
# PLC_stop.py - read-only link from the FX3U PLC to the E-stop.

# The PLC program latches the e-stop and drives the tower light; we follow it:

#     RED    (Y1 / coil 1)  e-stop button pressed      -> stop   (all robots freeze)
#     YELLOW (Y2 / coil 2)  button twisted out         -> home   (robots glide home)
#     GREEN  (Y3 / coil 3)  small reset button pressed -> resume (task carries on)

# X14 (the e-stop input) is also watched: while it reads pressed, robots stay stopped.

# PRINT_IO = True prints the inputs/outputs (same format as fx3u_read_io.py)
# every time any of them changes, so you can see exactly what the PLC reports.

# If the COM port given doesn't answer, every other COM port is tried
# automatically and the one that answers is used.

# Only Modbus functions 1 and 2 (READ) are used. Nothing is written to the PLC.

# Test on its own (no simulation):   python PLC_stop.py
# """

# import struct
# import threading
# import time

# try:
#     import serial
#     import serial.tools.list_ports
# except ImportError:          # pyserial missing
#     serial = None

# # ---------------- settings ----------------
# COM_PORT = "COM3"            # first port to try ("AUTO" = search every port)
# AUTO_FIND_PORT = True        # if COM_PORT doesn't answer, try every other COM port
# SLAVE_ID = 1
# BAUD = 57600
# POLL_PERIOD = 0.1            # seconds between PLC reads
# RETRY_PERIOD = 2.0           # seconds between reconnect attempts

# PRINT_IO = True              # print Inputs/Outputs whenever anything changes
# HEARTBEAT_SECONDS = 1.0      # also print a one-line status this often (0 = off)
# VERSION = "PLC_stop v3 (prints I/O)"
# ESTOP_INPUT = 12             # X14 -> Modbus discrete input 12 (labels skip 8 and 9)
# ESTOP_PRESSED_VALUE = True   # X14 read 1 while pressed in your photo; flip if backwards
# USE_ESTOP_INPUT = True       # also watch X14 directly (not just the red light)
# STOP_ON_LINK_LOSS = True     # cable pulled / PLC off after connecting -> stop the robots

# RED_COIL, YELLOW_COIL, GREEN_COIL = 1, 2, 3

# INPUT_NAMES = tuple([f"X{i}" for i in range(8)] + [f"X1{i}" for i in range(6)])
# OUTPUT_NAMES = tuple([f"Y{i}" for i in range(8)] + ["Y10", "Y11"])


# # ---------------- Modbus RTU (read-only, same as fx3u_read_io.py) ----------------
# def crc16(data):
#     crc = 0xFFFF
#     for byte in data:
#         crc ^= byte
#         for _ in range(8):
#             crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
#     return crc


# def read_exact(port, length):
#     received = bytearray()
#     while len(received) < length:
#         chunk = port.read(length - len(received))
#         if not chunk:
#             break
#         received.extend(chunk)
#     if len(received) != length:
#         raise RuntimeError(f"Timeout: received {len(received)} of {length} expected bytes")
#     return bytes(received)


# def read_bits(port, function, quantity):
#     """function 1 = read coils (outputs), function 2 = read discrete inputs."""
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
#     return tuple(bool(packed[i // 8] & (1 << (i % 8))) for i in range(quantity))


# def open_port(name):
#     return serial.Serial(port=name, baudrate=BAUD, bytesize=serial.EIGHTBITS,
#                          parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.5)


# def read_io(port):
#     inputs = read_bits(port, function=2, quantity=14)
#     outputs = read_bits(port, function=1, quantity=10)
#     return inputs, outputs


# def format_io(inputs, outputs):
#     return ("Inputs : " + "  ".join(f"{n}={int(v)}" for n, v in zip(INPUT_NAMES, inputs)) + "\n"
#             + "Outputs: " + "  ".join(f"{n}={int(v)}" for n, v in zip(OUTPUT_NAMES, outputs)))


# def print_io(inputs, outputs):
#     """Print the PLC inputs/outputs exactly like fx3u_read_io.py."""
#     print(format_io(inputs, outputs), flush=True)


# def candidate_ports(preferred):
#     ports = []
#     if preferred and str(preferred).upper() != "AUTO":
#         ports.append(preferred)
#     if AUTO_FIND_PORT or not ports:
#         try:
#             for p in serial.tools.list_ports.comports():
#                 if p.device not in ports:
#                     ports.append(p.device)
#         except Exception:
#             pass
#     return ports


# def find_plc(preferred=COM_PORT):
#     """Open the first COM port where the PLC answers a Modbus read. Returns (serial, name) or (None, errors)."""
#     errors = []
#     for name in candidate_ports(preferred):
#         try:
#             port = open_port(name)
#         except Exception as exc:
#             errors.append(f"{name}: {exc}")
#             continue
#         try:
#             read_io(port)
#             return port, name
#         except Exception as exc:
#             errors.append(f"{name}: {exc}")
#             port.close()
#     return None, errors


# def light_from(outputs):
#     """Which tower light is on (red wins if more than one)."""
#     if outputs[RED_COIL]:
#         return "RED"
#     if outputs[YELLOW_COIL]:
#         return "YELLOW"
#     if outputs[GREEN_COIL]:
#         return "GREEN"
#     return None


# def read_once(port_name=COM_PORT):
#     """One-off read + print, like running fx3u_read_io.py. Returns (inputs, outputs)."""
#     port, name = find_plc(port_name)
#     if port is None:
#         print(f"[PLC] no answer on any port: {name}", flush=True)
#         return None
#     with port:
#         inputs, outputs = read_io(port)
#     print(f"[PLC] {name}", flush=True)
#     print_io(inputs, outputs)
#     return inputs, outputs


# # ---------------- the monitor ----------------
# class PLCEStopMonitor:
#     def __init__(self, stop, home, resume, is_active=None, log=None, port=COM_PORT):
#         self.stop_fn, self.home_fn, self.resume_fn = stop, home, resume
#         self.is_active = is_active or (lambda: False)
#         self.log_fn = log
#         self.port = port
#         self.light = None            # last light colour seen
#         self.button_pressed = False  # last X14 state
#         self.connected = False
#         self.inputs = None           # last raw readings (for anyone who wants them)
#         self.outputs = None
#         self._closed = threading.Event()
#         self._thread = None

#     def start(self):
#         if serial is None:
#             self._log("[PLC] pyserial not installed (python -m pip install pyserial==3.5) - PLC E-stop DISABLED")
#             return
#         self._log(f"[PLC] {VERSION} loaded - searching for the PLC (trying {self.port} first)...")
#         self._thread = threading.Thread(target=self._run, name="PLC E-stop", daemon=True)
#         self._thread.start()

#     def close(self):
#         self._closed.set()

#     def _log(self, msg):
#         print(msg, flush=True)
#         if self.log_fn:
#             try:
#                 self.log_fn(msg)
#             except Exception:
#                 pass

#     def _stop(self, msg):
#         if not self.is_active():
#             self.stop_fn(msg)

#     def _run(self):
#         warned = False
#         while not self._closed.is_set():
#             port, name = find_plc(self.port)
#             if port is None:
#                 if self.connected:
#                     self._log("[PLC] link lost")
#                     if STOP_ON_LINK_LOSS:
#                         self._stop("PLC link lost")
#                     self.connected = False
#                 if not warned:
#                     self._log("[PLC] !!! PLC NOT FOUND - the physical e-stop will NOT work. Tried: "
#                               + "; ".join(name or ["no COM ports found"]))
#                     self._log("[PLC]     Close fx3u_read_io.py / other serial programs. Retrying every "
#                               f"{RETRY_PERIOD:.0f}s...")
#                     warned = True
#                 self._closed.wait(RETRY_PERIOD)
#                 continue

#             self.port = name                     # remember the port that worked
#             warned = False
#             try:
#                 with port:
#                     first = True
#                     last_beat = 0.0
#                     while not self._closed.is_set():
#                         inputs, outputs = read_io(port)
#                         if first:
#                             self.connected = True
#                             self._log(f"[PLC] CONNECTED on {name} - light is {light_from(outputs)}")
#                         changed = first or inputs != self.inputs or outputs != self.outputs
#                         if PRINT_IO and changed:
#                             print("[PLC] CHANGE:", flush=True)
#                             print_io(inputs, outputs)
#                         now = time.time()
#                         if HEARTBEAT_SECONDS and now - last_beat >= HEARTBEAT_SECONDS:
#                             last_beat = now
#                             print(f"[PLC] reading OK  X13(reset)={int(inputs[11])}  "
#                                   f"X14(e-stop)={int(inputs[ESTOP_INPUT])}  "
#                                   f"Y1(red)={int(outputs[RED_COIL])}  Y2(yellow)={int(outputs[YELLOW_COIL])}  "
#                                   f"Y3(green)={int(outputs[GREEN_COIL])}  -> light {light_from(outputs)}  "
#                                   f"| robots stopped: {self.is_active()}", flush=True)
#                         self.inputs, self.outputs = inputs, outputs
#                         self._update(inputs, outputs, first)
#                         first = False
#                         self._closed.wait(POLL_PERIOD)
#             except Exception as exc:
#                 if self._closed.is_set():
#                     break
#                 self._log(f"[PLC] link lost on {name}: {exc}")
#                 if self.connected and STOP_ON_LINK_LOSS:
#                     self._stop("PLC link lost")
#                 self.connected = False
#                 self._closed.wait(RETRY_PERIOD)

#     def _update(self, inputs, outputs, first):
#         pressed = USE_ESTOP_INPUT and (inputs[ESTOP_INPUT] == ESTOP_PRESSED_VALUE)
#         light = light_from(outputs)

#         if pressed and not self.button_pressed:
#             self._log("[PLC] X14 e-stop button PRESSED")
#         if pressed:
#             self._stop("PLC: E-stop button pressed")
#         self.button_pressed = pressed

#         if first:
#             self.light = light
#             if light == "RED":
#                 self._stop("PLC: E-stop latched (red light) at start-up")
#             return

#         if light is None or light == self.light:
#             return
#         previous, self.light = self.light, light
#         self._log(f"[PLC] light {previous} -> {light}")

#         if light == "RED":
#             self._stop("PLC: E-stop (red light)")
#         elif light == "YELLOW":
#             self.home_fn()
#         elif light == "GREEN" and not pressed:
#             self.resume_fn()


# if __name__ == "__main__":
#     # Stand-alone test (no simulation): prints the I/O on every change
#     # and what the robots WOULD do.
#     mon = PLCEStopMonitor(stop=lambda m="": print("   -> ROBOTS WOULD STOP:", m),
#                           home=lambda: print("   -> ROBOTS WOULD GO HOME"),
#                           resume=lambda: print("   -> ROBOTS WOULD RESUME"))
#     mon.start()
#     print("Press the e-stop, twist it out, press reset. Ctrl+C to quit.", flush=True)
#     try:
#         while True:
#             time.sleep(1)
#     except KeyboardInterrupt:
#         mon.close()