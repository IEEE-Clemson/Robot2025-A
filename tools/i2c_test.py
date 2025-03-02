import time
import smbus2
import struct
bus = smbus2.SMBus(1)
V_MAX = 2.0
O_MAX = 8.0
INT16_MAX = 1 << 15
vx = 0.00
vy = -0.0
omega = 0
vx_raw = int(vx * INT16_MAX / V_MAX)
vy_raw = int(vy * INT16_MAX / V_MAX)
omega_raw = int(omega * INT16_MAX / O_MAX)


data = struct.pack("<hhh", vx_raw, vy_raw, omega_raw)
bus.write_i2c_block_data(0x41, 0, data)
#print(bus.read_byte_data(0x41, 200))
#print(bus.read_i2c_block_data(0x41, 200, 4))
start = time.time()
n = 1000
for i in range(n):
    data = struct.unpack('<hhh', bytes(bus.read_i2c_block_data(0x41, 6, 6)))
    print(data)
    vx_actual = data[0] * V_MAX / INT16_MAX
    vy_actual = data[1] * V_MAX / INT16_MAX
    o_actual = data[2] * O_MAX / INT16_MAX
    print(f"({vx_actual}, {vy_actual}, {o_actual})")
    time.sleep(0.5)
print(bus.read_i2c_block_data(0x41, 6, 6))
dt = time.time() - start
print(dt)
print(dt / n)
