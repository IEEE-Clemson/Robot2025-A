import time
import smbus2
import struct
bus = smbus2.SMBus(1)

#print(bus.read_byte_data(0x41, 200))
#print(bus.read_i2c_block_data(0x41, 200, 4))
start = time.time()
n = 1000
for i in range(n):
    bus.write_i2c_block_data(0x41, 0, [1, 2, 3, 4, 5, 6])
    data = struct.unpack('<hhhh', bytes(bus.read_i2c_block_data(0x41, 6, 8)))
    print(data)
    time.sleep(0.5);
print(bus.read_i2c_block_data(0x41, 6, 6))
dt = time.time() - start
print(dt)
print(dt / n)
