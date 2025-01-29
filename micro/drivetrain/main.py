from encoder import Encoder
import utime

encoder = Encoder(0, 2, 3)

while True:
    utime.sleep(1)
    print(encoder.get_count())

    print()