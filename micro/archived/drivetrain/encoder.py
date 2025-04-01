from encoder_pio import pio_encoder
from machine import Pin
from rp2 import PIO, StateMachine


class Encoder:
    """Tracks a quadrature encoder by utilizing the picos PIO
    Currently, the class is configured to only run on PIO 0
    """

    def __init__(self, sm_id, pin_a: int, pin_b: int) -> None:
        """Initializes encoder data and PIO for tracking encoder

        Args:
            sm_id (_type_): id from 0 to 3 of the state machine, one encoder per sm
            pin_a (int): pin for the A channel of the quadrature
            pin_b (int): pin for the B channel of the quadrature
        """

        # Due to the way the PIO works, the lower pin is always seen as the A channel
        # If the lower pin is actually the B channel, it will invert the quadrature
        self._pin_swapped = pin_a > pin_b
        self._low_pin_id = min(pin_a, pin_b)
        self._high_pin_id = max(pin_a, pin_b)

        # Pins should only have a difference in position of 1
        assert (self._high_pin_id - self._low_pin_id) == 1
        # From here on out, assume everything is running on PIO 0
        assert sm_id < 4

        self._low_pin = Pin(self._low_pin_id, Pin.ALT_PIO0, pull=Pin.PULL_UP)
        self._high_pin = Pin(self._high_pin_id, Pin.ALT_PIO0, pull=Pin.PULL_UP)

        # TODO: Should PIO be running at full AHB clock?
        # We have been using hall effect encoders so we haven't seen any switch bounce
        self._sm = StateMachine(
            sm_id,
            pio_encoder,
            in_base=self._low_pin,
            in_shiftdir=PIO.SHIFT_LEFT,
            out_shiftdir=PIO.SHIFT_RIGHT,

        )
        self._sm.restart()
        self._sm.active(1)

    def get_count(self) -> int:
        """Gets raw encoder pulse count

        Returns:
            int: Encoder pulse count
        """
        count_raw = self._get_count_raw()
        
        # convert unsigned to signed
        if(count_raw & 0x80000000):
            count_raw = -0x100000000 + count_raw

        if self._pin_swapped:
            return -count_raw
        else:
            return count_raw

    def set_count(self, count: int):
        """Sets the encoder pulse count

        Args:
            count (int): encoder pulse count
        """
        if self._pin_swapped:
            self._set_count_raw(count)
        else:
            self._set_count_raw(count)

    def _get_count_raw(self) -> int:
        # HACK
        # PIO program repeatedly fills FIFO with encoder counts, regardless of whether its full
        # To read, clear the FIFO and wait for a value

        # Clear RX and TX FIFO
        # Micropython does not have method to clear properly,
        # so read 4x32 values then return the fifth
        for _ in range(4):
            self._sm.get()

        # Wait for and read first value from RX FIFO
        return self._sm.get()

    def _set_count_raw(self, count: int):
        self._sm.put(count)
