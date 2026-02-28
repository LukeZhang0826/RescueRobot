from machine import SPI, Pin
import time

# Pixy2 SPI response sync word (0xC1AF on wire => AF C1 in bytes)
RESP_SYNC = b"\xAF\xC1"

# Pixy2 CCC frame is typically 316x208
FRAME_W = 316
FRAME_CENTER_X = FRAME_W // 2  # 158

def u16le(b0, b1):
    return b0 | (b1 << 8)

class PixySPI:
    """
    Minimal Pixy2 (CCC) SPI reader for MicroPython.
    Returns blocks as dicts: {sig, x, y, w, h, index, age, err_x}
    """
    def __init__(
        self,
        spi_id=0,
        cs_pin=21,
        sck=18,
        mosi=19,
        miso=20,
        baudrate=500_000,
        read_len=256,
        debug=False,
    ):
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.spi = SPI(
            spi_id,
            baudrate=baudrate,
            polarity=1, phase=1,          # mode 3
            bits=8, firstbit=SPI.MSB,
            sck=Pin(sck),
            mosi=Pin(mosi),
            miso=Pin(miso),
        )
        self.read_len = read_len
        self.debug = debug

    def _req_with_checksum(self, ptype, payload):
        csum = sum(payload) & 0xFFFF
        return bytes([
            0xAE, 0xC1,                  # request sync
            ptype & 0xFF,
            len(payload) & 0xFF,
            csum & 0xFF, (csum >> 8) & 0xFF
        ]) + payload

    def _send_and_read(self, req):
        self.cs.value(0)
        time.sleep_us(50)
        self.spi.write(req)
        buf = bytearray(self.read_len)
        self.spi.readinto(buf, 0x00)
        self.cs.value(1)
        return bytes(buf)

    def _parse_response(self, data):
        # Use last sync in buffer (more robust than find)
        i = data.rfind(RESP_SYNC)
        if i < 0:
            raise Exception("No response sync")

        ptype = data[i + 2]
        length = data[i + 3]
        csum = u16le(data[i + 4], data[i + 5])
        payload = data[i + 6 : i + 6 + length]

        if (sum(payload) & 0xFFFF) != csum:
            raise Exception("Checksum mismatch")

        return ptype, payload

    def get_blocks(self, sigmap=0xFF, max_blocks=10, center_x=FRAME_CENTER_X):
        # GET_BLOCKS: type=32, resp=33
        req = self._req_with_checksum(32, bytes([sigmap & 0xFF, max_blocks & 0xFF]))
        ptype, pl = self._parse_response(self._send_and_read(req))

        if self.debug:
            print("ptype=%d payload_len=%d" % (ptype, len(pl)))

        if ptype != 33:
            return []

        blocks = []
        stride = 14
        n = len(pl) - (len(pl) % stride)

        for off in range(0, n, stride):
            sig = u16le(pl[off + 0],  pl[off + 1])
            x   = u16le(pl[off + 2],  pl[off + 3])
            y   = u16le(pl[off + 4],  pl[off + 5])
            w   = u16le(pl[off + 6],  pl[off + 7])
            h   = u16le(pl[off + 8],  pl[off + 9])
            idx = u16le(pl[off + 10], pl[off + 11])
            age = u16le(pl[off + 12], pl[off + 13])

            blocks.append({
                "sig": sig,
                "x": x, "y": y, "w": w, "h": h,
                "index": idx,
                "age": age,
                "err_x": x - center_x,     # horizontal error (pixels)
            })

        return blocks


# ---- main loop ----

pixy = PixySPI(debug=False)

SIG_ALL = 0xFF
# If you want only signature 1, try SIG1 = 0x01 (as you had working before)
SIG1 = 0x01

while True:
    try:
        blocks = pixy.get_blocks(sigmap=SIG_ALL, max_blocks=10)

        if not blocks:
            print("no blocks")
        else:
            b = blocks[0]
            # one clean line you can parse/log
            print("sig=%d x=%d y=%d w=%d h=%d err_x=%d" %
                  (b["sig"], b["x"], b["y"], b["w"], b["h"], b["err_x"]))

    except Exception as e:
        print("ERR:", e)

    time.sleep_ms(200)