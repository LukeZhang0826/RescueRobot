# pixy.py
import utime
from machine import SPI, Pin

RESP_SYNC = b"\xAF\xC1"

def u16le(b0, b1):
    return b0 | (b1 << 8)

class PixySPI:
    def __init__(self, spi_id=0, cs_pin=21, sck=18, mosi=19, miso=20,
                 baudrate=500_000, read_len=256):
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.spi = SPI(
            spi_id, baudrate=baudrate,
            polarity=1, phase=1,
            bits=8, firstbit=SPI.MSB,
            sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso),
        )
        self.read_len = read_len

    def _req_with_checksum(self, ptype, payload):
        csum = sum(payload) & 0xFFFF
        return bytes([0xAE, 0xC1, ptype & 0xFF, len(payload) & 0xFF, csum & 0xFF, (csum >> 8) & 0xFF]) + payload

    def _send_and_read(self, req):
        self.cs.value(0)
        utime.sleep_us(50)
        self.spi.write(req)
        buf = bytearray(self.read_len)
        self.spi.readinto(buf, 0x00)
        self.cs.value(1)
        return bytes(buf)

    def _parse_response(self, data):
        i = data.rfind(RESP_SYNC)
        if i < 0:
            return None, b""
        ptype = data[i + 2]
        length = data[i + 3]
        csum = u16le(data[i + 4], data[i + 5])
        payload = data[i + 6 : i + 6 + length]
        if (sum(payload) & 0xFFFF) != csum:
            return None, b""
        return ptype, payload

    def get_blocks(self, sigmap=0xFF, max_blocks=10):
        req = self._req_with_checksum(32, bytes([sigmap & 0xFF, max_blocks & 0xFF]))
        ptype, pl = self._parse_response(self._send_and_read(req))
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
            blocks.append({"sig": sig, "x": x, "y": y, "w": w, "h": h, "area": w*h})
        return blocks

    def best_block(self, area_min=2000, sigmap=0xFF):
        blocks = self.get_blocks(sigmap=sigmap, max_blocks=10)
        blocks = [b for b in blocks if b["area"] >= area_min]
        if not blocks:
            return None
        return max(blocks, key=lambda bb: bb["area"])