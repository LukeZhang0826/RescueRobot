# pixy.py
# Pixy2 camera driver (SPI, block mode)

import utime
from machine import SPI, Pin

RESP_SYNC = b"\xAF\xC1"


def _u16le(b0, b1):
    """Convert two bytes to unsigned 16-bit little-endian."""
    return b0 | (b1 << 8)


class PixySPI:
    """
    Pixy2 camera interface over SPI (CCC block mode).
    
    Returns detected color blocks with position and size.
    """

    def __init__(self, spi_id, cs_pin, sck_pin, mosi_pin, miso_pin,
                 baudrate=500_000, read_len=256):
        
        self.cs = Pin(cs_pin, Pin.OUT, value=1)
        self.spi = SPI(
            spi_id,
            baudrate=baudrate,
            polarity=1,
            phase=1,
            bits=8,
            firstbit=SPI.MSB,
            sck=Pin(sck_pin),
            mosi=Pin(mosi_pin),
            miso=Pin(miso_pin),
        )
        self.read_len = read_len

    def _build_req(self, ptype, payload=b""):
        return bytes([
            0xAE, 0xC1,
            ptype & 0xFF,
            len(payload) & 0xFF
        ]) + payload

    def _send_and_read(self, req):
        """Send request and read response."""
        self.cs.value(0)
        utime.sleep_us(50)
        self.spi.write(req)
        buf = bytearray(self.read_len)
        self.spi.readinto(buf, 0x00)
        self.cs.value(1)
        return bytes(buf)

    def _parse_response(self, data):
        """Parse response packet, return (type, payload)."""
        i = data.rfind(RESP_SYNC)
        if i < 0:
            return None, b""
        
        ptype = data[i + 2]
        length = data[i + 3]
        csum = _u16le(data[i + 4], data[i + 5])
        payload = data[i + 6 : i + 6 + length]
        
        if (sum(payload) & 0xFFFF) != csum:
            return None, b""
        
        return ptype, payload

    def get_blocks(self, sigmap=0xFF, max_blocks=10):
        payload = bytes([
            sigmap & 0xFF,
            max_blocks & 0xFF
        ])
        req = self._build_req(32, payload)
        ptype, payload = self._parse_response(self._send_and_read(req))

        if ptype != 33:
            return []

        blocks = []
        stride = 14
        n = len(payload) - (len(payload) % stride)

        for off in range(0, n, stride):
            sig = _u16le(payload[off + 0], payload[off + 1])
            x   = _u16le(payload[off + 2], payload[off + 3])
            y   = _u16le(payload[off + 4], payload[off + 5])
            w   = _u16le(payload[off + 6], payload[off + 7])
            h   = _u16le(payload[off + 8], payload[off + 9])

            blocks.append({
                "sig": sig,
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "area": w * h
            })

        return blocks
    
    def best_block(self, sigmap=0xFF, area_min= 500):
        """
        Get the largest block above minimum area threshold.
        
        Returns:
            Block dict or None if no valid block found.
        """
        blocks = self.get_blocks(sigmap=sigmap, max_blocks=10)
        blocks = [b for b in blocks if b["area"] >= area_min]
        
        if not blocks:
            return None
        
        return max(blocks, key=lambda b: b["area"])
