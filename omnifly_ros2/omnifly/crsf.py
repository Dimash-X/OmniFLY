"""Minimal CRSF (Crossfire/ExpressLRS) RC-channel frame builder."""

CRSF_SYNC = 0xC8
TYPE_RC_CHANNELS = 0x16
CH_MIN, CH_MID, CH_MAX = 172, 992, 1811


def crc8(data: bytes, poly: int = 0xD5) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def norm_to_crsf(v: float) -> int:
    """-1..1 -> 172..1811 (992 = centre)."""
    v = max(-1.0, min(1.0, v))
    return int(round(CH_MID + v * (CH_MAX - CH_MID)))


def unit_to_crsf(v: float) -> int:
    """0..1 -> 172..1811 (throttle)."""
    v = max(0.0, min(1.0, v))
    return int(round(CH_MIN + v * (CH_MAX - CH_MIN)))


def rc_frame(channels) -> bytes:
    """Pack up to 16 channels (11 bit each) into a CRSF RC frame."""
    ch = list(channels) + [CH_MID] * (16 - len(channels))
    bits = 0
    for i, c in enumerate(ch[:16]):
        bits |= (int(c) & 0x7FF) << (11 * i)
    body = bytes([TYPE_RC_CHANNELS]) + bits.to_bytes(22, "little")
    return bytes([CRSF_SYNC, len(body) + 1]) + body + bytes([crc8(body)])
