# pyright: reportInvalidTypeForm=false

from amaranth import *
from amaranth.lib import wiring
from amaranth.lib.wiring import In, Out


class WishboneSignature(wiring.Signature):
    def __init__(self, address_width=32, data_width=32, granularity=None):
        super().__init__({
            "addr":     Out(address_width),
            "data_w":   Out(data_width),    # Data Out of Master
            "data_r":   In(data_width),     # Data In  of Master
            "sel":      Out(data_width // (granularity or data_width)),
            "cyc":      Out(1),
            "stb":      Out(1),
            "we":       Out(1),
            "ack":      In(1)
        })

# TODO: Decoder