# pyright: reportInvalidTypeForm=false

from amaranth import *
from amaranth.lib import wiring

from .wishbone import WishboneSignature
from amaranth.lib.memory import Memory


class WishboneMemory(wiring.Component):
    bus:    wiring.In(WishboneSignature(address_width=32, data_width=32, granularity=8))
    
    def __init__(self, size):
        self.memory = Memory(shape=32, depth=size, init=[0])
        self.write  = self.memory.write_port(granularity=8)
        self.read   = self.memory.read_port()
        super().__init__()

    def elaborate(self):
        m = Module()
        m.submodules.memory = self.memory

        is_selected = Signal(1)
        ack         = Signal(1)

        m.d.comb += [
            is_selected.eq(self.bus.cyc & self.bus.stb),

            self.read.en.eq(is_selected & (~self.bus.we)),
            self.read.addr.eq(self.bus.adr),
            self.bus.data_r.eq(self.read.data),

            self.write.en.eq(Mux(is_selected & self.bus.we, self.bus.sel, 0)),
            self.write.addr.eq(self.bus.adr),
            self.write.data.eq(self.bus.dat_w),
        ]

        m.d.sync += ack.eq(is_selected & (~ack))

        m.d.comb += self.bus.ack.eq(ack)
        return m