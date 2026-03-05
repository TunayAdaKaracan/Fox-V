# pyright: reportInvalidTypeForm=false

from isa import OpCodes
from test_helpers.decoder_test_codes import code_1

from amaranth import *
from amaranth.sim import Period, Simulator
from amaranth.lib import wiring


class Decoder(wiring.Component):
    # Inputs
    instruction:    wiring.In(32)

    # Outputs Values
    rs1:            wiring.Out(5)
    rs2:            wiring.Out(5)
    rd:             wiring.Out(5)

    funct7:         wiring.Out(7)
    funct3:         wiring.Out(3)

    immediate:      wiring.Out(32)

    # Output Signals
    lui:            wiring.Out(1)
    auipc:          wiring.Out(1)

    jal:            wiring.Out(1)
    jalr:           wiring.Out(1)

    branch:         wiring.Out(1)
    load:           wiring.Out(1)
    store:          wiring.Out(1)

    alu_op_i:       wiring.Out(1)
    alu_op_r:       wiring.Out(1)

    fence:          wiring.Out(1)
    system:         wiring.Out(1)

    def __init__(self):
        super().__init__()
    
    def elaborate(self, platform):
        m = Module()

        opcode = Signal(7)
        m.d.comb += opcode.eq(self.instruction[0:7])

        m.d.comb += [
            self.lui.eq(opcode      == OpCodes.LUI),
            self.auipc.eq(opcode    == OpCodes.AUIPC),
            self.jal.eq(opcode      == OpCodes.JAL),
            self.jalr.eq(opcode     == OpCodes.JALR),
            self.branch.eq(opcode   == OpCodes.BRANCH),
            self.load.eq(opcode     == OpCodes.LOAD),
            self.store.eq(opcode    == OpCodes.STORE),
            self.alu_op_i.eq(opcode == OpCodes.ALU_I),
            self.alu_op_r.eq(opcode == OpCodes.ALU_R),
            self.fence.eq(opcode    == OpCodes.FENCE),
            self.system.eq(opcode   == OpCodes.SYSTEM)
        ]

        m.d.comb += [
            self.rs1.eq(self.instruction[15:20]),
            self.rs2.eq(self.instruction[20:25]),
            self.rd.eq(self.instruction[7:12])
        ]

        m.d.comb += [
            self.funct3.eq(self.instruction[12:15]),
            self.funct7.eq(self.instruction[25:32])
        ]

        # TODO: Immediate decoding
        m.d.comb += self.immediate.eq(0)
        return m

def test():
    top = Decoder()

    async def bench(ctx):
        for instr in code_1:
            print(f"Emulating instruction {instr}")
            ctx.set(top.instruction, int(instr, base=2))
            await ctx.delay(Period(us=2))
    

    sim = Simulator(top)
    sim.add_testbench(bench)

    with sim.write_vcd("sim_output/decoder.vcd"):
        sim.run()

if __name__ == "__main__":
    test()