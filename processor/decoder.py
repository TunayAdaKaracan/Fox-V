# pyright: reportInvalidTypeForm=false

from isa import OpCodes

from amaranth import *
from amaranth.lib import wiring


class Decoder(wiring.Component):
    # Inputs
    instruction:    wiring.In(32)

    # Outputs Values
    rs1:            wiring.Out(5)
    rs2:            wiring.Out(5)
    rd:             wiring.Out(5)

    funct3:         wiring.Out(3)
    funct7:         wiring.Out(7)

    immediate:      wiring.Out(32)

    # Output Signals
    lui:            wiring.Out(1)
    auipc:          wiring.Out(1)

    jal:            wiring.Out(1)
    jalr:           wiring.Out(1)

    branch:         wiring.Out(1)
    load:           wiring.Out(1)
    store:          wiring.Out(1)

    alu_op:         wiring.Out(1)
    alu_op_i:       wiring.Out(1)

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
            self.alu_op.eq((opcode  == OpCodes.ALU_R) | (opcode == OpCodes.ALU_I)),
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

        # Define immediates to be used later
        # No Rimm as R-Type instructions do not contain immediate values
        Iimm = Signal(32)
        Simm = Signal(32)
        Bimm = Signal(32)
        Uimm = Signal(32)
        Jimm = Signal(32)
        m.d.comb += [
            Iimm.eq(Cat(self.instruction[20:32], self.instruction[31].replicate(20))),

            Simm.eq(Cat(self.instruction[7:12], self.instruction[25:32], self.instruction[31].replicate(20))),
            
            Bimm.eq(Cat(Const(0), self.instruction[8:12], self.instruction[25:31], self.instruction[7], self.instruction[31].replicate(20))),
            
            Uimm.eq(Cat(Const(0).replicate(12), self.instruction[12:32])),

            Jimm.eq(Cat(Const(0), self.instruction[21:31], self.instruction[20], self.instruction[12:20], self.instruction[31].replicate(12)))
        ]

        with m.Switch(opcode):
            # Maps to OpCodes.LUI and OpCodes.AUIPC
            with m.Case("0-10111"):
                m.d.comb += self.immediate.eq(Uimm)

            with m.Case(OpCodes.JAL):
                m.d.comb += self.immediate.eq(Jimm)

            with m.Case(OpCodes.JALR):
                m.d.comb += self.immediate.eq(Iimm)

            with m.Case(OpCodes.BRANCH):
                m.d.comb += self.immediate.eq(Bimm)

            with m.Case(OpCodes.LOAD):
                m.d.comb += self.immediate.eq(Iimm)

            with m.Case(OpCodes.STORE):
                m.d.comb += self.immediate.eq(Simm)

            with m.Case(OpCodes.ALU_I):
                m.d.comb += self.immediate.eq(Iimm)

            # Set immediate output to 0 if R-Type
            with m.Case(OpCodes.ALU_R):
                m.d.comb += self.immediate.eq(0)

            with m.Case(OpCodes.FENCE):
                m.d.comb += self.immediate.eq(Iimm)

            # System instructions do not contain immediate
            with m.Case(OpCodes.SYSTEM):
                m.d.comb += self.immediate.eq(0)
                
        return m