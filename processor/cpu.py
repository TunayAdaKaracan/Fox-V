# pyright: reportInvalidTypeForm=false


from amaranth import *
from amaranth.lib import wiring

from .decoder   import Decoder
from .alu       import ALU
from .regfile   import RegisterFile
from .wishbone  import WishboneSignature
from .memory    import WishboneMemory

class ExecutionStage:
    FETCH_INSTRUCTION   = "FETCH_INSTRUCTION"
    FETCH_REGISTERS     = "FETCH_REGISTERS"
    EXECUTE             = "EXECUTE"
    WRITE_BACK          = "WRITE_BACK"


class CPU(wiring.Component):
    bus: wiring.Out(WishboneSignature(address_width=32, data_width=32, granularity=8))
    def __init__(self, code):
        
        self.regs       = RegisterFile()
        self.decoder    = Decoder()
        self.alu        = ALU()
        self.memory     = WishboneMemory(size=1024) # 1024 words of memory
        
        super().__init__({})
    
    def elaborate(self):
        m = Module()

        m.submodules.decoder    = decoder   = self.decoder
        m.submodules.alu        = alu       = self.alu
        m.submodules.regs       = regfile   = self.regs
        m.submodules.memory     = memory    = self.memory

        wiring.connect(m, self.bus, memory.bus)

        PC = Signal(32)

        instruction = Signal(32)
        rs1         = Signal(32)
        rs2         = Signal(32)

        m.d.comb += [
            # Decoder Stage
            self.decoder.instruction.eq(instruction),

            regfile.rs1_addr.eq(decoder.rs1),
            regfile.rs2_addr.eq(decoder.rs2),
            regfile.rd_addr .eq(decoder.rd),

            # ALU Stage
            alu.value1      .eq(rs1),
            alu.value2      .eq(Mux(decoder.alu_op_i, decoder.immediate, rs2)),
            alu.funct3      .eq(decoder.funct3),
            alu.funct7      .eq(decoder.funct7),
            alu.immediate   .eq(decoder.alu_op_i),

            regfile.rd_data .eq(alu.result)
        ]

        m.d.comb += [
            self.bus.addr   .eq(0),
            self.bus.data_w .eq(0),
            self.bus.sel    .eq(0),
            self.bus.cyc    .eq(0),
            self.bus.stb    .eq(0),
            self.bus.we     .eq(0)
        ]

        with m.FSM(reset=ExecutionStage.FETCH_INSTRUCTION) as fsm:
            # Set addresses to fetch the instruction
            with m.State(ExecutionStage.FETCH_INSTRUCTION):
                m.d.comb += [
                    self.bus.addr   .eq(PC),
                    self.bus.data_w .eq(0),
                    self.bus.sel    .eq(0b1111),
                    self.bus.cyc    .eq(1),
                    self.bus.stb    .eq(1),
                    self.bus.we     .eq(1)
                ]
                with m.If(self.bus.ack):
                    m.d.sync += instruction.eq(self.bus.data_r)
                    m.next = ExecutionStage.FETCH_REGISTERS
                with m.Else():
                    m.next = ExecutionStage.FETCH_INSTRUCTION
            
            # Fetch rs1, rs2
            with m.State(ExecutionStage.FETCH_REGISTERS):
                m.d.sync += [
                    rs1.eq(regfile.rs1_data),
                    rs2.eq(regfile.rs2_data)
                ]

                m.next = ExecutionStage.EXECUTE
            
            with m.State(ExecutionStage.EXECUTE):
                m.d.sync += PC.eq(PC+4)

                m.next = ExecutionStage.FETCH_INSTRUCTION

        m.d.comb += [
            regfile.rs_re.eq(fsm.ongoing == ExecutionStage.FETCH_REGISTERS),
        ]
        
        return m

