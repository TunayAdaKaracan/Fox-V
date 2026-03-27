# pyright: reportInvalidTypeForm=false


from amaranth import *
from amaranth.lib import wiring

from .isa       import Branch, Load

from .decoder   import Decoder
from .alu       import ALU
from .regfile   import RegisterFile
from .wishbone  import WishboneSignature
from .memory    import WishboneMemory

class ExecutionStage:
    FETCH_INSTRUCTION   = "FETCH_INSTRUCTION"
    FETCH_REGISTERS     = "FETCH_REGISTERS"
    EXECUTE             = "EXECUTE"
    LOAD                = "LOAD"
    STORE               = "STORE"


class CPU(wiring.Component):
    bus: wiring.Out(WishboneSignature(address_width=32, data_width=32, granularity=8))

    def __init__(self, code=[]):
        
        self.regs       = RegisterFile()
        self.decoder    = Decoder()
        self.alu        = ALU()
        self.memory     = WishboneMemory(size=1024, init=code) # 1024 words of memory
        
        super().__init__()
    
    def elaborate(self, platform):
        m = Module()

        m.submodules.decoder    = decoder   = self.decoder
        m.submodules.alu        = alu       = self.alu
        m.submodules.regs       = regfile   = self.regs
        m.submodules.memory     = memory    = self.memory

        wiring.connect(m, self.bus, memory.bus)

        PC          = Signal(32)

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
            alu.immediate   .eq(decoder.alu_op_i)
        ]

        # Default values for Wishbone Bus
        m.d.comb += [
            self.bus.addr   .eq(0),
            self.bus.data_w .eq(0),
            self.bus.sel    .eq(0),
            self.bus.cyc    .eq(0),
            self.bus.stb    .eq(0),
            self.bus.we     .eq(0)
        ]

        takeBranch = Signal(1)
        with m.Switch(decoder.funct3):
            with m.Case(Branch.BEQ):
                m.d.comb += takeBranch.eq(rs1 == rs2)
            with m.Case(Branch.BNE):
                m.d.comb += takeBranch.eq(rs1 != rs2)
            with m.Case(Branch.BLT):
                m.d.comb += takeBranch.eq(rs1.as_signed() < rs2.as_signed())
            with m.Case(Branch.BGE):
                m.d.comb += takeBranch.eq(rs1.as_signed() >= rs2.as_signed())
            with m.Case(Branch.BLTU):
                m.d.comb += takeBranch.eq(rs1.as_unsigned() < rs2.as_unsigned())
            with m.Case(Branch.BGEU):
                m.d.comb += takeBranch.eq(rs1.as_unsigned() >= rs2.as_unsigned())
            # Defaulted to 0
            with m.Case("---"):
                m.d.comb += takeBranch.eq(0)

        nextPC = Signal(32)
        with m.If((decoder.branch & takeBranch) | decoder.jal):
            m.d.comb += nextPC.eq(PC + decoder.immediate)
        with m.Elif(decoder.jalr):
            m.d.comb += nextPC.eq((rs1 + decoder.immediate) & ~Const(1, shape=32))
        with m.Else():
            m.d.comb += nextPC.eq(PC + 4)

        load_data   = Signal(32)
        store_data  = Signal(32)

        # TODO: Fix wishbone bus transactions
        with m.FSM(init=ExecutionStage.FETCH_INSTRUCTION) as fsm:
            # Set addresses to fetch the instruction
            with m.State(ExecutionStage.FETCH_INSTRUCTION):
                m.d.comb += [
                    self.bus.addr   .eq(PC),
                    self.bus.data_w .eq(0),
                    self.bus.sel    .eq(0b1111),
                    self.bus.cyc    .eq(1),
                    self.bus.stb    .eq(1),
                ]

                with m.If(self.bus.ack):
                    m.d.sync += instruction.eq(self.bus.data_r)
                    m.next = ExecutionStage.FETCH_REGISTERS
            
            # Fetch rs1, rs2
            with m.State(ExecutionStage.FETCH_REGISTERS):
                m.d.sync += [
                    rs1.eq(regfile.rs1_data),
                    rs2.eq(regfile.rs2_data)
                ]

                m.next = ExecutionStage.EXECUTE
            
            with m.State(ExecutionStage.EXECUTE):
                with m.If(~decoder.system):
                    m.d.sync += PC.eq(nextPC)

                with m.If(decoder.load):
                    m.next = ExecutionStage.LOAD
                with m.Elif(decoder.store):
                    m.next = ExecutionStage.STORE
                with m.Else():
                    m.next = ExecutionStage.FETCH_INSTRUCTION
            
            with m.State(ExecutionStage.LOAD):
                m.d.comb += [
                    self.bus.addr   .eq(rs1 + decoder.immediate),
                    self.bus.sel    .eq(0b1111), # TODO: Sel
                    self.bus.cyc    .eq(1),
                    self.bus.stb    .eq(1),
                ]

                with m.If(self.bus.ack):
                    m.d.sync += load_data.eq(
                        #TODO: Load
                    )
                    m.next = ExecutionStage.FETCH_REGISTERS
            
            with m.State(ExecutionStage.STORE):
                # TODO: ...
                pass

        writeback_en    = Signal(1)
        writeback_data  = Signal(32)

        # Writeback Enable
        with m.If(fsm.ongoing(ExecutionStage.EXECUTE)):
            m.d.comb += writeback_en.eq(
                decoder.alu_op  |
                decoder.jal     |
                decoder.jalr    |
                decoder.lui     |
                decoder.auipc
            )
        with m.Elif(fsm.ongoing(ExecutionStage.LOAD)):
            m.d.comb += writeback_en.eq(decoder.load)
        with m.Else():
            m.d.comb += writeback_en.eq(0)

        # Writeback Data
        with m.If(decoder.load):
            m.d.comb += writeback_data.eq(load_data)
        with m.Elif(decoder.jal | decoder.jalr):
            m.d.comb += writeback_data.eq(PC + 4)
        with m.Elif(decoder.lui):
            m.d.comb += writeback_data.eq(decoder.immediate)
        with m.Elif(decoder.auipc):
            m.d.comb += writeback_data.eq(PC + decoder.immediate)
        with m.Else():
            m.d.comb += writeback_data.eq(alu.result)

        m.d.comb += [
            regfile.rs_re   .eq(fsm.ongoing(ExecutionStage.FETCH_REGISTERS)),
            regfile.rd_we   .eq(writeback_en),
            regfile.rd_data .eq(writeback_data)
        ]

        return m

