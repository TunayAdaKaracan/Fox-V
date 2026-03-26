from enum import IntEnum
# Instruction Types:

# R-Type: rd = rs1 OP rs2
# funct7[7]     | rs2[5] | rs1[5] | funct3[3] | rd[5]       | opcode[7]

# I-Type: rd = rs1 OP Imm(12) | Load Op
# Imm[11:0]              | rs1[5] | funct3[3] | rd[5]       | opcode[7]

# S-Type: 
# Imm[11:5]     | rs2[5] | rs1[5] | funct3[3] | Imm[4:0]    | opcode[7]

# B-Type:
# Imm[12|10:5]  | rs2[5] | rs1[5] | funct3[3] | Imm[4:1|11] | opcode[7]

# U-Type:
# Imm[31:12]                                  | rd[5]       | opcode[7]

# J-Type:
# Imm[20|10:1|11|19:12]                       | rd[5]       | opcode[7]

class OpCodes(IntEnum):
    LUI     = 0b0110111 # U-Type -> Load Upper Immediate
    AUIPC   = 0b0010111 # U-Type -> Add Upper Immediate to PC

    JAL     = 0b1101111 # J-Type -> Jump and Link
    JALR    = 0b1100111 # I-Type -> Jump and Link Register

    # Branch Operations (0b1100011) | B-Type
    BRANCH  = 0b1100011 # Grouping
    BEQ     = 0b1100011 # funct3: 000
    BNE     = 0b1100011 # funct3: 001
    BLT     = 0b1100011 # funct3: 100
    BGE     = 0b1100011 # funct3: 101
    BLTU    = 0b1100011 # funct3: 110
    BGEU    = 0b1100011 # funct3: 111

    # Load Operations   (0b0000011) | I-Type
    LOAD    = 0b0000011 # Grouping
    LB      = 0b0000011
    LH      = 0b0000011
    LW      = 0b0000011
    LBU     = 0b0000011
    LHU     = 0b0000011

    # Store Operations  (0b0100011) | S-Type
    STORE   = 0b0100011 # Grouping
    SB      = 0b0100011
    SH      = 0b0100011
    SW      = 0b0100011

    # ALU Operations:
    # opcode[5] == 0b1 -> Immediate Mode
    # opcode[5] == 0b0 -> Register  Mode

    # ALU-I Operations  (0b0010011) | I-Type
    ALU_I   = 0b0010011 # Grouping
    ADDI    = 0b0010011
    SLTI    = 0b0010011
    SLTIU   = 0b0010011
    XORI    = 0b0010011
    ORI     = 0b0010011
    ANDI    = 0b0010011
    SLLI    = 0b0010011
    SRLI    = 0b0010011
    SRAI    = 0b0010011

    # ALU-R Operations  (0b0110011) | R-Type
    ALU_R   = 0b0110011 # Grouping
    ADD     = 0b0110011
    SUB     = 0b0110011
    SLL     = 0b0110011
    SLT     = 0b0110011
    SLTU    = 0b0110011
    XOR     = 0b0110011
    SRL     = 0b0110011
    SRA     = 0b0110011
    OR      = 0b0110011
    AND     = 0b0110011

    # Fence Operaton    (0b0001111) | I-Type
    FENCE   = 0b0001111

    # System Operations (0b1110011)
    SYSTEM  = 0b1110011 # Grouping
    ECALL   = 0b1110011
    EBREAK  = 0b1110011

class Branch(IntEnum):
    BEQ     = 0b000
    BNE     = 0b001
    BLT     = 0b100
    BGE     = 0b101
    BLTU    = 0b110
    BGEU    = 0b111

class Load(IntEnum):
    LB      = 0b000
    LH      = 0b001
    LW      = 0b010
    LBU     = 0b100
    LHU     = 0b101

class AluOp(IntEnum):
    ADD     = 0b000
    SUB     = 0b000
    SLL     = 0b001
    SLT     = 0b010
    SLTU    = 0b011
    XOR     = 0b100
    SRL     = 0b101
    SRA     = 0b101
    OR      = 0b110
    AND     = 0b111

class AluOpFunct(IntEnum):
    # funct7[5] | AluOp
    ADD     = 0b0_000
    SUB     = 0b1_000
    SLL     = 0b0_001
    SLT     = 0b0_010
    SLTU    = 0b0_011
    XOR     = 0b0_100
    SRL     = 0b0_101
    SRA     = 0b1_101
    OR      = 0b0_110
    AND     = 0b0_111