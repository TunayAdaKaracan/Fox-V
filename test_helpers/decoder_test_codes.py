from riscv_assembler.convert import AssemblyConverter

# TODO: Use custom asm instead of this
_code_1 = """
    addi s0 x0 10
    addi s1 x0 10
    beq s1 s0 loop
    loop:
        addi s1 s0 -32
"""

# TODO: Tests framework
code_1 = AssemblyConverter().convert(_code_1)


if __name__ == "__main__":
    print(code_1)