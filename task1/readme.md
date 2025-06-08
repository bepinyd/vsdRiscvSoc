
Week 1 Task - RISC-V Bare-Metal
This document outlines the steps taken and observations made while performing Week 1 tasks related to RISC-V bare-metal programming, using the riscv-none-elf-* toolchain.

1. Install & Sanity-Check the Toolchain
Goal: Install the RISC-V GCC cross-compiler and confirm its functionality.

Steps:

Extract your downloaded toolchain archive:
Bash

tar -xzf riscv-toolchain-rv32imac-x86_64-ubuntu.tar.gz

Add the bin directory to PATH (temporary for current terminal):
Bash

export PATH=$HOME/riscv/bin:$PATH

Permanently add to PATH (if desired):
Bash

echo 'export PATH=$HOME/riscv/bin:$PATH' >> ~/.bashrc
source ~/.bashrc

Check if tools work:
Bash

riscv-none-elf-gcc --version
riscv-none-elf-objdump --version
riscv-none-elf-gdb --version

My Output:
The commands successfully displayed version information for riscv-none-elf-gcc (14.2.0), riscv-none-elf-objdump (2.43.1), and riscv-none-elf-gdb (15.1), confirming the toolchain installation.



2. Compile "Hello, RISC-V"
Goal: Compile a minimal C "hello world" program for RV32IMC and produce an ELF executable.


Files:
hello.c 

C

#include <stdio.h>

int main() {
    printf("Hello, RISC-V!\n");
    return 0;
}
Compile:

Bash

riscv-none-elf-gcc hello.c -march=rv32imc -mabi=ilp32 -o hello.elf
Verify:

Bash

file hello.elf
My Output:
The file hello.elf command confirmed the output was "ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically Linked, not stripped".


3. From C to Assembly
Goal: Generate the assembly (.s) file from the C code and understand the function prologue/epilogue.

Command:

Bash

riscv-none-elf-gcc -S -O0 hello.c -o hello.s
What to look for (Prologue/Epilogue Examples):


Prologue example:
Code snippet

addi sp, sp, -16
sw ra, 12(sp)
Epilogue example:
Code snippet

lw ra, 12(sp)
addi sp, sp, 16
ret
My Output:
The command successfully generated hello.s. (The specific assembly output would be present in the hello.s file).

4. Hex Dump & Disassembly
Goal: Turn the ELF into a raw hex dump and disassemble it with objdump, understanding the meaning of each column.

Disassemble ELF:

Bash

riscv-none-elf-objdump -d hello.elf > hello.dump
Convert to Intel HEX:

Bash

riscv-none-elf-objcopy -O ihex hello.elf hello.hex
Columns in disassembly:

Column	Meaning
Address	Memory address of instruction
Opcode	Instruction machine code (hex)
Instruction	Human-readable assembly

Export to Sheets
My Output:
The commands successfully generated hello.dump and hello.hex. The tail hello.dump command showed disassembled instructions with addresses, opcodes, and assembly mnemonics.

5. ABI & Register Cheat-Sheet
Goal: List all 32 RV32 integer registers with their ABI names and typical calling-convention roles.

Answer Outline:

Reg No	ABI Name	Role
x0	zero	Always 0
x1	ra	Return address
x2	sp	Stack pointer
x3	gp	Global pointer
x4	tp	Thread pointer
x5-x7	t0-t2	Temporaries (caller saved)
x8-x9	s0-s1	Saved registers / frame pointer
x10-x17	a0-a7	Arguments + return values
x18-x27	s2-s11	Saved registers (callee saved)
x28-x31	t3-t6	Temporaries (caller saved)

Export to Sheets
Calling convention summary:


a0-a7: arguments and return values
s0-s11: callee saved
t0-t6: caller saved
6. Stepping with GDB
Goal: Start riscv-none-elf-gdb on the ELF, set a breakpoint at main, step, and inspect registers.

Command:

Bash

riscv-none-elf-gdb hello.elf
Inside GDB:

Code snippet

target sim
break main
run
info registers
step
continue
quit
My Output:
GDB was launched, but break main failed due to "No debugging symbols found in hello.elf" and "Cannot access memory at address 0x10172". Register information could be viewed using info registers.



Learning: For effective debugging, the ELF must be compiled with debugging symbols (e.g., using -g flag during GCC compilation). Also, a simulator like QEMU is often needed to provide a target for GDB.

7. Running Under an Emulator (GDB with QEMU)
Goal: Emulate the RISC-V ELF using QEMU and connect GDB to it for debugging.


Steps:

In one terminal, start QEMU with a GDB server:
Bash

qemu-system-riscv32 -nographic -machine virt -kernel hello.elf -S -gdb tcp::1234

-S: freezes the CPU at startup (waits for debugger).
-gdb tcp::1234: opens port 1234 for GDB to connect.
In a second terminal, launch GDB:
Bash

riscv-none-elf-gdb hello.elf

Inside GDB:
Code snippet

(gdb) target remote localhost:1234
(gdb) break main
(gdb) continue

Learning: This setup allows GDB to break at main and enable stepping through the code, as QEMU emulates the memory and provides a target for the debugger.

8. Exploring GCC Optimization
Theory: Compiler optimization improves performance or reduces code size. Common flags include:

-O0: No optimization (useful for debugging).
-O2: High-level optimization (removes dead code, improves speed).
Commands:

Bash

riscv-none-elf-gcc -S -O0 hello.c -o hello_00.s
riscv-none-elf-gcc -S -O2 hello.c -o hello_02.s
Observe:

Compare the two .s files.
The -O2 version will typically use fewer instructions and more registers.
My Output:
Using diff hello_00.s hello_02.s showed differences, indicating that -O2 indeed resulted in a more optimized assembly output compared to -O0.

9. Inline Assembly Basics
Theory:

C doesn't always provide enough control, so assembly can be embedded using asm volatile.
The csrr instruction is used to read special control and status registers (CSRs), like the cycle counter (0xC00).
File: hello.c 

C

#include <stdint.h>
#define UART_TX (volatile uint8_t*)0x10000000

void uart_puts(const char* s) {
    while (*s) *UART_TX = *s++;
}

static inline uint32_t rdcycle(void) {
    uint32_t c;
    asm volatile ("csrr %0, cycle" : "=r"(c));
    return c;
}

int main() {
    uint32_t cycles = rdcycle();
    uart_puts("Cycle count: ");
    *UART_TX = '0' + (cycles % 10); // crude print, for demo
    uart_puts("\n");
    while (1);
}
Compile: 

Bash

riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 \
-nostdlib -nostartfiles -T linker.ld -o hello.elf hello.c start.s
Run: 

Bash

qemu-system-riscv32 -nographic -machine virt -kernel hello.elf
10. Memory-Mapped I/O Demo
Theory:

In embedded systems, peripherals like GPIO and UART are mapped to memory addresses.
Writing to these addresses controls the peripheral (e.g., 0x10012000 for a GPIO pin).
File: hello.c 

C

#include <stdint.h>
#define GPIO (volatile uint32_t*)0x10012000

int main() {
    *GPIO = 0x1; // Turn on simulated GPIO
    while (1); // Loop forever
}
Compile and Run: Same way as Step 9.

Learning: There will be no visible output in QEMU unless it specifically emulates GPIO. This demonstration is primarily for real hardware.

11. Linker Script 101
Theory:

The linker script controls where code and data are placed in memory.
For the RISC-V virt machine, RAM typically starts at 0x80000000.
File: linker.ld 

Code snippet

ENTRY(_start)
SECTIONS {
    . = 0x80000000;
    .text : { *(.text*) }
    .rodata : { *(.rodata*) }
    .data : { *(.data*) }
    .bss : { *(.bss*) *(COMMON) }
}
Compile: Use -T linker.ld (as already done in previous steps).

12. Start-up Code & crt0
Theory:

Bare-metal programs require a _start function (often found in crt0.S) that:
Sets up the stack.
Calls main().
Loops forever after main returns.
File: start.s 

Code snippet

.globl _start
_start:
    la sp, _stack_top
    call main
    j .

.section .bss
.space 1024
.global _stack_top
_stack_top:
Compile: Include start.s in your compilation command.

13. Interrupt Primer
Goal: Enable the machine-timer interrupt (MTIP) and write a simple handler in C/assembly.

Files:
trap_handler.s 

Code snippet

.section .text
.globl _start
.globl _trap_handler
_start:
    la sp, _stack_top
    call main
    j .

_trap_handler:
    csrr t0, mcause
    li t1, 0x80000007
    bne t0, t1, trap_exit
    # Timer handling logic
    li t2, 0x200bff8 # mtime
    lw t3, 0(t2)
    li t4, 100000
    add t3, t3, t4
    li t5, 0x2004000 # mtimecmp
    sw t3, 0(t5)
    li t6, 'T'
    li t2, 0x10000000 # UART0
    sb t6, 0(t2)
trap_exit:
    mret
.section .bss
.space 1024
.global _stack_top
_stack_top:
Learning: The .globl _trap_handler directive is crucial for the C file to locate the label.

hello.c 

C

#include <stdint.h>
#define UART0 ((volatile uint8_t*)0x10000000)
#define MTIMECMP ((volatile uint64_t*)0x2004000)
#define MTIME ((volatile uint64_t*)0x200bff8)
extern void _trap_handler(); // Let the linker know this comes from assembly

static inline void enable_interrupts() {
    asm volatile("csrw mtvec, %0" :: "r"(&_trap_handler));
    *MTIMECMP = *MTIME + 100000;
    asm volatile("csrs mie, %0" :: "r"(0x80));
    asm volatile("csrs mstatus, %0" :: "r"(0x8));
}

int main() {
    *UART0 = 'M';
    enable_interrupts();
    while (1);
}
Recompile: 

Bash

riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 \
-nostdlib -nostartfiles -T linker.ld \
-o hello.elf hello.c trap_handler.s
Run on QEMU: 

Bash

qemu-system-riscv32 -nographic -machine virt -kernel hello.elf
Output: You should see MTTTTTTTTT....

14. rv32imac vs rv32imc - What's the "A"? (Atomic Instructions)
Theory:

The 'A' extension in rv32imac adds atomic memory instructions like lr.w (Load-Reserved Word) and sc.w (Store-Conditional Word).

These instructions are essential for implementing lock-free synchronization mechanisms such as spinlocks and mutexes in multi-threaded environments or OS kernels.

File: hello.c 

C

#include <stdint.h>
#define UART0 ((volatile uint8_t*)0x10000000)
#define LOCK ((volatile uint32_t*)0x10012000)

void uart_putc(char c) {
    *UART0 = c;
}

void acquire_lock(volatile uint32_t* lock) {
    uint32_t tmp;
    do {
        __asm__ volatile (
            "lr.w %[tmp], (%[lock])\n"
            "bnez %[tmp], 1f\n"
            "li %[tmp], 1\n"
            "sc.w %[tmp], %[tmp], (%[lock])\n"
            "1:"
            : [tmp] "=&r"(tmp)
            : [lock] "r"(lock)
            : "memory"
        );
    } while (tmp != 0);
}

void release_lock(volatile uint32_t* lock) {
    *lock = 0;
}

int main() {
    acquire_lock(LOCK);
    uart_putc('L');
    release_lock(LOCK);
    while (1);
}
Compile + Run: 

Bash

riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 \
-nostdlib -nostartfiles -T linker.ld \
-o hello.elf hello.c start.s
qemu-system-riscv32 -nographic -machine virt -bios none -kernel hello.elf
Output: L.

15. Atomic Mutex Test Program
Theory:

This demonstrates simulating two "pseudo-threads" within main().
The same lock acquisition and release functions from Step 14 are used to guard shared resource access.
File: hello.c 

C

#include <stdint.h>
#define UART0 ((volatile uint8_t*)0x10000000)
#define LOCK ((volatile uint32_t*)0x10012000)

void uart_putc(char c) {
    *UART0 = c;
}

void acquire(volatile uint32_t* lock) {
    uint32_t tmp;
    do {
        __asm__ volatile (
            "lr.w %0, (%1)\n"
            "bnez %0, 1f\n"
            "li %0, 1\n"
            "sc.w %0, %0, (%1)\n"
            "1:"
            : "=&r"(tmp)
            : "r"(lock)
            : "memory"
        );
    } while (tmp != 0);
}

void release(volatile uint32_t* lock) {
    *lock = 0;
}

void thread1() {
    acquire(LOCK);
    uart_putc('A');
    release(LOCK);
}

void thread2() {
    acquire(LOCK);
    uart_putc('B');
    release(LOCK);
}

int main() {
    thread1();
    thread2();
    while (1);
}
Compile + Run: 

Bash

riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 \
-nostdlib -nostartfiles -T linker.ld \
-o hello.elf hello.c start.s
qemu-system-riscv32 -nographic -machine virt -bios none -kernel hello.elf
Output: AB.

16. Using printf() Without OS
Theory:

The printf() function from Newlib can be used in a bare-metal environment by implementing the _write() system call.

Linking requires -lc and -lgcc.
File: syscalls.c 

C

#include <unistd.h>
#include <stdint.h>
#define UART0 ((volatile uint8_t*)0x10000000)

int _write(int fd, const void* buf, size_t len) {
    const char* p = (const char*)buf;
    for (size_t i = 0; i < len; i++)
        *UART0 = p[i];
    return len;
}
File: hello.c 

C

#include <stdio.h>
int main() {
    printf("Hello, printf on bare metal!\n");
    while (1);
}
Compile + Run: 

Bash

riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 \
-nostartfiles -T linker.ld \
-o hello.elf hello.c syscalls.c start.s -lc -lgcc
qemu-system-riscv32 -nographic -machine virt -bios none -kernel hello.elf
Output: Hello, printf on bare metal!.

17. Endianness & Struct Packing
Theory:

RISC-V is typically little-endian by default, meaning the least significant byte is stored at the lowest memory address.

A union trick in C can be used to verify byte ordering.
File: hello.c 

C

#include <stdint.h>
#include <stdio.h>

union {
    uint32_t i;
    uint8_t b[4];
} u;

int main() {
    u.i = 0x01020304;
    printf("Bytes: %x %x %x %x\n", u.b[0], u.b[1], u.b[2], u.b[3]);
    while (1);
}
Compile + Run: Same as Step 16.

Output: Bytes: 4 3 2 1.

Learning: This output confirms the little-endian layout, as the least significant byte (0x04) is stored at the lowest address (index 0).


Sources




