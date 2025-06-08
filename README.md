# RISC-V Bare-Metal Programming – Week 1 Tasks

This repository contains my progress and experiments while learning RISC-V bare-metal programming using the `riscv-none-elf` toolchain and QEMU. Each step builds foundational knowledge in embedded systems programming without an OS.

---

## 📦 Toolchain Setup

### ✅ Install & Sanity Check

1. **Extract the Toolchain:**
   ```bash
   tar -xzf riscv-toolchain-rv32imac-x86_64-ubuntu.tar.gz
   ```
   ![Toolchain Setup](assets/.png)


2. **Add to PATH:**
   ```bash
   export PATH=$HOME/riscv/bin:$PATH
   echo 'export PATH=$HOME/riscv/bin:$PATH' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Verify Installation:**
   ```bash
   riscv-none-elf-gcc --version
   riscv-none-elf-objdump --version
   riscv-none-elf-gdb --version
   ```

---

## 🧪 Task List & Implementation

### 1️⃣ Hello, RISC-V (Minimal C Program)

- File: `hello.c`
- Compile:
  ```bash
  riscv-none-elf-gcc hello.c -march=rv32imc -mabi=ilp32 -o hello.elf
  file hello.elf
  ```

### 2️⃣ Generate Assembly from C

```bash
riscv-none-elf-gcc -S -O0 hello.c -o hello.s
```
- Look for function **prologue** and **epilogue**.

### 3️⃣ Hex Dump & Disassembly

```bash
riscv-none-elf-objdump -d hello.elf > hello.dump
riscv-none-elf-objcopy -O ihex hello.elf hello.hex
```

### 4️⃣ Register ABI Cheat-Sheet

| Reg | Name | Description            |
|-----|------|------------------------|
| x0  | zero | Constant zero          |
| x1  | ra   | Return address         |
| x2  | sp   | Stack pointer          |
| x3  | gp   | Global pointer         |
| x4  | tp   | Thread pointer         |
| x5–7 | t0–2| Temporaries            |
| x8–9 | s0–1| Saved registers        |
| x10–17 | a0–7| Arguments/Return values |
| x18–27 | s2–11| Callee-saved       |
| x28–31 | t3–6| Temporaries         |

---

## 🧵 GDB & QEMU Debugging

### 5️⃣ GDB Standalone

```bash
riscv-none-elf-gdb hello.elf
(gdb) target sim
(gdb) break main
(gdb) run
(gdb) info registers
(gdb) step
(gdb) continue
```

### 6️⃣ GDB with QEMU

**Terminal 1:**
```bash
qemu-system-riscv32 -nographic -machine virt -kernel hello.elf -S -gdb tcp::1234
```

**Terminal 2:**
```bash
riscv-none-elf-gdb hello.elf
(gdb) target remote localhost:1234
(gdb) break main
(gdb) continue
```

---

## ⚙️ Advanced Tasks

### 7️⃣ GCC Optimization

```bash
riscv-none-elf-gcc -S -O0 hello.c -o hello_O0.s
riscv-none-elf-gcc -S -O2 hello.c -o hello_O2.s
```

---

### 8️⃣ Inline Assembly (Read Cycle Counter)

```c
static inline uint32_t rdcycle(void) {
    uint32_t c;
    asm volatile ("csrr %0, cycle" : "=r"(c));
    return c;
}
```

Compile:
```bash
riscv-none-elf-gcc -march=rv32imac_zicsr -mabi=ilp32 -nostdlib -nostartfiles -T linker.ld -o hello.elf hello.c start.s
```

---

### 9️⃣ Memory-Mapped I/O Demo

```c
#define GPIO (volatile uint32_t*)0x10012000
*GPIO = 0x1;  // Turn on GPIO
```

---

### 🔟 Linker Script Basics (`linker.ld`)

```ld
ENTRY(_start)
SECTIONS {
  . = 0x80000000;
  .text : { *(.text*) }
  .rodata : { *(.rodata*) }
  .data : { *(.data*) }
  .bss : { *(.bss*) *(COMMON) }
}
```

---

## 🧃 Startup Code & Interrupts

### 1️⃣1️⃣ `start.s` (Minimal Boot Code)

```asm
.globl _start
_start:
    la sp, _stack_top
    call main
    j .

.section .bss
.space 1024
.global _stack_top
_stack_top:
```

---

### 1️⃣2️⃣ Timer Interrupt with Trap Handler

Trap handler handles `mcause`, compares with timer interrupt, updates `mtimecmp`, and sends UART character.

---

## 🧵 Atomic Memory Instructions (`A` Extension)

### 1️⃣3️⃣ Lock Using `lr.w` and `sc.w`

```c
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
```

---

### 1️⃣4️⃣ Mutex Test With Two Threads

```c
void thread1() {
    acquire_lock(LOCK);
    uart_putc('A');
    release_lock(LOCK);
}
void thread2() {
    acquire_lock(LOCK);
    uart_putc('B');
    release_lock(LOCK);
}
```

Expected output: `AB`

---

## 🖨️ Printf Without OS

Implement `_write` syscall:
```c
int _write(int fd, const void* buf, size_t len) {
    const char* p = buf;
    for (size_t i = 0; i < len; i++)
        *UART0 = p[i];
    return len;
}
```

Compile with `-lc -lgcc`.

---

## 🧠 Endianness & Struct Packing

```c
union {
    uint32_t i;
    uint8_t b[4];
} u = { .i = 0x01020304 };

printf("%x %x %x %x\n", u.b[0], u.b[1], u.b[2], u.b[3]);
```

Expected Output:
```
4 3 2 1
```

---

## 🧾 Summary

| Task             | Description                     |
|------------------|---------------------------------|
| ✅ Toolchain      | Install and verify toolchain    |
| ✅ Compilation    | Compile C to ELF                |
| ✅ Assembly       | Generate and read assembly      |
| ✅ Debugging      | Use GDB and QEMU                |
| ✅ Optimization   | Compare -O0 vs -O2              |
| ✅ Inline ASM     | Use inline assembly in C        |
| ✅ Interrupts     | Setup and use timer interrupt   |
| ✅ Memory I/O     | GPIO/UART simulation            |
| ✅ Atomic Ops     | Locking using lr.w / sc.w       |
| ✅ Syscalls       | Use printf via _write syscall   |
| ✅ Endianness     | Byte order exploration          |

---

## 📁 File Structure

```
.
├── hello.c
├── start.s
├── linker.ld
├── trap_handler.s
├── syscalls.c
├── hello.elf
├── hello.dump
├── hello.hex
└── README.md
```

---

## 📚 References

- [RISC-V Toolchain Docs](https://github.com/riscv-collab/riscv-gnu-toolchain)
- [QEMU RISC-V](https://wiki.qemu.org/Documentation/Platforms/RISCV)
- [The Embedded Rust Book (applies conceptually)](https://docs.rust-embedded.org/book/)
