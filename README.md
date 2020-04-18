# RAM, ROM and IO for mos6502 with Arduino Mega

## Compile the Assembly

I use [this Assembler](http://sun.hasenbraten.de/vasm/). Compiled with

```bash
make CPU=6502 SYNTAX=oldstyle
```

After you compiled it just run:

```bash
vasm6502_oldstyle -Fbin -dotdir test.asm
```

### Dump the binary

```bash
hexdump -e '16/1 " %02X" "\n"' test.bin
```
