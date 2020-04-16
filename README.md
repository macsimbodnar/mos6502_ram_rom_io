# RAM, ROM and IO for mos6502 with Arduino Mega

### Compile the Assembly

I use [this Assembler](http://www.floodgap.com/retrotech/xa/).

After you compiled it just run:

```bash
xa test.asm -o test.bin
```

### Dump the binary

```bash
hexdump -e '16/1 " %02X" "\n"' test.bin
```
