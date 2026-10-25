## Project Change Log

This is the project changelog starting from version 2.0.0.0.
The current hardware version is available in the `mimpid` CSR as a four-number string (BCD encoded),

As an example, see below:

```
mimpid = 0x02010312 -> Version 02.01.03.12 -> v2.1.3.12
```

All dates are in dd.mm.yyyy format.

| Date       | Version  | Comment | Issue |
|:----------:|:--------:|:--------|:-----:|
| 17-08-2026 | 2.0.0.0  | First commit of 5-stage pipelined SoC | |
| 18-08-2026 | 2.0.0.1  | [core] fix vectored imterrupt calculation | |
| 20-08-2026 | 2.0.0.2  | [core] interrupts only acknowledged when not stepping, halt ack only when in debug state | |
| 22-08-2026 | 2.0.0.3  | [core] fixed an issue where trap during step did not save the correct PC | |
| 30-08-2026 | 2.0.0.4  | [core] implemented Zimop, removed unused signal, FENCE now count as retired instruction | |
| 01-09-2026 | 2.0.0.5  | [core] WFI now retires | |
| 08-09-2026 | 2.0.0.6  | [core] DCSR.cause is now written as one-shot | |
| 12-09-2026 | 2.0.0.7  | [core] fix a bug that prevented to entering debug state on the first HW trigger, code cleanup | |
| 13-09-2026 | 2.0.0.8  | [mem/address] all memories now return zero when not accessed, addres decoder now fuses returns from memory and I/O | |
| 18-09-2026 | 2.0.0.9  | [io_bust_switch] fuse all I/O output | |
| 19-09-2026 | 2.0.0.10 | [core] implemented Zbb | |
| 01-10-2026 | 2.0.0.11 | [core] dcsr.NMIP reflects NMI pending | |
| 06-10-2026 | 2.0.0.12 | [core] make CSR addr/imm default from instruction | |
| 09-10-2026 | 2.0.0.13 | [core] removed signal triggering if_id.pc | |
| 10-10-2026 | 2.0.0.14 | [io_bus_switch] remove multiplexer logic | |
| 21-10-2026 | 2.0.0.15 | [...] implement 1-Wire ONEWIRE1 device, implemented Zbkb extension | |
| 25-10-2025 | 2.0.0.16 | [onewire] support for bit transmission to support scanning the 1-Wire bus for attached devices | |

