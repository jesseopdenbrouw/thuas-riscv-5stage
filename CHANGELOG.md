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

