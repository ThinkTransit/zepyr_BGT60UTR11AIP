
Overview
********

Test code for BGT60UTR11AIP

west build -b nrf5340dk/nrf5340/cpuapp .

Expected output
********

Code performs read/write test.

Code reads value from registers 0x00, 0x01 and 0x02

Expected default values from datasheet are 0x1C0B00, 0x0A0240 and 0x00070C

```
*** Booting Zephyr OS build 4.1.0-rc3 ***
Starting
Wrote 0x005500 to reg 0x06
Read back reg 0x06 = 0x005500 (expected 0x005500)
Read reg 0x00 = 0x1C0B00 (GSR0: 0xF0)
Read reg 0x01 = 0x0A0240 (GSR0: 0xF0)
Read reg 0x02 = 0x00070C (GSR0: 0xF0)
```
