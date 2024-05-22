Please flash the binary to esp32c6 with the following command:
```bash
    esptool.py --chip esp32c6 -p <PORT> write_flash --flash_size 4MB --flash_freq 80m 0x0 c6_slave_22_may.bin
```
