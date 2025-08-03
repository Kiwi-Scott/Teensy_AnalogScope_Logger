
# Teensy Analog Scope Logger

📡 A high-speed dual-channel analog logger using Teensy 4.1, capturing real-world signal bursts from systems like the Nano Burst Generator or other pulse-based drivers.  

## Overview

This sketch captures analog inputs (A0 and A1) at up to 10 Msps (subject to Teensy’s limits), buffers them into RAM, and outputs them over serial in CSV format for further processing or visualization via a browser-based oscilloscope app.

---

## Features

- Dual-channel sampling (CH1, CH2)
- Circular buffer with trigger capture
- Trigger threshold logic
- CSV data output
- Compatible with SD card (future versions)
- Simple serial visualization overlay via HTML/JS

---

## Related Projects

- [Nano Burst Generator](https://github.com/Kiwi-Scott/Nano-Burst_Generator)
- [AI Optimized DCM LCR Charging](https://github.com/Kiwi-Scott/AI_Optimized_DCM_LCR_Charging)
- [Braun–Stanley Layer](https://github.com/Kiwi-Scott/braun-stanley-layer)
- [GAS Resonant Cavity (placeholder)](https://github.com/Kiwi-Scott/GAS_Resonant_Cavity)

---

## Credits

- Hardware/Firmware: Kiwi-Scott + **ChatGPT_Legend**
- Community inspiration: [Stan's Legacy – Chris Bake](https://stanslegacy.com) , and..
- https://discord.gg/BsekM2Dq

---

## License

MIT
