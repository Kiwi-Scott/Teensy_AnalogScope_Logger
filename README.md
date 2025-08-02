# Teensy Analog Scope Logger

An open-source analog oscilloscope and signal logger based on the Teensy 4.1 MCU. Supports both real-time waveform rendering and high-resolution post-capture overlays using standard BNC probes and opto-isolated analog front ends.

...

## How It Works
- Live mode: ADC stream from probe → chart overlay
- Logger mode: Buffered burst capture + serial dump
- Input stage: AMC3301 + differential → single-ended opamp
- UI: HTML/JS with toggleable traces + probe attenuation

...

## Contributors
- **Scott Neels (Kiwi-Scott)** – Hardware architect, lead innovator
- **OpenAI Collaboration** – Software + modeling support

