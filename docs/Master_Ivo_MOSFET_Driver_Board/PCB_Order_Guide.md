# How to Build: Master Ivo's Isolated MOSFET Driver Board (v6.2)

This guide explains how to order parts and assemble the isolated SiC MOSFET driver board based on the v6.2 design.

---

## BOM (Bill of Materials)

All parts have been consolidated into a Mouser-ready BOM for convenience:

 [Download Mouser_FULL_BOM_for_3xBoards.csv](./Mouser_FULL_BOM_for_3xBoards.csv)

- Includes parts for **3 complete boards**
- Most critical components sourced from **Wolfspeed**, **Infineon**, **Murata**, **WIMA**
- Compatible with standard passive substitutions where noted

---

## PCB Files

If you don’t already have the PCBs, they can be fabricated by uploading the Gerbers below:

 `Gerber_Single-Isolated-mosfet-switch-PCBv6.2.zip`  
 Contains all layers: copper, silkscreen, drill, outline  
 Recommended fab settings:
- 2-layer board
- 1.6 mm FR4
- HASL or ENIG finish
- Solder mask: Green
- Silkscreen: Top only (optional)

---

## Build Tips

- Mount **low-profile components first** (resistors, SMD ICs)
- Use **heatsinks and thermal paste** for TO-247 SiC MOSFETs
- Respect **isolation clearance** when wiring gate and load side
- **Gate resistor** (10–33 Ω) and optional **TVS diode** on drain recommended for inductive loads

---

## Acknowledgments

Original design by **Master Ivo**  
This repository extends his open-source work with sourcing and support documentation.
