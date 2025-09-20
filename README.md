<p align="center">
  <img src="https://raw.githubusercontent.com/svnty/ISS-arduino-tracker/refs/heads/main/images/logo.png?raw=true" alt="International Space Station Orbit Tracker"/>
</p>

# International Space Station Orbit Tracker

Arduino implementation of an ISS tracker

Print all files in stl_files folder, use Nylon for the bearing (Race Inner Bottom, Race Inner Top, Race Outer, Rolls) at a high infill, and PLA for everything else at 15% infill.

## Model

<p align="center">
  <img src="https://raw.githubusercontent.com/svnty/ISS-arduino-tracker/refs/heads/main/images/ISS-arduino-tracker.png?raw=true" alt="International Space Station Orbit Tracker"/>
</p>

## Parts list

| Part | Cost (AUD) |
|--|--|
| [Ardino uno R4 WiFi](https://www.jaycar.com.au/arduino-uno-wifi-r4-development-board/p/XC9211) | $50 |
| [Nema 17 stepper motor](https://www.jaycar.com.au/nema17-stepper-motor/p/YM2756) | $25 |
| [SG90 servo motor](https://www.jaycar.com.au/arduino-compatible-9g-micro-servo-motor/p/YM2758) | $12 |
| [TMC2209 stepper motor driver](https://www.ebay.com.au/sch/i.html?_nkw=TMC2209) | $15 |
| [230V AC -> 12V DC power supply](https://www.jaycar.com.au/mean-well-35w-12v-3a-power-supply/p/MP3285) | $40 |
| [12V DC -> 9V DC voltage regulator](https://core-electronics.com.au/20w-adjustable-dc-dc-buck-converter-with-digital-display.html) | $10 |
| [GY-271 QMC5883 magnometer](https://www.ebay.com.au/sch/i.html?_nkw=QMC5883+GY-271) | $5 |
| [12 wire 5A slip ring 22mm](https://www.ebay.com.au/sch/i.html?_nkw=22mm+12+wire+5A+slip+ring) | $30 |
| 15 AWG wire | $5 |
| 20 AWG wire | $15 |
| Dupont jumper wires | $20 |
| 16x2 LCD + i2c Backpack | $10 |
| 1kg Nylon filament | $60 |
| 1kg PLA filament | $20 |
| [ATGM336H GPS](https://www.ebay.com.au/sch/i.html?_nkw=BDS+GPS+ATGM336H) | $20 |
| Solder breadboard | $15 |
| 3 pin rocker power switch | $10 |
| M3 nuts + screws | $10 |
| PTFE grease | $10 |
| Solder | $5 |
| Heat shrink | $5 |
| Electrical tape | $5 |
| 1m GT2 pulley belt | $10 |
| Superglue | $1 |
| 2.1mm DC barrel jack | $2 |
| Contact Cement | $10 |
| 100 uF capacitor | $2 |

Total cost: ≈ $400

## Tools used

1. Anycubic Kobra S1 3D printer
2. TS80 soldering iron
3. Phillips head screwdriver
4. Precision knife

# Build Example

<p align="center">
  <img src="https://raw.githubusercontent.com/svnty/ISS-arduino-tracker/refs/heads/main/images/img1.png?raw=true" alt="International Space Station Orbit Tracker"/>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/svnty/ISS-arduino-tracker/refs/heads/main/images/img2.png?raw=true" alt="International Space Station Orbit Tracker"/>
</p>

# Credits

Inspired by 
[Grady Hillhouse's ISS tracking pointer](https://github.com/gradyh/ISS-Tracking-Pointer/tree/master), as seen on [Youtube](https://www.youtube.com/watch?v=sIE0mcOGnms).

# Troubleshooting

Can't open the project? Open it in VS-Code and install the `svnty.vscode-arduino-intellisense` VS-Code extension.

# Power usage

Device uses real 3W (0.06A).

# TODO

- Add 3S LiPo support?