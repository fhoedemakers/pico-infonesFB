# pico-infones with framebuffers

Experimental version using HSTX, uses framebuffers in stead of rendering individual lines to display.

Adafruit Metro RP2350 Only

## ROM

The rom must be compiled into this project. I do not provide any roms. You must provide your own.
Example below is for a rom named `rom.nes` in the root of the project, which will be compiled into the project as `nes_rom.c` using `xxd`. Unix - only, you need to have `xxd` installed.

```bash
sudo apt-get install xxd
xxd -i -n nes_rom Contra.nes | sed -e 's/unsigned/const unsigned/' > nes_rom.c
mkdir build
cd build
cmake ..
make
```

Copy `pico-infones.uf2` to the RP2350 in bootselect mode.

## Controller
The following controllers are supported.

- BUFFALO BGC-FC801
- SONY DUALSHOCK 4
- SONY DualSense

There are several special functions assigned to button combinations.

| Buttton               | Function               |
| --                    | --                     |
| SELECT + START        | Reset the emulator     |
| SELECT + LEFT / RIGHT | Select the next ROM    |
| SELECT + UP / DOWN    | Switch the screen mode |
| SELECT + A / B        | Toggle rapid-fire      |

## Battery backed SRAM
If there is a game with battery-backed memory, 8K bytes per title will be allocated from address 0x10080000 in the reverse direction.
Writing to Flash ROM is done at the timing when reset or ROM selection is made.


