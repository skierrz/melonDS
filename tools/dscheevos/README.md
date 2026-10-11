# DScheevos additions

This branch adds two things to melonDS for testing DScheevos code without real hardware.

## Headless runner (`headless/`)

Runs a `.nds` in DS or DSi mode with no window. Test programs print lines that start with `@@` through the no$gba debug port (libnds `nocashMessage`); the runner echoes them and can stop once a line contains a marker.

```sh
cmake -S headless -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
build/melonds-headless --rom test.nds --until DONE
build/melonds-headless --rom test.nds --mode dsi --bios-dir DIR --nand nand-copy.bin --until DONE
```

DS mode works without any console files (melonDS's free BIOS). DSi mode needs your own BIOS, firmware and NAND dumps in `--bios-dir`: `biosnds7.bin`, `biosnds9.bin`, `dsfirmware.bin`, `biosdsi7.bin`, `biosdsi9.bin`, `dsifirmware.bin`. Pass a copy of the NAND, since melonDS writes to it. Run with no arguments for all options.

## Cache timing model (`--cache-model`)

Stock melonDS charges a flat cost for every access to cached memory, hit or miss. With `--cache-model`, the ARM9 tracks which lines its caches hold (8 KB instruction, 4 KB data, 4-way, 32-byte lines), so a hit costs 1 cycle and a miss costs a line fill at the memory's bus speed, plus a write-back for a dirty line. Stores don't allocate. Cache maintenance operations clear the model. It only changes timing; the data the CPU sees is the same.

The change is in `src/CP15.cpp` and `src/ARM.h`, and it's off unless a frontend sets `ARM9.CacheModel`.

## Cache benchmark (`cachebench/`)

A small `.nds` that prints cycles per load for patterns with known cache behavior: hits, misses, sequential and uncached. Build it with devkitARM (`make`). It prints to the screen too, so the same file can be compared on real hardware.
