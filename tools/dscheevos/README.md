# DScheevos additions

This branch adds a few things to melonDS for testing DScheevos code without real hardware.

## Headless runner (`headless/`)

Runs a `.nds` in DS or DSi mode with no window. Test programs print lines that start with `@@` through the no$gba debug port (libnds `nocashMessage`); the runner echoes them and can stop once a line contains a marker.

```sh
cmake -S headless -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
build/melonds-headless --rom test.nds --until DONE
build/melonds-headless --rom test.nds --mode dsi --bios-dir DIR --nand nand-copy.bin --until DONE
```

With `--net`, the emulated Wi-Fi joins melonDS's built-in access point and reaches the network through a user-mode stack (libslirp, bundled with melonDS). The console gets an address by DHCP, and the host machine is reachable at `10.64.0.1`, so a test server running on the host is easy to reach. When a real firmware dump is used, the runner points its saved Wi-Fi settings at that access point in memory; the file isn't changed. Needs libenet installed.

`--arg` passes command-line arguments to a direct-booted program the way homebrew loaders do; nds-bootstrap needs its own path as the first one (for example `--arg sd:/_nds/nds-bootstrap-hb-release.nds`). The SD card is a 1 GB FAT32 image synced from the `--sd` folder.

DS mode works without any console files (melonDS's free BIOS). DSi mode needs your own BIOS, firmware and NAND dumps in `--bios-dir`: `biosnds7.bin`, `biosnds9.bin`, `dsfirmware.bin`, `biosdsi7.bin`, `biosdsi9.bin`, `dsifirmware.bin`. Pass a copy of the NAND, since melonDS writes to it. Run with no arguments for all options.

## Profile (`--profile`) and write watch (`--watch-write`)

`--profile` counts, per VBlank interval, the ARM9 cycles spent inside the DScheevos module (its VBlank call and its idle work) and whether the game reached idle, and prints a summary at exit (`--profile-log` writes one CSV line per interval). The DScheevos repo's `tests/perf/AB.md` explains the numbers and the lag definition. The hook is `ARMv5::ProfileInstr` in `src/ARM.cpp`, called after each interpreted instruction, plus a callback at the start of VBlank in `src/GPU.cpp`; both are off unless a frontend sets `ARM9.ProfOn`.

`--watch-write ADDR:SIZE` prints the bus writes (CPU or DMA) to a range in DSi mode, with the frame and the CPU's PC. The hook is in `DSi::ARM9Write*` and `DSi::ARM7Write*` (`NDS::WatchHook`), off unless set. Mirrors are separate addresses: main RAM written through 0x0C000000 needs its own watch.

## DSi direct boot fix

melonDS's DSi direct boot ignored the ROM header field (0x1B8) where a program asks for SD card access, so DSi homebrew such as nds-bootstrap couldn't see the SD card. The fix in `src/DSi.cpp` grants it when the header asks, as the DSi's own launcher does.

## Cache timing model (`--cache-model`)

Stock melonDS charges a flat cost for every access to cached memory, hit or miss. With `--cache-model`, the ARM9 tracks which lines its caches hold (8 KB instruction, 4 KB data, 4-way, 32-byte lines), so a hit costs 1 cycle and a miss costs a line fill at the memory's bus speed, plus a write-back for a dirty line. Stores don't allocate. Cache maintenance operations clear the model. It only changes timing; the data the CPU sees is the same.

The change is in `src/CP15.cpp` and `src/ARM.h`, and it's off unless a frontend sets `ARM9.CacheModel`.

With nds-bootstrap on DSi, the model on from boot makes nds-bootstrap's start hang (the ARM9 waits in the BIOS; not investigated). `--cache-model-from-game` (with `--profile`) switches it on at the game's first `OS_Halt` call instead, starting from empty caches.

## Cache benchmark (`cachebench/`)

A small `.nds` that prints cycles per load for patterns with known cache behavior: hits, misses, sequential and uncached. Build it with devkitARM (`make`). It prints to the screen too, so the same file can be compared on real hardware.
