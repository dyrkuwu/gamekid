# gamekid

a virtual machine / emulator / bytecode interpreter for a non-existent gameboy clone from the 1990s

## running the emulator

to run the emulator:
```bash
./gamekid --rom <rom_file> --debug
# or
cargo run -- --rom <rom_file> --debug
```

- `--rom` - optional, loads a specific rom file (otherwise runs the fallback rom)
- `--debug` - optional, enables debug output

## controls

- **space** - play/pause the emulation
- **tab** - advance one cycle (when paused)
- **arrow keys** - directional input (if the game supports it)
- **z** and **x** - action buttons (if the game supports it)

## compiling games

`/gksm` contains a compiler for .gksm files (check `/roms` for examples)

to compile a .gksm file into a rom:
```bash
uv run main.py game.gksm -v
# or
python main.py game.gksm -v
```

- `-v` - optional, verbose output

**important**: you need to compile .gksm files (human readable assembly) into rom files (bytecode binary) before running them in the emulator

## documentation

all documentation for instructions and memory sectors is at the top of `/src/main.rs`
