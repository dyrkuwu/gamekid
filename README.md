# GameKid

a virtual Machine / emulator / bytecode interpreter for a non-existent gameboy clone from 1990s

`/gksm` contains a compiler for .gksm files (check `/roms` for examples)

to run use ```(./gamekid / cargo run --``` --rom (optional, otherwise runs the fallback rom) --debug (optional)```
(!) important, to run a rom you need to compile gksm (human readable assembly) into a rom (bytecode binary) with the gksm compiler

to run gksm compiler, use ```(uv run / python) main.py game.gksm -v (optional)```

all documentation for instructions / memory sectors is at the top of `/src/main.rs`
