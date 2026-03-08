# Bouncy Slicer

Arch vase mode G-code generator for FDM 3D printers. Instead of flat layers, the toolpath traces bouncy arch patterns — decorative and structurally interesting.


FYI this is entirely experimental, do not leave your printer running this unattended!!!

**[Try it in your browser](https://laszlokopits.github.io/bouncyslicer/)** — no install needed.

## Features

- Shapes: cylinder, rectangular prism, polygon, or any STL file
- Configurable arch count, height, and segment resolution
- Bottom and top solid layers
- Custom start/end G-code with variable substitution (paste from PrusaSlicer, Cura, etc.)
- 3D preview with layer slider
- Drag-and-drop STL upload
- Runs entirely client-side via Pyodide

## CLI Usage

```bash
python3 bouncy_slicer.py --shape cylinder --radius 30 --height 80 -o vase.gcode
```

With an STL:
```bash
python3 bouncy_slicer.py --stl model.stl -o model.gcode
```

Custom printer G-code:
```bash
python3 bouncy_slicer.py --stl model.stl --start-gcode-file my_start.gcode --end-gcode-file my_end.gcode
```

Run `python3 bouncy_slicer.py --help` for all options.

## Local Development

```bash
python3 -m http.server
# open http://localhost:8000
```

## Tests

```bash
python3 -m pytest test_bouncy_slicer.py -v
```

## Inspiration

Inspired by [@hw_onakaitai's 3D printer having a bouncy day](https://x.com/hw_onakaitai/status/2029887366748659755).
