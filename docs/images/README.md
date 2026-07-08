# Images for the documentation

Drop the image files here with these exact names so they show up in
[`../Getting_Started.md`](../Getting_Started.md). All are referenced as
`images/<name>` from that guide.

| File | What it should show | Good source to shoot/redraw from |
|------|---------------------|----------------------------------|
| `board_overview.png` | The MKS/ODrive board with ST‑Link, motor and PSU connected, labelled | your own bench photo |
| `platformio_install.png` | The "PlatformIO IDE" extension in VS Code's Extensions panel | screenshot of VS Code |
| `stlink_wiring.png` | ST‑Link → SWD hookup (GND, SWDIO/PA13, SWCLK/PA14, optional 3.3 V) | [stm32‑base: connecting your debugger](https://stm32-base.org/guides/connecting-your-debugger.html) |
| `platformio_toolbar.png` | PlatformIO status bar with ✓ (build) and → (upload) icons | screenshot of VS Code |
| `can_wiring.png` | MCP2515 ↔ board CANH/CANL/GND + two 120 Ω terminators | [MCP2515 tutorial](https://lastminuteengineers.com/mcp2515-can-module-arduino-tutorial/) |

PNG or JPG both work; keep them under ~1 MB so the repo stays light. If you
rename a file, update the matching `![...](images/...)` line in
`Getting_Started.md`.
