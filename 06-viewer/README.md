# 06 — Interactive 3-D Viewer & Renders (Ender-MMU)

Offline Three.js r160 interactive 3-D engineering inspector matching the Mach-X template.

```bash
python3 06-viewer/make_viewer.py
python3 06-viewer/serve_viewer.py    # -> http://127.0.0.1:28773/viewer/
```

- `viewer/` — Offline Three.js r160 viewer (`index.html`, `style.css`, `app.js`, `manifest.json`, `vendor/`, `parts/`, `print_stls/`, `media/`)
- `renders/` — Shaded preview PNGs and inside-MMU `colour_change.gif`
- `render_mmu.py` — Optional Blender Cycles studio render script
