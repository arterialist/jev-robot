# Repository images

The images show the local simulator in offline mode. The model loops use deterministic fallback decisions, so these captures are evidence of the simulator and interface, not successful JEV or Gemini API calls.

`control-room.png` is a full-page browser screenshot of the control room with the Arena camera selected. It shows the G1 near the green goal, the map, measured loop rates, self-thought controls, and cognition trace. The dashboard labels both model loops `LOCAL` and reports zero model throughput.

`g1-arena.png` is a frame from the same run with the Follow camera selected. It shows the upright G1, physical obstacles, arena walls, and green goal marker. The capture preserves the rendered frame; converting its JPEG stream frame to PNG does not add detail.

`social-preview.png` is a direct MuJoCo render at 1280 by 640 pixels for GitHub's social preview. It uses the local runtime's reported position, orientation, and all 29 joint positions in the same scene with the Follow camera. MuJoCo renders the image at its final dimensions; the image has no added illustration or raster resizing.

Run the captured interface with:

```bash
uv run jev-robot-web --offline --no-open --port 8000
```

Open `http://127.0.0.1:8000`, select a camera, and capture the page or a frame from `/stream.mjpeg`. The state and rates recorded beside the Follow frame are in `capture-metadata.json`. They describe one local run and do not establish a performance guarantee.
