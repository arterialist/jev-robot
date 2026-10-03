# DESIGN.md - JEV robot control room

## Context

- Artifact type: operational dashboard with a conversational control surface
- Positioning: technical and utilitarian
- Audience: the hackathon operator | Primary action: inject a thought and see it become robot motion
- Adjectives: instrumental, lucid, alive, candid
- Visual translations: instrumental means a strict instrument grid and tabular numerals; lucid means three clear cognition layers; alive means fresh state pulses and a real camera feed; candid means actual latency, confidence, and fallback labels stay visible
- Aesthetic essence: flight recorder, clear signals
- Single-minded proposition: you can see and steer the whole cognition loop without stopping physics
- References: Grafana for operational density, flight instrumentation for status hierarchy, Raycast for low-latency restraint
- Mode: dark | Density: dense
- Constraints: plain HTML, CSS, and JavaScript; local FastAPI server; WCAG 2.2 AA; no frontend build step

## Aesthetic

- Direction: Flight Recorder, an industrial control-room treatment with terminal discipline
- Defining trait: thin rules and a strict asymmetric grid organize the screen instead of rounded cards
- Signature move: a vertical cognition trace shows Self-thought -> Gemini -> JEV -> Motion as live state

## Typography

- Display: Barlow Condensed | Google Fonts | OFL
- Body and mono: IBM Plex Mono | Google Fonts | OFL
- Scale: 1.125 major second, base 16px. Display 36px/1; H1 28px/1.1; H2 20px/1.2; body 16px/1.55; small 13px/1.45; label 11px/1.2
- Weights: 400, 500, 600, 700 | Measure: 68ch | Labels use 0.1em tracking and uppercase

## Color

- Strategy: near-black blue graphite makes the live MuJoCo image dominant. Phosphor green only marks live state and the primary action. Signal red and amber retain literal safety meanings.
- Distribution: 60 graphite / 30 raised graphite / 10 signal colors
- Palette:
  - bg: oklch(0.15 0.015 250) | #11161d
  - surface: oklch(0.20 0.018 250) | #1b222c
  - surface-high: oklch(0.25 0.02 250) | #27313d
  - fg: oklch(0.94 0.012 155) | #e9f1ec
  - muted: oklch(0.69 0.024 235) | #9eabb8
  - border: oklch(0.35 0.025 240) | #465463
  - accent: oklch(0.83 0.18 145) | #72ed8c
  - accent-fg: oklch(0.16 0.03 145) | #102417
  - warning: oklch(0.82 0.15 88) | #e7c958
  - error: oklch(0.66 0.21 28) | #ef6154

## Spacing, radius, shadow

- Spacing base: 4px; scale 1, 2, 3, 4, 6, 8, 12
- Radius: 2px and 999px for status dots only
- Shadow approach: defined edges only. No diffuse shadows.

## Layout and composition

- Grid: desktop asymmetric two-column instrument panel, fluid left stage and 420px cognition rail
- Spacing rhythm: 8px inside data groups, 16px between controls, 24px between major regions
- Signature layout move: the cognition trace forms one uninterrupted vertical rail beside the simulation
- Density: dense | Scanning: F pattern
- Responsive: desktop-first; collapse to one column below 960px; preserve 44px controls on touch

## Components and states

- Primary button: phosphor fill. Secondary: graphite fill with defined edge. Tertiary: text button.
- Buttons have hover contrast, 0.98 active scale, 2px accent focus ring, disabled opacity, and a waiting label while disconnected.
- Self-thought input has a persistent label, live status text, Enter submission, and retained text on errors.
- Metrics use tabular numerals and vertical separators, not boxed cards.
- Empty state says what is waiting. Error state names the connection problem and retries automatically.
- Focus ring: `box-shadow: 0 0 0 2px var(--accent)` with 2px offset.

## Motion

- Durations: instant 100ms, fast 150ms, normal 200ms
- Easing: `cubic-bezier(0.23, 1, 0.32, 1)` for entered state
- Only button transforms and connection opacity animate. High-frequency telemetry does not animate.
- Reduced motion removes transforms and retains opacity feedback.

## Iconography and imagery

- Custom geometric status marks on a 16px grid, 1.5px stroke, square joins
- The real MuJoCo stream is the only hero image. No stock or generated decoration.
- Stream has a text alternative and telemetry duplicates its key state.

## Dark mode

- Single designed dark mode. Near-black L 0.15, off-white L 0.94, elevation through L 0.20 and 0.25.
- Accent is a bright but non-glowing green. Borders are lighter than their surfaces.

## Accessibility

- Text and controls target WCAG AA contrast. Focus is visible and never clipped.
- Native buttons and form controls support keyboard use. Targets are at least 44px.
- Labels accompany every status color. The UI remains legible in grayscale.
- Reduced motion is honored. Live updates use polite status regions without announcing every physics tick.

## Tokens

```css
:root {
  --font-display: "Barlow Condensed", "Arial Narrow", sans-serif;
  --font-body: "IBM Plex Mono", ui-monospace, monospace;
  --font-mono: "IBM Plex Mono", ui-monospace, monospace;
  --space-1: 0.25rem;
  --space-2: 0.5rem;
  --space-3: 0.75rem;
  --space-4: 1rem;
  --space-6: 1.5rem;
  --space-8: 2rem;
  --radius: 2px;
  --bg: #11161d;
  --surface: #1b222c;
  --surface-high: #27313d;
  --fg: #e9f1ec;
  --muted: #9eabb8;
  --border: #465463;
  --accent: #72ed8c;
  --accent-fg: #102417;
  --warning: #e7c958;
  --error: #ef6154;
}
```

- Adapter: plain CSS custom properties

## Cards and surfaces

- Panels use one-pixel borders, 2px radius, and 16px to 24px padding. Panels do not nest visually; inner groups use rules and spacing.

## Slop audit

- Date: 2026-09-25 | Result: pass
- No purple gradients, generic rounded card grid, glass effects, diffuse shadows, decorative illustration, or animation on streaming telemetry.
- The cognition trace is specific to this product. Form labels, focus, keyboard behavior, status text, reduced motion, and responsive stacking are included.

## Changelog

- 2026-09-25: Created the browser control-room system for injected first-person self-thought, live simulation, and structured cognition telemetry.
