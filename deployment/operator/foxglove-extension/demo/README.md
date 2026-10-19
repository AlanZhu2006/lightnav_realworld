# Synthetic operator preview

`generate_fixtures.py` creates `live-rgb.jpg`, `live-goal.jpg` and
`fixture-status.json` entirely from constants. Despite the legacy filenames,
they contain no camera image, deployment receipt or machine identity.
`npm run demo:build` regenerates them, then bundles the TypeScript demo.
Preview controls operate only on simulated state and never connect to hardware.

Browser tests use Playwright's installed Chromium. Run `npx playwright install
chromium` once or set `CHROME_PATH` to an existing local Chrome executable. The
tests start only a local ephemeral HTTP server and always stop it afterward.
