# Code/Pi5/static

Files Flask serves verbatim to the dashboard browser. Flask publishes this
folder at `/static/` on its own, because both `stream_server.py` (the robot) and
`dashboard_preview.py` (the UI mock) build their app as `Flask(__name__)` from
`Code/Pi5`, so no route registration is needed.

## bootstrap.min.css

Bootstrap 5.3.3, MIT licensed, byte-identical to upstream:

    curl -sSL -o Code/Pi5/static/bootstrap.min.css \
      https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css

Vendored on purpose. The dashboard is often reached over the Pi's own access
point with no route to the internet, and a CDN `<link>` would silently fail
there. Keep it vendored — do not swap it back for a CDN URL.

`dashboard_page.py` layers the APEX red/black theme on top of this in its inline
`<style>`, and that theme defines every colour, card, button and button row
itself. If this file is ever missing the dashboard still lays out and works; it
just loses Bootstrap's reset, typography and spacing utilities.

The file ends with a `sourceMappingURL` comment pointing at a `.map` that is not
vendored. Browsers only request it with devtools open, where it 404s harmlessly.
It is left in so the file stays a verifiable copy of the upstream release.
