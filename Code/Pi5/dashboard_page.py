"""
dashboard_page.py -- the APEX Mission Control web page markup, in one place.

Pure string. No imports, no ROS, no hardware -- so both stream_server.py (the
real dashboard) and dashboard_preview.py (the throwaway UI mock) render the
*same* page and cannot drift apart.

render() takes only what the server knows at request time. The page's live
state comes from polling /status, exactly as before.

Layout, top to bottom: APEX wordmark + status pills -> video feed -> steering
-> startup (home/stand/go) -> obstacle avoidance -> debug. The wordy
explanations are tucked behind a small "i" button on each card heading
(toggleInfo), off by default.

Styling: Bootstrap 5 for the reset, typography, grid and spacing utilities,
with the APEX red/black theme layered on top in the inline <style>. Bootstrap
is served from the robot, not a CDN -- Code/Pi5/static/bootstrap.min.css, which
Flask publishes at /static/ automatically because both servers construct their
app as Flask(__name__) from this directory. Nothing on this page reaches the
internet, so it renders identically with the Pi acting as its own access point
in the field.

The theme is also deliberately self-sufficient -- colours, cards, buttons and
the button rows are all defined here with plain flexbox -- so even if
static/bootstrap.min.css goes missing the page still lays out and reads
correctly, it just loses Bootstrap's polish.

Colour semantics: red = engaged / live, dark grey = idle, amber = degraded.
"""

# Tokens, not f-string / .format() / Template: the page is full of literal { }
# in its CSS and JavaScript, and a plain replace() sidesteps every escaping
# question. Note also that no two braces may end up adjacent in the rendered
# output ("{{" / "}}") -- the verification script treats that as an escaping
# bug -- so nested CSS/JS blocks always close on separate lines.
_PAGE = """
<html lang="en">
<head>
    <meta charset="utf-8">
    <title>APEX Mission Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link href="/static/bootstrap.min.css" rel="stylesheet">
    <style>
        :root {
            --apex-bg:      #08080a;
            --apex-panel:   #141417;
            --apex-panel-2: #0f0f12;
            --apex-line:    #2a2a31;
            --apex-red:     #e01020;
            --apex-red-hi:  #ff2a3a;
            --apex-text:    #e9e9ec;
            --apex-muted:   #8b8b95;
            --apex-amber:   #f0a020;
        }
        html, body { height: 100%; }
        body {
            background:
                radial-gradient(1100px 520px at 50% -160px, rgba(224,16,32,.18), transparent 70%),
                var(--apex-bg);
            background-attachment: fixed;
            color: var(--apex-text);
            font-family: "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            text-align: center;
            padding-bottom: 2.5rem;
        }

        /* ---- header / wordmark ------------------------------------------ */
        .apex-header { padding: 1.6rem 1rem .4rem; }
        .apex-brand {
            font-size: clamp(2.6rem, 12vw, 5.4rem);
            font-weight: 900;
            line-height: 1;
            letter-spacing: .32em;
            text-indent: .32em;   /* cancels the trailing letter-space, keeps it centred */
            color: #fff;
            text-shadow: 0 0 18px rgba(224,16,32,.60), 0 0 52px rgba(224,16,32,.28);
        }
        .apex-tag {
            margin-top: .55rem;
            font-size: .72rem;
            font-weight: 600;
            letter-spacing: .42em;
            text-indent: .42em;
            color: var(--apex-muted);
        }
        .apex-rule {
            height: 3px; width: min(520px, 88%); margin: .9rem auto 0; border-radius: 2px;
            background: linear-gradient(90deg, transparent, var(--apex-red) 18%,
                        var(--apex-red-hi) 50%, var(--apex-red) 82%, transparent);
        }

        /* ---- status pills ------------------------------------------------ */
        .pills { display:flex; flex-wrap:wrap; gap:.5rem; justify-content:center;
                 margin: 1rem auto 0; }
        .pill {
            font-size: .68rem; font-weight: 700; letter-spacing: .14em;
            padding: .34rem .8rem; border-radius: 999px;
            border: 1px solid var(--apex-line); background: #131316;
            color: var(--apex-muted); font-family: ui-monospace, Consolas, monospace;
        }
        .pill.live {
            background: linear-gradient(180deg, var(--apex-red-hi), var(--apex-red));
            border-color: #ff4b57; color: #fff;
            box-shadow: 0 0 14px rgba(224,16,32,.45);
        }

        /* ---- cards -------------------------------------------------------- */
        .apex-wrap { max-width: 560px; margin: 0 auto; padding: 0 .75rem; }
        .apex-card {
            position: relative; overflow: hidden;
            background: linear-gradient(180deg, var(--apex-panel), var(--apex-panel-2));
            border: 1px solid var(--apex-line); border-radius: 14px;
            padding: 1rem .9rem 1.1rem; margin: 1rem 0;
            box-shadow: 0 10px 30px rgba(0,0,0,.55);
        }
        .apex-card::before {
            content: ''; position: absolute; top:0; left:0; right:0; height: 2px;
            background: linear-gradient(90deg, transparent, var(--apex-red), transparent);
        }
        .card-head {
            display: flex; align-items: center; justify-content: center; gap: .55rem;
            font-size: .74rem; font-weight: 800; letter-spacing: .22em;
            text-indent: .22em; text-transform: uppercase;
            color: #d8d8dd; margin: 0 0 .9rem;
        }
        .info-btn {
            background: transparent; color: var(--apex-red-hi);
            border: 1px solid rgba(224,16,32,.55); border-radius: 50%;
            width: 20px; height: 20px; line-height: 1; padding: 0;
            font-size: 12px; font-family: Georgia, "Times New Roman", serif;
            font-style: italic; font-weight: 700; cursor: pointer;
            text-indent: 0; letter-spacing: 0; flex: 0 0 auto;
        }
        .info-btn:hover { background: rgba(224,16,32,.18); color: #fff; }
        .info {
            display: none; text-align: left; font-size: .8rem; line-height: 1.55;
            color: #a0a0aa; background: #0c0c0f; border: 1px solid #222228;
            border-left: 3px solid var(--apex-red); border-radius: 8px;
            padding: .7rem .8rem; margin: 0 0 .9rem;
        }
        .info.show { display: block; }
        .info code { color: #ff8b93; }

        /* ---- video feed ---------------------------------------------------- */
        .feed-frame {
            border: 1px solid var(--apex-line); border-radius: 10px;
            background: #000; overflow: hidden; line-height: 0;
        }
        .feed-frame img { width: 100%; display: block; }

        /* ---- buttons ------------------------------------------------------- */
        .btn-row { display: flex; flex-wrap: wrap; gap: .5rem; justify-content: center; }
        /* Fixed 2x2. These labels swap between "Deactivate FL" and the wider
           "Reactivate FL" at runtime, which is enough to tip a flex row over
           into an ugly 3 + 1 wrap; a grid keeps them square whatever the text. */
        .btn-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: .5rem; }
        .btn-grid .apex-btn { min-width: 0; width: 100%; }
        .apex-btn {
            flex: 1 1 auto; min-width: 84px;
            background: #17171b; color: var(--apex-text);
            border: 1px solid #2e2e36; border-radius: 10px;
            padding: .72rem .9rem; cursor: pointer;
            font-size: .76rem; font-weight: 700; letter-spacing: .1em;
            text-transform: uppercase; transition: all .14s ease;
        }
        .apex-btn:hover:not(:disabled) {
            border-color: var(--apex-red); color: #fff;
            box-shadow: 0 0 0 1px rgba(224,16,32,.35), 0 6px 18px rgba(224,16,32,.20);
        }
        .apex-btn:active:not(:disabled) { transform: translateY(1px); }
        .apex-btn:disabled { opacity: .38; cursor: not-allowed; }
        .apex-btn.primary {
            background: linear-gradient(180deg, var(--apex-red-hi), var(--apex-red));
            border-color: #ff4b57; color: #fff;
            box-shadow: 0 0 16px rgba(224,16,32,.35);
        }
        .apex-btn.stop { background: #240a0d; border-color: #7a1018; color: #ff9098; }
        .apex-btn.stop:hover:not(:disabled) {
            background: var(--apex-red); border-color: #ff4b57; color: #fff;
        }
        .apex-btn.wide { flex: 1 1 100%; padding: .9rem; font-size: .84rem;
                         letter-spacing: .16em; margin-top: .3rem; }

        /* engaged / idle / degraded */
        .apex-btn.on {
            background: linear-gradient(180deg, var(--apex-red-hi), var(--apex-red));
            border-color: #ff4b57; color: #fff;
            box-shadow: 0 0 18px rgba(224,16,32,.45);
        }
        .apex-btn.off { background: #17171b; border-color: #2e2e36; color: var(--apex-muted); }
        .apex-btn.degraded {
            background: #2a1c06; border-color: var(--apex-amber); color: #ffce7a;
        }

        /* ---- steering readout / slider -------------------------------------- */
        .readout {
            font-family: ui-monospace, Consolas, monospace;
            font-size: 1.9rem; font-weight: 700; color: #fff;
            text-shadow: 0 0 16px rgba(224,16,32,.55); line-height: 1;
        }
        .readout-label {
            font-size: .64rem; letter-spacing: .28em; text-indent: .28em;
            color: var(--apex-muted); text-transform: uppercase; margin-bottom: .3rem;
        }
        input[type=range] {
            -webkit-appearance: none; appearance: none;
            width: 92%; height: 6px; margin: 1.1rem auto .4rem; display: block;
            border-radius: 3px; outline: none;
            background: linear-gradient(90deg, #2a2a31, #3a3a44, #2a2a31);
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none; appearance: none;
            width: 22px; height: 22px; border-radius: 50%; cursor: pointer;
            background: linear-gradient(180deg, var(--apex-red-hi), var(--apex-red));
            border: 2px solid rgba(255,255,255,.18);
            box-shadow: 0 0 12px rgba(224,16,32,.75);
        }
        input[type=range]::-moz-range-thumb {
            width: 20px; height: 20px; border-radius: 50%; cursor: pointer;
            background: var(--apex-red); border: 2px solid rgba(255,255,255,.18);
            box-shadow: 0 0 12px rgba(224,16,32,.75);
        }
        .tick-row {
            display: flex; justify-content: space-between; width: 92%; margin: 0 auto;
            font-size: .6rem; letter-spacing: .14em; color: #5c5c66;
            font-family: ui-monospace, Consolas, monospace;
        }

        /* ---- warning banner --------------------------------------------------- */
        #warnBanner {
            display: none; max-width: 560px; margin: 0 auto; border-radius: 10px;
            padding: .75rem .9rem; font-size: .78rem; font-weight: 700;
            letter-spacing: .05em; color: #fff;
            background: repeating-linear-gradient(135deg, #9d0c17 0 14px, #7d0a13 14px 28px);
            border: 1px solid #ff4b57;
        }

        /* ---- avoidance readout -------------------------------------------------- */
        #avoidState {
            font-family: ui-monospace, Consolas, monospace; font-size: .82rem;
            color: var(--apex-muted); background: #0c0c0f; border: 1px solid #222228;
            border-radius: 8px; padding: .6rem .7rem; margin: .8rem 0 0; min-height: 2.6rem;
        }

        .apex-foot {
            margin-top: 1.8rem; font-size: .62rem; letter-spacing: .3em;
            text-indent: .3em; color: #45454e; text-transform: uppercase;
        }

        /* ---- route editor ------------------------------------------------ */
        .wp-row {
            display: grid;
            grid-template-columns: 1.6rem 1fr 1fr auto;
            gap: .4rem; align-items: center; margin-bottom: .45rem;
        }
        .wp-num {
            font-family: ui-monospace, Consolas, monospace; font-size: .78rem;
            color: var(--apex-muted); text-align: right;
        }
        .wp-row.active .wp-num { color: var(--apex-red-hi); font-weight: 700; }
        .wp-input {
            width: 100%; box-sizing: border-box;
            background: #0c0c0f; color: var(--apex-text);
            border: 1px solid #2e2e36; border-radius: 7px;
            padding: .5rem .55rem; font-size: .82rem;
            font-family: ui-monospace, Consolas, monospace;
        }
        .wp-input:focus { outline: none; border-color: var(--apex-red); }
        .wp-input.bad { border-color: var(--apex-amber); }
        .wp-ctl { display: flex; gap: .25rem; }
        .wp-mini {
            background: #17171b; color: var(--apex-text);
            border: 1px solid #2e2e36; border-radius: 6px;
            width: 1.9rem; height: 1.9rem; padding: 0; cursor: pointer;
            font-size: .85rem; line-height: 1;
        }
        .wp-mini:hover:not(:disabled) { border-color: var(--apex-red); color: #fff; }
        .wp-mini:disabled { opacity: .3; cursor: not-allowed; }
        .wp-mini.del:hover { background: var(--apex-red); border-color: #ff4b57; color: #fff; }
        #wpStatus {
            font-family: ui-monospace, Consolas, monospace; font-size: .78rem;
            color: var(--apex-muted); margin: .6rem 0 0;
        }
        #wpSaveState {
            font-size: .72rem; color: var(--apex-amber); margin: .35rem 0 0;
            min-height: 1rem;
        }

        /* ---- collapsible navigation sections ---------------------------- */
        .nav-sec { border-top: 1px solid var(--apex-line); }
        .nav-sec:first-of-type { border-top: none; }
        .nav-sec-head {
            width: 100%; display: flex; align-items: center; gap: .5rem;
            background: transparent; border: none; cursor: pointer;
            color: #d8d8dd; padding: .85rem .2rem; text-align: left;
            font-size: .74rem; font-weight: 800; letter-spacing: .12em;
            text-transform: uppercase;
        }
        .nav-sec-head:hover { color: #fff; }
        .nav-sec-chevron {
            display: inline-block; transition: transform .15s ease;
            color: var(--apex-red-hi); font-size: .8rem; flex: 0 0 auto;
        }
        .nav-sec.open .nav-sec-chevron { transform: rotate(90deg); }
        .nav-sec-title { flex: 1 1 auto; }
        .nav-sec-chip {
            font-family: ui-monospace, Consolas, monospace; font-size: .6rem;
            letter-spacing: .1em; padding: .2rem .5rem; border-radius: 999px;
            border: 1px solid var(--apex-line); color: var(--apex-muted);
            background: #131316; flex: 0 0 auto;
        }
        .nav-sec.open .nav-sec-chip {
            background: linear-gradient(180deg, var(--apex-red-hi), var(--apex-red));
            border-color: #ff4b57; color: #fff;
        }
        .nav-sec-body { display: none; padding: 0 .1rem .9rem; }
        .nav-sec.open .nav-sec-body { display: block; }

        .transport-row { display: flex; gap: .5rem; margin-top: .6rem; }
        .transport-row .apex-btn { flex: 1 1 0; min-width: 0; padding: .7rem .3rem; }
        .apex-btn.go   { background: #0c2a16; border-color: #1c7a3e; color: #7ff0a8; }
        .apex-btn.go:hover:not(:disabled) { background: #1c7a3e; border-color: #2ee06a; color: #fff; }
        .apex-btn.hold { background: #2a2406; border-color: var(--apex-amber); color: #ffce7a; }
        .apex-btn.hold:hover:not(:disabled) { background: #7a6320; border-color: var(--apex-amber); color: #fff; }

        @media (max-width: 380px) {
            .apex-btn { min-width: 72px; font-size: .7rem; padding: .62rem .5rem; }
        }
    </style>
    <script>
        function toggleInfo(id) {
            document.getElementById(id).classList.toggle('show');
        }
        function sendDir(val) {
            fetch('/set_direction', {"method": 'POST', "headers": {'Content-Type': 'application/x-www-form-urlencoded'}, "body": 'angle=' + val});
            document.getElementById('angleDisp').innerText = val + '°';
            // Keep the slider in step with the LEFT / FWD / RIGHT presets.
            // Setting .value programmatically does not re-fire oninput, so this
            // does not loop back through sendDir.
            document.getElementById('dirSlider').value = val;
        }
        function toggleNav() {
            // Master MANUAL/AUTONOMOUS switch. Start/Stop below also move it;
            // refreshStatus() is what keeps the button honest either way.
            fetch('/toggle_nav', {"method": 'POST'}).then(r => r.json()).catch(() => {});
        }
        function navControl(action) {
            fetch('/nav_control', {
                "method": 'POST',
                "headers": {'Content-Type': 'application/x-www-form-urlencoded'},
                "body": 'action=' + action,
            }).then(r => r.json()).then(d => {
                if (!d.ok && d.error) alert(d.error);
            }).catch(() => {});
        }
        function toggleSection(id) {
            document.getElementById(id).classList.toggle('open');
        }
        function homeLeg(id) {
            fetch('/home_leg', {"method": 'POST', "headers": {'Content-Type': 'application/x-www-form-urlencoded'}, "body": 'leg=' + id});
        }
        function doStand() {
            fetch('/stand', {"method": 'POST'}).then(r => r.json()).then(d => {
                if (!d.ok) alert(d.error || 'Cannot stand yet.');
            });
        }
        function doGo() {
            fetch('/go', {"method": 'POST'}).then(r => r.json()).then(d => {
                if (!d.ok) alert(d.error || 'Cannot go yet.');
            });
        }
        function doStop() {
            fetch('/stop', {"method": 'POST'});
        }
        function toggleLeg(id, currentlyDeactivated) {
            const url = currentlyDeactivated ? '/reactivate_leg' : '/deactivate_leg';
            fetch(url, {"method": 'POST', "headers": {'Content-Type': 'application/x-www-form-urlencoded'}, "body": 'leg=' + id});
        }
        function toggleAvoid() {
            fetch('/toggle_avoid', {"method": 'POST'})
            .then(r => r.json())
            .then(d => { if (!d.ok && d.error) alert(d.error); });
        }
        // Must match AvoidState.CODES in vision_obstacle.py.
        const AVOID_STATES = {
            0: ['OFF',      '#8b8b95', 'not running'],
            1: ['CLEAR',    '#4ade80', 'path clear, navigation steering'],
            2: ['AVOIDING', '#f0a020', 'obstacle ahead, committed to a detour'],
            3: ['CLEARING', '#f0a020', 'driving past before turning back'],
            4: ['BLOCKED',  '#ff3b30', 'no gap -- marching in place'],
            5: ['ESCAPE',   '#ff3b30', 'arcing to find a way out']
        };
        const LEG_NAMES = ['FL', 'FR', 'RR', 'RL'];
        function refreshStatus() {
            fetch('/status').then(r => r.json()).then(s => {
                const nHomed = s.homed.filter(v => v === 1).length;
                const allHomed = nHomed === 4;
                LEG_NAMES.forEach((name, i) => {
                    const btn = document.getElementById('home_' + name);
                    btn.classList.toggle('on', s.homed[i] === 1);
                    btn.innerText = (s.homed[i] === 1 ? '✓ ' : '') + 'Home ' + name;

                    const deactBtn = document.getElementById('deact_' + name);
                    const isOff = s.deactivated[i] === 1;
                    deactBtn.classList.toggle('degraded', isOff);
                    deactBtn.innerText = (isOff ? 'Reactivate ' : 'Deactivate ') + name;
                    deactBtn.onclick = () => toggleLeg(i, isOff);
                });

                document.getElementById('pillHomed').innerText = 'HOMED ' + nHomed + '/4';
                document.getElementById('pillHomed').classList.toggle('live', allHomed);
                document.getElementById('pillStand').classList.toggle('live', s.standing === 1);
                document.getElementById('pillWalk').classList.toggle('live', s.walking === 1);

                // ---- Navigation: mode, pause, route progress, transport ----
                const navOn = s.nav_mode === 1;
                const navPaused = s.nav_paused === 1;
                const walking = s.walking === 1;

                const navBtn = document.getElementById('navBtn');
                navBtn.classList.toggle('on', navOn);
                navBtn.classList.toggle('off', !navOn);
                navBtn.innerText = navOn ? 'NAV MODE: ON' : 'NAV MODE: OFF';

                const startBtn = document.getElementById('navStartBtn');
                const pauseBtn = document.getElementById('navPauseBtn');
                const stopBtn2 = document.getElementById('navStopBtn');
                startBtn.innerText = navPaused ? 'Resume' : 'Start';
                startBtn.disabled = !(walking && (!navOn || navPaused));
                pauseBtn.disabled = !(walking && navOn && !navPaused);
                stopBtn2.disabled = !navOn;

                const wpEl = document.getElementById('wpStatus');
                if (wpEl) {
                    let msg;
                    if (!s.wp_total) {
                        msg = 'no route loaded — NAV MODE holds position';
                    } else if (s.wp_index >= s.wp_total) {
                        msg = 'route complete (' + s.wp_total + '/' + s.wp_total + ') — holding position';
                    } else {
                        msg = 'driving to waypoint ' + (s.wp_index + 1) + ' of ' + s.wp_total;
                    }
                    if (navPaused) msg = 'PAUSED — ' + msg;
                    else if (!navOn) msg = 'nav off — ' + msg;
                    wpEl.innerText = msg;
                    if (!wpDirty) wpRender(s.wp_index);
                }

                document.getElementById('standBtn').disabled = !allHomed;
                document.getElementById('goBtn').disabled = !(allHomed && s.standing === 1);
                document.getElementById('goBtn').innerText = s.walking === 1 ? 'WALKING' : 'GO';
                if (s.walking === 1) document.getElementById('goBtn').disabled = true;
                document.getElementById('warnBanner').style.display = allHomed ? 'none' : 'block';

                const avoidBtn = document.getElementById('avoidBtn');
                avoidBtn.disabled = s.avoid_available !== 1;
                if (s.avoid_available !== 1) {
                    avoidBtn.classList.remove('on');
                    avoidBtn.classList.add('off');
                    avoidBtn.innerText = 'AVOIDANCE: NO MODEL';
                    document.getElementById('avoidState').innerHTML =
                        'Depth model not loaded — run ' +
                        '<code>python3 vision_obstacle.py --download</code> on the Pi.';
                } else {
                    const on = s.avoid_enabled === 1;
                    avoidBtn.classList.toggle('on', on);
                    avoidBtn.classList.toggle('off', !on);
                    avoidBtn.innerText = on ? 'AVOIDANCE: ON' : 'AVOIDANCE: OFF';
                    const st = AVOID_STATES[s.avoid_state] || AVOID_STATES[0];
                    document.getElementById('avoidState').innerHTML = on
                        ? '<span style="color:' + st[1] + '"><b>' + st[0] + '</b></span> — ' + st[2] +
                          '<br><span style="color:#6f6f79">steer ' + (s.avoid_steer >= 0 ? '+' : '') +
                          s.avoid_steer + '° · stride ' + s.avoid_stride + '%</span>'
                        : 'off — steering is unmodified';
                }
            }).catch(() => {});
        }
        // ---- GPS route editor ---------------------------------------
        // WP is the browser's working copy: [[lat, lon], ...] as strings while
        // being typed. The robot only sees it when "Send route" is pressed.
        let WP = [];
        let wpDirty = false;

        function wpValidPair(latStr, lonStr) {
            const lat = parseFloat(latStr), lon = parseFloat(lonStr);
            return {
                lat: lat, lon: lon,
                latOk: Number.isFinite(lat) && lat >= -90 && lat <= 90,
                lonOk: Number.isFinite(lon) && lon >= -180 && lon <= 180,
            };
        }
        function wpRender(activeIdx) {
            const list = document.getElementById('wpList');
            list.innerHTML = '';
            WP.forEach((pt, i) => {
                const v = wpValidPair(pt[0], pt[1]);
                const row = document.createElement('div');
                row.className = 'wp-row' + (i === activeIdx ? ' active' : '');
                row.innerHTML =
                    '<span class="wp-num">' + (i + 1) + '</span>' +
                    '<input class="wp-input' + (pt[0] !== '' && !v.latOk ? ' bad' : '') +
                        '" inputmode="decimal" placeholder="latitude" value="' + pt[0] + '" ' +
                        'oninput="wpEdit(' + i + ',0,this.value)">' +
                    '<input class="wp-input' + (pt[1] !== '' && !v.lonOk ? ' bad' : '') +
                        '" inputmode="decimal" placeholder="longitude" value="' + pt[1] + '" ' +
                        'oninput="wpEdit(' + i + ',1,this.value)">' +
                    '<span class="wp-ctl">' +
                        '<button class="wp-mini" title="move up" ' + (i === 0 ? 'disabled' : '') +
                            ' onclick="wpMove(' + i + ',-1)">&#9650;</button>' +
                        '<button class="wp-mini" title="move down" ' +
                            (i === WP.length - 1 ? 'disabled' : '') +
                            ' onclick="wpMove(' + i + ',1)">&#9660;</button>' +
                        '<button class="wp-mini del" title="remove" ' +
                            'onclick="wpRemove(' + i + ')">&#215;</button>' +
                    '</span>';
                list.appendChild(row);
            });
            document.getElementById('wpSend').disabled = !wpDirty;
            document.getElementById('wpSaveState').innerText =
                wpDirty ? 'unsent changes \u2014 press Send route' : '';
        }
        function wpEdit(i, col, val) { WP[i][col] = val.trim(); wpDirty = true; wpRender(); }
        function wpAdd() { WP.push(['', '']); wpDirty = true; wpRender(); }
        function wpRemove(i) { WP.splice(i, 1); wpDirty = true; wpRender(); }
        function wpMove(i, d) {
            const j = i + d;
            if (j < 0 || j >= WP.length) return;
            const tmp = WP[i]; WP[i] = WP[j]; WP[j] = tmp;
            wpDirty = true; wpRender();
        }
        function wpSend() {
            const out = [];
            for (let k = 0; k < WP.length; k++) {
                const v = wpValidPair(WP[k][0], WP[k][1]);
                if (!v.latOk || !v.lonOk) {
                    document.getElementById('wpSaveState').innerText =
                        'point ' + (k + 1) + ' is not a valid latitude/longitude';
                    return;
                }
                out.push([v.lat, v.lon]);
            }
            fetch('/set_waypoints', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({waypoints: out}),
            }).then(r => r.json()).then(d => {
                if (d.ok) {
                    wpDirty = false;
                    document.getElementById('wpSaveState').innerText =
                        'route sent (' + d.count + ' point' + (d.count === 1 ? '' : 's') + ')';
                    wpRender();
                } else {
                    document.getElementById('wpSaveState').innerText = d.error || 'route rejected';
                }
            }).catch(() => {
                document.getElementById('wpSaveState').innerText = 'could not reach the robot';
            });
        }
        function wpLoad() {
            fetch('/waypoints').then(r => r.json()).then(d => {
                if (wpDirty) return;   // do not stomp on what the operator is typing
                WP = (d.waypoints || []).map(pt => [String(pt[0]), String(pt[1])]);
                wpRender();
            }).catch(() => {});
        }

        setInterval(refreshStatus, 1000);
        window.onload = () => { refreshStatus(); wpLoad(); };
        // Both navigation sections start collapsed on purpose.
    </script>
</head>
<body>
    <header class="apex-header">
        <div class="apex-brand">APEX</div>
        <div class="apex-tag">MISSION CONTROL</div>
        <div class="apex-rule"></div>
        <div class="pills">
            <span class="pill" id="pillHomed">HOMED 0/4</span>
            <span class="pill" id="pillStand">STANDING</span>
            <span class="pill" id="pillWalk">WALKING</span>
        </div>
    </header>

    <div class="apex-wrap">

        <div class="apex-card">
            <h3 class="card-head">Live Feed</h3>
            <div class="feed-frame">
                <img src='/video_feed' alt="camera feed">
            </div>
        </div>

        <div class="apex-card">
            <h3 class="card-head">Steering
                <button class="info-btn" onclick="toggleInfo('i_steer')">i</button></h3>
            <div class="info" id="i_steer">Turn command. LEFT and RIGHT are 45° presets
                &mdash; a moderate arc that keeps about half the forward speed. Use the
                slider for anything else: a small angle keeps most of the forward speed,
                and the ±90° ends are a spin in place. The robot yaws by swinging each
                foot on an arc about its centre, so it only responds once it is walking
                (Home all four legs, then STAND, then GO). NAV MODE hands steering to
                autonomous GPS waypoint following.</div>

            <div class="readout-label">Direction</div>
            <div class="readout" id="angleDisp">0°</div>

            <input type="range" id="dirSlider" min="-90" max="90" value="0" oninput="sendDir(this.value)">
            <div class="tick-row"><span>-90</span><span>0</span><span>+90</span></div>

            <div class="btn-row mt-3">
                <button class="apex-btn" onclick="sendDir(-45)">&#9664; Left 45°</button>
                <button class="apex-btn" onclick="sendDir(0)">&#9650; Fwd</button>
                <button class="apex-btn" onclick="sendDir(45)">Right 45° &#9654;</button>
            </div>
        </div>

        <div id="warnBanner">&#9888; Not every leg is homed &mdash; STAND and GO are disabled until you home all four.</div>

        <div class="apex-card">
            <h3 class="card-head">Startup
                <button class="info-btn" onclick="toggleInfo('i_home')">i</button></h3>
            <div class="info" id="i_home">Position each leg by hand &mdash; hip roll 90°,
                knee locked 180°, hip pitch 0° &mdash; then press its Home button.
                Then STAND, then GO.<br><br>
                STOP cuts motor power but keeps encoder tracking, so re-homing is not needed to
                resume. It does NOT lower the robot first &mdash; only use it when the robot is
                off the ground or supported.<br><br>
                Pressing STAND while WALKING stops the walk and settles into the stand pose
                with the legs still powered &mdash; use that, not STOP, to halt a robot that is
                on its feet.</div>
            <div class="btn-row">
                <button id="home_FL" class="apex-btn" onclick="homeLeg(0)">Home FL</button>
                <button id="home_FR" class="apex-btn" onclick="homeLeg(1)">Home FR</button>
                <button id="home_RR" class="apex-btn" onclick="homeLeg(2)">Home RR</button>
                <button id="home_RL" class="apex-btn" onclick="homeLeg(3)">Home RL</button>
            </div>
            <div class="btn-row mt-2">
                <button id="standBtn" class="apex-btn" onclick="doStand()" disabled>Stand</button>
                <button id="goBtn" class="apex-btn primary" onclick="doGo()" disabled>GO</button>
                <button id="stopBtn" class="apex-btn stop" onclick="doStop()">Stop</button>
            </div>
        </div>

        <div class="apex-card">
            <h3 class="card-head">Navigation</h3>

            <div class="nav-sec" id="sec_gps">
                <button class="nav-sec-head" onclick="toggleSection('sec_gps')">
                    <span class="nav-sec-chevron">&#9654;</span>
                    <span class="nav-sec-title">GPS Waypoints &amp; Routing</span>
                    <span class="nav-sec-chip">OFF</span>
                </button>
                <div class="nav-sec-body">
                    <div class="info" id="i_gps">Enter a route as latitude/longitude pairs in
                        decimal degrees (e.g. <code>41.0561</code>, <code>-74.1452</code>) &mdash;
                        south and west are negative. &#9650;/&#9660; reorder, &#215; removes,
                        nothing reaches the robot until <b>Send route</b>. NAV MODE is the master
                        autonomous switch; <b>Start</b> runs the route from the first point,
                        <b>Pause</b> holds position, <b>Stop</b> returns to manual. With no route
                        loaded the robot just marches in place.</div>
                    <button class="info-btn" onclick="toggleInfo('i_gps')"
                            style="align-self:flex-start;margin-bottom:.5rem;">i</button>
                    <button id="navBtn" class="apex-btn wide {{NAV_STATE}}" onclick="toggleNav()">{{NAV_TEXT}}</button>
                    <div id="wpList" style="margin-top:.7rem;"></div>
                    <div class="btn-row">
                        <button class="apex-btn" onclick="wpAdd()">+ Add point</button>
                        <button id="wpSend" class="apex-btn primary" onclick="wpSend()" disabled>Send route</button>
                    </div>
                    <p id="wpStatus">no route loaded &mdash; NAV MODE holds position</p>
                    <p id="wpSaveState"></p>
                    <div class="transport-row">
                        <button id="navStartBtn" class="apex-btn go" onclick="navControl('start')" disabled>Start</button>
                        <button id="navPauseBtn" class="apex-btn hold" onclick="navControl('pause')" disabled>Pause</button>
                        <button id="navStopBtn" class="apex-btn stop" onclick="navControl('stop')" disabled>Stop</button>
                    </div>
                </div>
            </div>

            <div class="nav-sec" id="sec_avoid">
                <button class="nav-sec-head" onclick="toggleSection('sec_avoid')">
                    <span class="nav-sec-chevron">&#9654;</span>
                    <span class="nav-sec-title">Obstacle Avoidance</span>
                    <span class="nav-sec-chip">OFF</span>
                </button>
                <div class="nav-sec-body">
                    <div class="info" id="i_avoid">Camera-based. Works in manual and NAV mode:
                        it takes whichever direction the robot wants to travel and returns the
                        nearest one that is actually clear. Fully blocked &rarr; stride drops to
                        zero and it marches in place, still standing (unlike STOP). While ON, the
                        video feed is overlaid with detection bins (red = blocked).</div>
                    <button class="info-btn" onclick="toggleInfo('i_avoid')"
                            style="align-self:flex-start;margin-bottom:.5rem;">i</button>
                    <button id="avoidBtn" class="apex-btn wide off" onclick="toggleAvoid()" disabled>AVOIDANCE: OFF</button>
                    <p id="avoidState">&mdash;</p>
                </div>
            </div>
        </div>

        <div class="apex-card">
            <h3 class="card-head">Debug &mdash; per-leg deactivation
                <button class="info-btn" onclick="toggleInfo('i_debug')">i</button></h3>
            <div class="info" id="i_debug">Testing only. A deactivated leg is still fully
                computed every gait step, it just receives no wire signal, so it holds its last
                position while the other three keep walking.</div>
            <div class="btn-grid">
                <button id="deact_FL" class="apex-btn" onclick="toggleLeg(0, false)">Deactivate FL</button>
                <button id="deact_FR" class="apex-btn" onclick="toggleLeg(1, false)">Deactivate FR</button>
                <button id="deact_RR" class="apex-btn" onclick="toggleLeg(2, false)">Deactivate RR</button>
                <button id="deact_RL" class="apex-btn" onclick="toggleLeg(3, false)">Deactivate RL</button>
            </div>
        </div>

        <div class="apex-foot">APEX Quadruped</div>
    </div>
    {{EXTRA_BODY}}
</body>
</html>
"""


def render(nav_text="NAV MODE: OFF", nav_btn_color="#ff0000", extra_body="",
           nav_on=None):
    """Full HTML page.

    nav_text     -- label for the NAV MODE button, "NAV MODE: ON"/"OFF".
    nav_btn_color-- kept for backwards compatibility with existing callers and
                    ignored: the button's colour now comes from the theme's
                    on/off classes so it matches the rest of the page. If it is
                    the only thing a caller has, nav_on is derived from
                    nav_text instead.
    extra_body   -- injected just before </body>; the preview uses it to add a
                    repaint poke for its non-streaming placeholder feed, the
                    real server passes nothing.
    nav_on       -- explicit override for the button's initial state.
    """
    if nav_on is None:
        nav_on = nav_text.strip().upper().endswith("ON")
    return (_PAGE
            .replace("{{NAV_TEXT}}", nav_text)
            .replace("{{NAV_STATE}}", "on" if nav_on else "off")
            .replace("{{NAV_BTN_COLOR}}", nav_btn_color)
            .replace("{{EXTRA_BODY}}", extra_body))
