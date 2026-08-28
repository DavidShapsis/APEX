"""
dashboard_page.py -- the Mission Control web page markup, in one place.

Pure string. No imports, no ROS, no hardware -- so both stream_server.py (the
real dashboard) and dashboard_preview.py (the throwaway UI mock) render the
*same* page and cannot drift apart.

render() takes only what the server knows at request time. The page's live
state comes from polling /status, exactly as before.

Layout, top to bottom: video feed -> steering -> startup (home/stand/go) ->
obstacle avoidance -> debug. The wordy explanations are tucked behind a small
"i" button on each section heading (toggleInfo), off by default.
"""

# Tokens, not f-string / .format() / Template: the page is full of literal { }
# in its CSS and JavaScript, and a plain replace() sidesteps every escaping
# question.
_PAGE = """
<html>
<head>
    <title>Robodog Mission Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background:#1a1a1a; color:#00ff00; text-align:center; font-family:sans-serif; }
        h1 { letter-spacing:2px; }
        .section { border:1px solid #333; border-radius:8px; padding:12px;
                   margin:14px auto; max-width:440px; }
        .section > h3 { margin:0 0 10px; display:flex; align-items:center;
                        justify-content:center; gap:8px; }
        .info-btn { background:#2a2a2a; color:#0c8; border:1px solid #555;
                    border-radius:50%; width:20px; height:20px; line-height:16px;
                    font-size:12px; font-family:Georgia,'Times New Roman',serif;
                    font-style:italic; padding:0; cursor:pointer; }
        .info { display:none; text-align:left; font-size:0.82em; color:#9a9a9a;
                background:#222; border-radius:6px; padding:10px;
                margin:8px 0 6px; line-height:1.45; }
        .info.show { display:block; }
        .btn { background:#333; color:white; border:1px solid #555; padding:15px;
               margin:5px; width:80px; border-radius:5px; cursor:pointer; }
        .btn:active { background:#00ff00; color:black; }
        .btn:disabled { background:#222; color:#666; cursor:not-allowed; }
        .nav-btn { background:{{NAV_BTN_COLOR}}; color:black; font-weight:bold;
                   width:180px; padding:15px; margin:15px; border-radius:5px;
                   cursor:pointer; border:none; }
        input[type=range] { width: 80%; margin: 16px; }
        .home-btn { background:#333; color:white; border:1px solid #555;
                    padding:12px; margin:5px; width:70px; border-radius:5px;
                    cursor:pointer; }
        .home-btn.homed { background:#0a5; border-color:#0f6; }
        .home-btn:disabled { cursor:not-allowed; }
        .big-btn { padding:15px 25px; margin:8px; border-radius:6px; border:none;
                   cursor:pointer; font-weight:bold; }
        .big-btn:disabled { background:#333; color:#777; cursor:not-allowed; }
        #warnBanner { background:#c00; color:white; padding:10px; font-weight:bold;
                      display:none; max-width:440px; margin:0 auto; border-radius:6px; }
        .deact-btn { background:#333; color:white; border:1px solid #555;
                     padding:10px; margin:5px; width:90px; border-radius:5px;
                     cursor:pointer; }
        .deact-btn.off { background:#a50; border-color:#f80; }
        #avoidBtn { background:#ff0000; color:black; font-weight:bold; width:220px;
                    padding:15px; margin:10px; border-radius:5px; cursor:pointer;
                    border:none; }
        #avoidBtn:disabled { background:#333; color:#777; cursor:not-allowed; }
        #avoidState { font-family:monospace; font-size:0.95em; }
    </style>
    <script>
        function toggleInfo(id) {
            document.getElementById(id).classList.toggle('show');
        }
        function sendDir(val) {
            fetch('/set_direction', {"method": 'POST', "headers": {'Content-Type': 'application/x-www-form-urlencoded'}, "body": 'angle=' + val});
            document.getElementById('angleDisp').innerText = val + '°';
        }
        function toggleNav() {
            fetch('/toggle_nav', {"method": 'POST'})
            .then(response => response.json())
            .then(data => {
                var btn = document.getElementById('navBtn');
                if(data.nav_mode) {
                    btn.style.background = '#00ff00';
                    btn.innerText = 'NAV MODE: ON';
                } else {
                    btn.style.background = '#ff0000';
                    btn.innerText = 'NAV MODE: OFF';
                }
            });
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
            0: ['OFF',      '#888', 'not running'],
            1: ['CLEAR',    '#0f6', 'path clear, navigation steering'],
            2: ['AVOIDING', '#fc0', 'obstacle ahead, committed to a detour'],
            3: ['CLEARING', '#fc0', 'driving past before turning back'],
            4: ['BLOCKED',  '#f44', 'no gap -- marching in place'],
            5: ['ESCAPE',   '#f44', 'arcing to find a way out']
        };
        const LEG_NAMES = ['FL', 'FR', 'RR', 'RL'];
        function refreshStatus() {
            fetch('/status').then(r => r.json()).then(s => {
                const allHomed = s.homed.every(v => v === 1);
                LEG_NAMES.forEach((name, i) => {
                    const btn = document.getElementById('home_' + name);
                    btn.classList.toggle('homed', s.homed[i] === 1);
                    btn.innerText = (s.homed[i] === 1 ? '✓ ' : '') + 'Home ' + name;

                    const deactBtn = document.getElementById('deact_' + name);
                    const isOff = s.deactivated[i] === 1;
                    deactBtn.classList.toggle('off', isOff);
                    deactBtn.innerText = (isOff ? 'Reactivate ' : 'Deactivate ') + name;
                    deactBtn.onclick = () => toggleLeg(i, isOff);
                });
                document.getElementById('standBtn').disabled = !allHomed;
                document.getElementById('goBtn').disabled = !(allHomed && s.standing === 1);
                document.getElementById('goBtn').innerText = s.walking === 1 ? 'WALKING' : 'GO';
                if (s.walking === 1) document.getElementById('goBtn').disabled = true;
                document.getElementById('warnBanner').style.display = allHomed ? 'none' : 'block';

                const avoidBtn = document.getElementById('avoidBtn');
                avoidBtn.disabled = s.avoid_available !== 1;
                if (s.avoid_available !== 1) {
                    avoidBtn.innerText = 'AVOIDANCE: NO MODEL';
                    document.getElementById('avoidState').innerHTML =
                        '<span style="color:#888">Depth model not loaded — run ' +
                        '<code>python3 vision_obstacle.py --download</code> on the Pi.</span>';
                } else {
                    const on = s.avoid_enabled === 1;
                    avoidBtn.style.background = on ? '#00ff00' : '#ff0000';
                    avoidBtn.innerText = on ? 'AVOIDANCE: ON' : 'AVOIDANCE: OFF';
                    const st = AVOID_STATES[s.avoid_state] || AVOID_STATES[0];
                    document.getElementById('avoidState').innerHTML = on
                        ? '<span style="color:' + st[1] + '"><b>' + st[0] + '</b></span> — ' + st[2] +
                          '<br><span style="color:#999">steer ' + (s.avoid_steer >= 0 ? '+' : '') +
                          s.avoid_steer + '° · stride ' + s.avoid_stride + '%</span>'
                        : '<span style="color:#888">off — steering is unmodified</span>';
                }
            }).catch(() => {});
        }
        setInterval(refreshStatus, 1000);
        window.onload = refreshStatus;
    </script>
</head>
<body>
    <h1>ROBODOG VISION</h1>
    <img src='/video_feed' style='width:90%; max-width:600px; border:2px solid #333;'>

    <div class="section">
        <h3>Steering
            <button class="info-btn" onclick="toggleInfo('i_steer')">i</button></h3>
        <div class="info" id="i_steer">Turn command. LEFT / RIGHT spin the robot in
            place; the slider between them arcs &mdash; a small angle keeps most of the
            forward speed, a large one tapers it to a spin. The robot yaws by swinging
            each foot on an arc about its centre, so it only responds once it is walking
            (Home all four legs, then STAND, then GO). NAV MODE hands steering to
            autonomous GPS waypoint following.</div>
        <p>Direction: <span id="angleDisp">0°</span></p>
        <div>
            <button class="btn" onclick="sendDir(-90)">LEFT</button>
            <button class="btn" onclick="sendDir(0)">FWD</button>
            <button class="btn" onclick="sendDir(90)">RIGHT</button>
        </div>
        <input type="range" min="-180" max="180" value="0" oninput="sendDir(this.value)">
        <br>
        <button id="navBtn" class="nav-btn" onclick="toggleNav()">{{NAV_TEXT}}</button>
    </div>

    <div id="warnBanner">⚠ Not every leg is homed &mdash; STAND and GO are disabled until you home all four.</div>

    <div class="section">
        <h3>Startup
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
        <button id="home_FL" class="home-btn" onclick="homeLeg(0)">Home FL</button>
        <button id="home_FR" class="home-btn" onclick="homeLeg(1)">Home FR</button>
        <button id="home_RR" class="home-btn" onclick="homeLeg(2)">Home RR</button>
        <button id="home_RL" class="home-btn" onclick="homeLeg(3)">Home RL</button>
        <br>
        <button id="standBtn" class="big-btn" style="background:#08c;color:white;" onclick="doStand()" disabled>STAND</button>
        <button id="goBtn" class="big-btn" style="background:#0a5;color:white;" onclick="doGo()" disabled>GO</button>
        <button id="stopBtn" class="big-btn" style="background:#c00;color:white;" onclick="doStop()">STOP</button>
    </div>

    <div class="section">
        <h3>Obstacle Avoidance
            <button class="info-btn" onclick="toggleInfo('i_avoid')">i</button></h3>
        <button id="avoidBtn" onclick="toggleAvoid()" disabled>AVOIDANCE: OFF</button>
        <p id="avoidState">&mdash;</p>
        <div class="info" id="i_avoid">Camera-based. Works in manual and NAV mode: it takes
            whichever direction the robot wants to travel and returns the nearest one that
            is actually clear. In NAV mode the GPS bearing is recomputed every pass, so
            once an obstacle is behind it heads for the waypoint again on its own. Fully
            blocked &rarr; stride drops to zero and it marches in place, still standing
            (unlike STOP). While ON, the video feed is overlaid with detection bins
            (red = blocked).</div>
    </div>

    <div class="section">
        <h3>Debug: per-leg deactivation
            <button class="info-btn" onclick="toggleInfo('i_debug')">i</button></h3>
        <div class="info" id="i_debug">Testing only. A deactivated leg is still fully
            computed every gait step, it just receives no wire signal, so it holds its last
            position while the other three keep walking.</div>
        <button id="deact_FL" class="deact-btn" onclick="toggleLeg(0, false)">Deactivate FL</button>
        <button id="deact_FR" class="deact-btn" onclick="toggleLeg(1, false)">Deactivate FR</button>
        <button id="deact_RR" class="deact-btn" onclick="toggleLeg(2, false)">Deactivate RR</button>
        <button id="deact_RL" class="deact-btn" onclick="toggleLeg(3, false)">Deactivate RL</button>
    </div>
    {{EXTRA_BODY}}
</body>
</html>
"""


def render(nav_text="NAV MODE: OFF", nav_btn_color="#ff0000", extra_body=""):
    """Full HTML page. extra_body is injected just before </body> -- the preview
    uses it to add a repaint poke for its non-streaming placeholder feed; the
    real server passes nothing."""
    return (_PAGE
            .replace("{{NAV_TEXT}}", nav_text)
            .replace("{{NAV_BTN_COLOR}}", nav_btn_color)
            .replace("{{EXTRA_BODY}}", extra_body))
