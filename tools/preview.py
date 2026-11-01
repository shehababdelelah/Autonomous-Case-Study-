"""Live preview of the box web pages, without the box.

    python tools/preview.py      then open http://localhost:8000

Pages (each re-read from its source file on every load, whatever branch is checked out):
    /         src/WebHMI.cpp       driving page
    /test     src/TestLoggerWeb.h  zone test logger
    /survey   src/NavSurveyWeb.h   LiDAR survey
The endpoints the pages call answer with simulated data, and the browser reloads when a source
file is saved.
"""
import json, math, os, re, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
MAIN = os.path.join(SRC, "main.cpp")
PORT = 8000

# route -> (source file, name of the C++ raw string delimiter)
PAGES = {
    "/": ("WebHMI.cpp", "rawpage"),
    "/test": ("TestLoggerWeb.h", "HTML"),
    "/survey": ("NavSurveyWeb.h", "HTML"),
}

RELOAD = """<script>
// PREVIEW ONLY: reload when the source file changes.
(function(){let m=null;setInterval(async()=>{try{const t=await (await fetch('/__mtime?p='+location.pathname)).text();
 if(m&&t!==m)location.reload();m=t;}catch(e){}},700);})();
</script>"""

drive = {"l": 0.0, "r": 0.0, "t": 0.0}

# mode -> (name, record seconds, first WP, last WP, expected ramps); same tables as TestLogger.h
ZONES = {0: ("full path", 450, 0, 7, 3), 1: ("B to C", 90, 0, 2, 1), 2: ("F to E", 180, 2, 3, 1),
         3: ("D/E passage to K", 120, 3, 5, 1), 4: ("K to H", 60, 5, 7, 0), 5: ("sensor bench", 600, 0, 0, 0)}
TYPES = ["sensors only", "manual drive", "auto"]
SENSORS = ["imu", "lidar", "us", "gps", "events", "features"]
TEST_SPEED = 0.3
logger = {"rec": None, "mode": 1, "type": 1, "t0": 0.0, "wp": 0, "avg": 0.0, "wps": [], "obs": 0,
          "events": [], "note": "Idle. Choose a zone and press Start.", "runs": {}, "csv": {}}
AVG_S = 5   # the box averages 20 s; shortened here so the preview is quicker to click through


def setting(name, default):
    m = re.search(rf"\b{name}\s*=\s*([\d.]+)", open(MAIN, encoding="utf-8").read())
    return m.group(1).rstrip("f") if m else default


def build_page(route):
    fname, delim = PAGES[route]
    path = os.path.join(SRC, fname)
    if not os.path.exists(path):
        return f"<h1>{fname} is not on this branch</h1><p>Check out a branch that has it.</p>"
    m = re.search(rf'R"{delim}\((.*?)\){delim}"', open(path, encoding="utf-8").read(), re.S)
    if not m:
        return f'<h1>No R"{delim}(...){delim}" string found in {fname}</h1>'
    page = m.group(1)
    page = page.replace("%TITLE%", "CAS Box " + setting("BOX_ID", "8"))
    page = page.replace("%MAXSPD%", "%.2f" % float(setting("MAX_SPEED", "0.5")))
    return page.replace("</body>", RELOAD + "</body>", 1)


def lidar():
    t = time.time()
    d = []
    for i in range(90):  # a 3.6 m x 2.6 m room, a moving obstacle and a gap
        a = math.radians(i * 4)
        r = min(abs(180 / (math.cos(a) or 1e-9)), abs(130 / (math.sin(a) or 1e-9)))
        if 20 < i < 26: r = 60 + 15 * math.sin(t)
        if 60 < i < 64: r = 0
        d.append(round(r))
    return d


def values():
    t = time.time()
    moving = time.time() - drive["t"] < 0.4 and (drive["l"] or drive["r"])
    state = ("Driving L=%.2f R=%.2f" % (drive["l"], drive["r"])) if moving else "Rover: ok (preview, simulated data)"
    return {
        "state": state,
        "sensors": {
            "us": [182, 131, 129, 176, round(95 + 20 * math.sin(t))], "usMax": 400,
            "lidar": {"ok": 1, "pps": 450, "crc": 0, "d": lidar()},
            "imu": {"ok": 1, "yaw": round(t * 3 % 360, 1), "pitch": 0.4, "roll": -1.2, "cal": [3, 3, 3, 2]},
            "gps": {"chars": 1, "fix": 1, "sats": 9, "hdop": 0.9, "lat": 52.520008, "lon": 13.404954},
        },
        "values": [{"name": "rover.ip", "value": "192.168.18.2"}, {"name": "rover.voltage_V", "value": "11.8"},
                   {"name": "rover.rtt_ms", "value": "14"}, {"name": "sys.heap_kB", "value": "182.3"}],
        "cycle": {"min": 2, "avg": 4, "max": 9},
    }


# ---- fake TestLogger (/test/*) ---------------------------------------------------------------
# Writes an events file in the box's format (16 columns), so the Results tab can be tried out.
def ev(event, note="", **cols):
    keys = ["wp", "lat", "lon", "spread_m", "rms_m", "yaw_deg", "samples", "moved",
            "ramp", "pitch_min", "pitch_max", "duration_ms", "first"]
    ms = int((time.time() - logger["t0"]) * 1000)
    logger["events"].append(",".join([str(ms), event] + [str(cols.get(k, "")) for k in keys] + [note]))


def zone_of(mode):
    return 1 if mode == 0 else mode


def test_stop(why="stopped by hand"):
    run = logger["rec"]
    if not run:
        logger["note"] = "Nothing is recording."
        return
    if logger["avg"]:
        logger["avg"] = 0.0
        ev("WP_CANCELLED", "stopped while averaging")
    m = logger["mode"]
    first, last, exp = ZONES[m][2], ZONES[m][3], ZONES[m][4]
    if logger["type"] == 2 and m in (1, 2, 3):   # pretend the route ramp was found
        ev("RAMP", f"route ramp R{m}", ramp=1, pitch_min=-0.4, pitch_max=3.2 if m == 1 else -3.1,
           duration_ms=4200, first="DOWN" if m == 1 else "UP")
    ev("STOP", why)
    ev("ZONE_SUMMARY", f"WP{first}-WP{max(first, logger['wp'] - 1)}; ramps {1 if logger['type'] == 2 and m in (1, 2, 3) else 0} "
                       f"of {exp} expected; GPS fix 92%")
    header = ("t_ms,event,wp,lat,lon,spread_m,rms_m,yaw_deg,samples,moved,"
              "ramp,pitch_min,pitch_max,duration_ms,first,note")
    name = f"{run}_z{zone_of(m)}_events.csv"
    logger["csv"][f"/{run}/{name}"] = header + "\n" + "\n".join(logger["events"]) + "\n"
    sec = time.time() - logger["t0"]
    logger["runs"][run] = [{"n": f"{run}_z{zone_of(m)}_{s}.csv", "s": int(400 + sec * 900)} for s in SENSORS]
    logger["rec"] = None
    logger["note"] = f"Run {run} saved - {why}. Results and files are on the Results tab."


def tick():
    if not logger["rec"]:
        return
    now = time.time()
    if logger["avg"] and now - logger["avg"] >= AVG_S:   # waypoint averaging finished
        logger["avg"] = 0.0
        wp = logger["wp"]
        res = 1 if wp == 1 else 0                       # WP1 always "moved", to show a Fail
        spread = round(0.8 + 0.3 * wp, 1)
        ev("WP", "MOVED - undo and mark again" if res else "ok", wp=wp, lat=52.5200081, lon=13.4049541,
           spread_m=spread, rms_m=round(spread / 2, 2), yaw_deg=round(2.0 + wp, 1), samples=80, moved=res)
        logger["wps"].append({"wp": wp, "res": res, "spread": spread})
        logger["wp"] += 1
        logger["note"] = f"WP{wp}: MOVED - undo and mark again." if res else f"WP{wp} saved, GPS spread {spread} m. Drive on."
    if now - logger["t0"] >= ZONES[logger["mode"]][1] and not logger["avg"]:
        m = ZONES[logger["mode"]][1]
        ev("AUTO_STOP", f"zone time done, {m // 60}:{m % 60:02d}")
        test_stop(f"zone time done")


def test_state():
    tick()
    m = logger["mode"]
    rec = bool(logger["rec"])
    t = time.time()
    return {
        "fs": 1, "rec": int(rec), "run": logger["rec"] or (list(logger["runs"])[-1] if logger["runs"] else ""),
        "mode": m, "zone": zone_of(m), "type": logger["type"],
        "drive": TEST_SPEED if rec and logger["type"] == 2 else 0, "cfgDrive": TEST_SPEED,
        "zoneMs": int((t - logger["t0"]) * 1000) if rec else 0, "totalMs": ZONES[m][1] * 1000,
        "runMs": int((t - logger["t0"]) * 1000) if rec else 0,
        "wp": logger["wp"], "wpFirst": ZONES[m][2], "wpLast": ZONES[m][3],
        "avgLeft": max(0, int(AVG_S - (t - logger["avg"]) + 0.999)) if logger["avg"] else -1,
        "ramps": 0, "rampsExp": ZONES[m][4], "obstacles": logger["obs"], "front": 182, "block": "",
        "obstacleCheck": 0, "yawRef": 1, "yawErr": round(1.5 * math.sin(t / 3), 1), "imu": 1, "fix": 1,
        "freeKb": 1180, "freeMin": 18, "full": 0, "wps": logger["wps"] if rec else [], "note": logger["note"],
    }


def test_status():
    s = test_state()
    if not s["rec"]:
        return "IDLE - choose a zone and press Start\n" + s["note"]
    return f"RECORDING {s['run']} - zone {s['zone']}\nNext: Mark WP{s['wp']}\n" + s["note"]


def test_files():
    tick()
    runs = [{"run": r, "files": f} for r, f in logger["runs"].items()]
    if logger["rec"]:
        z = zone_of(logger["mode"])
        runs.append({"run": logger["rec"], "files": [{"n": f"{logger['rec']}_z{z}_{s}.csv", "s": 512} for s in SENSORS]})
    return {"rec": logger["rec"] or "", "runs": runs}


def test_post(cmd, q):
    arg = lambda k, d="": q.get(k, [d])[0]
    tick()
    if cmd == "start":
        if logger["rec"]:
            logger["note"] = "Stop the current run first."
            return
        m = int(arg("zone", "1"))
        typ = 0 if m == 5 else int(arg("type", "2"))
        prefix = "F-R" if m == 0 else "S-R" if m == 5 else f"Z{m}-R"
        k = 1 + sum(1 for r in logger["runs"] if r.startswith(prefix))
        logger.update(rec=f"{prefix}{k}", mode=m, type=typ, t0=time.time(), wp=ZONES[m][2], avg=0.0,
                      wps=[], obs=0, events=[])
        ev("START", f"run {logger['rec']} zone {zone_of(m)} {ZONES[zone_of(m) if m else 1][0]}")
        ev("TYPE", f"{TYPES[typ]}, forward {TEST_SPEED:.2f}" if typ == 2 else f"{TYPES[typ]}, the logger does not drive")
        ev("PITCH_ZERO", "pitch zero 0.31 deg from 4 samples")
        ev("HEADING_REF", "heading reference 2.0 deg")
        logger["note"] = f"Recording {logger['rec']}. Press Mark at WP{logger['wp']}."
    elif cmd == "stop":
        test_stop()
    elif cmd == "mark":
        if not logger["rec"]: logger["note"] = "Press Start first."
        elif logger["avg"]: logger["note"] = "Busy: still averaging the last waypoint."
        else:
            logger["avg"] = time.time()
            ev("WP_START", f"averaging WP{logger['wp']}")
            logger["note"] = f"Averaging WP{logger['wp']} - keep still."
    elif cmd == "undo" and logger["rec"]:
        if logger["avg"]:
            logger["avg"] = 0.0
            ev("WP_CANCELLED", "undo")
            logger["note"] = "Averaging cancelled."
        elif logger["wp"] > ZONES[logger["mode"]][2]:
            logger["wp"] -= 1
            if logger["wps"] and logger["wps"][-1]["wp"] == logger["wp"]:
                logger["wps"].pop()
            ev("UNDO", f"WP{logger['wp']} removed")
            logger["note"] = f"WP{logger['wp']} removed. Press Mark to record it again."
    elif cmd == "obstacle" and logger["rec"]:
        logger["obs"] += 1
        ev("OBSTACLE", f"logged by hand #{logger['obs']}; front 182 cm; US front 182 cm")
        logger["note"] = f"Obstacle {logger['obs']} logged, front 182 cm."
    elif cmd == "next" and logger["rec"]:
        logger["note"] = "Next zone works only in a full path run." if logger["mode"] else "Next zone (not simulated in the preview)."
    elif cmd == "delete":
        run = arg("run")
        if run == "*":
            logger["runs"].clear(); logger["csv"].clear()
        else:
            logger["runs"].pop(run, None)
            logger["csv"] = {p: c for p, c in logger["csv"].items() if not p.startswith(f"/{run}/")}


def survey_report():
    w = 1200 + int(80 * math.sin(time.time()))
    return ("LIDAR SURVEY   bin 270 = ahead, + bearing = clockwise (right)   [PREVIEW, simulated]\n"
            f"ahead {w:5d}   left   650   right   640   behind  1800  mm\n"
            "path strip +-200 mm: clear   end wall: YES at 1750 mm over 38 deg\n"
            "\nRUNS (jump>150 mm splits, span<3 deg dropped)  3 found\n"
            "  bearing span  near  chord tilt  looks like\n"
            "    -90    40   650   1100    4  wall\n"
            "    +12     5  1150    110   20  pillar?\n"
            "    +90    42   640   1150    3  wall\n"
            "\nOPENINGS (need a boundary each side, min 500 mm)\n"
            "  left : OPEN width 820 mm, -60 to -35 deg\n"
            "  right: none \n")


class Handler(BaseHTTPRequestHandler):
    def _send(self, body, ctype, extra=None):
        data = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in PAGES:
            self._send(build_page(u.path), "text/html; charset=utf-8")
        elif u.path == "/values":
            self._send(json.dumps(values()), "application/json")
        elif u.path == "/drive":
            drive.update(l=float(q.get("l", [0])[0]), r=float(q.get("r", [0])[0]), t=time.time())
            print("[drive] l=%.2f r=%.2f" % (drive["l"], drive["r"]))
            self._send('{"ok":true}', "application/json")
        elif u.path == "/test/status":
            self._send(test_status(), "text/plain")
        elif u.path == "/test/state":
            self._send(json.dumps(test_state()), "application/json")
        elif u.path == "/test/files":
            self._send(json.dumps(test_files()), "application/json")
        elif u.path == "/test/file":
            path = q.get("path", ["/x.csv"])[0]
            body = logger["csv"].get(path, "t_ms,value\n0,0\n250,1\n500,2\n")
            self._send(body, "text/csv", {"Content-Disposition": f'attachment; filename="{os.path.basename(path)}"'})
        elif u.path == "/survey/data":
            self._send(survey_report(), "text/plain")
        elif u.path == "/__mtime":
            route = q.get("p", ["/"])[0]
            path = os.path.join(SRC, PAGES.get(route, PAGES["/"])[0])
            self._send(str(os.path.getmtime(path)) if os.path.exists(path) else "0", "text/plain")
        else:
            self.send_error(404)

    def do_POST(self):
        u = urlparse(self.path)
        if u.path.startswith("/test/"):
            test_post(u.path[len("/test/"):], parse_qs(u.query))
            print("[test]", u.path, u.query)
            self._send("ok", "text/plain")
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    print("Preview: http://localhost:%d  /test  /survey  (save a source file to reload; Ctrl+C to stop)" % PORT)
    ThreadingHTTPServer(("127.0.0.1", PORT), Handler).serve_forever()