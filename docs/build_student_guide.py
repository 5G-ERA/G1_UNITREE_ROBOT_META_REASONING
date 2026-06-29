#!/usr/bin/env python3
"""Build G1_Student_Guide.pdf — the full, self-contained student setup & run guide.
Run:  python docs/build_student_guide.py   (writes G1_Student_Guide.pdf in repo root)
Deps: reportlab.
"""
import os
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Preformatted,
    Table, TableStyle, PageBreak, HRFlowable, ListFlowable, ListItem)

REPO = "https://github.com/5G-ERA/G1_UNITREE_ROBOT_META_REASONING.git"
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "G1_Student_Guide.pdf")

NAVY=colors.HexColor("#1F3B73"); RED=colors.HexColor("#B00000"); GREEN=colors.HexColor("#1a7a33")
GRAY=colors.HexColor("#F1F3F5"); LBLUE=colors.HexColor("#E8F0FE"); LRED=colors.HexColor("#FBE9E9")
LGRN=colors.HexColor("#E8F6EC")
ss=getSampleStyleSheet()
H1=ParagraphStyle("H1",parent=ss["Heading1"],textColor=NAVY,fontSize=15.5,spaceBefore=13,spaceAfter=5)
H2=ParagraphStyle("H2",parent=ss["Heading2"],textColor=NAVY,fontSize=12,spaceBefore=9,spaceAfter=3)
BODY=ParagraphStyle("BODY",parent=ss["BodyText"],fontSize=10,leading=14,spaceAfter=5)
SMALL=ParagraphStyle("SMALL",parent=BODY,fontSize=8.5,textColor=colors.HexColor("#555"))
CODE=ParagraphStyle("CODE",parent=ss["Code"],fontName="Courier",fontSize=8.3,leading=10.6,backColor=GRAY,
    borderColor=colors.HexColor("#D0D4D9"),borderWidth=0.5,borderPadding=6,spaceBefore=3,spaceAfter=7)
TITLE=ParagraphStyle("TITLE",parent=ss["Title"],textColor=NAVY,fontSize=22,leading=26)
SUB=ParagraphStyle("SUB",parent=ss["Title"],textColor=colors.HexColor("#444"),fontSize=12,leading=16)

def P(t,st=BODY): return Paragraph(t,st)
def code(t): return Preformatted(t,CODE)
def bullets(items):
    return ListFlowable([ListItem(Paragraph(i,BODY),leftIndent=10) for i in items],
                        bulletType="bullet",start="•",leftIndent=14,spaceAfter=4)
def callout(t,bg,bc):
    tb=Table([[Paragraph(t,BODY)]],colWidths=[6.6*inch])
    tb.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,-1),bg),("BOX",(0,0),(-1,-1),0.8,bc),
        ("LEFTPADDING",(0,0),(-1,-1),9),("RIGHTPADDING",(0,0),(-1,-1),9),
        ("TOPPADDING",(0,0),(-1,-1),7),("BOTTOMPADDING",(0,0),(-1,-1),7)]))
    return tb
S=[]
def hr(): S.append(HRFlowable(width="100%",thickness=0.6,color=colors.HexColor("#CCCCCC"),spaceBefore=6,spaceAfter=8))

# ---------------- title ----------------
S+=[Spacer(1,0.4*inch),
 P("Autonomous A→B Navigation on the Unitree G1",TITLE),
 P("Student Setup &amp; Run Guide — with GPU Vision",SUB), Spacer(1,8),
 P("This guide takes you from a powered-off robot to running "
   "<font face='Courier'>python g1_goto.py gotoviz B</font> with live GPU perception and the robot's "
   "own performance metrics. Follow the parts in order; each ends with a <b>Checkpoint</b>.",BODY),
 Spacer(1,6),
 callout("<b>Read Part 1 (Safety) before touching the robot.</b> A walking humanoid can fall or strike "
         "someone. One person always holds the remote as a kill switch.",LRED,RED),
 Spacer(1,8),
 P("Two ways to run it",H2),
 bullets([
  "<b>A) All-in-one (Ubuntu, 2× RTX):</b> robot link and vision server on the same PC.",
  "<b>B) Split (recommended if the Ubuntu↔iPhone link is troublesome):</b> robot link on a Mac, heavy "
  "vision on the Ubuntu PC over the LAN.",
 ]),
 P("Same code both ways — only the proxy location (Part B) and <font face='Courier'>G1_PERC</font> "
   "(Part G) differ.",SMALL)]
S.append(PageBreak())

# ---------------- 1 safety ----------------
S+=[P("1 · Safety (mandatory)",H1),
 callout("<b>Kill switch:</b> the operator holds the G1 remote at all times. <b>L2 + B</b> = damping/stop. "
         "Press it the instant anything looks wrong.",LRED,RED), Spacer(1,6),
 bullets([
  "Clear <b>2–3 m</b> of floor. No people in the robot's path during a run.",
  "Battery <b>&gt; 80%</b> (except the deliberate low-battery test). Low battery degrades balance, gait and LiDAR.",
  "Robot <b>standing</b> in walk mode, head level so LiDAR and camera face forward.",
  "Two people: one at the laptop, one beside the robot with the remote.",
  "For payload tests use <b>water, not hot liquid</b>, in an open cup taped so it cannot fall on the electronics.",
  "Stop if the robot spins in place, drifts, or loses localisation.",
 ])]
hr()
# ---------------- 2 need ----------------
S+=[P("2 · What you need",H1),
 bullets([
  "Unitree <b>G1 (\"Air\")</b>, charged &gt; 80%, with its remote.",
  "<b>iPhone</b> with the Unitree app, paired to this robot.",
  "<b>USB cable</b> iPhone ↔ computer.",
  "<b>Ubuntu PC with NVIDIA GPU(s)</b> (all-in-one) and/or a <b>Mac</b>.",
  "A printed <b>checkerboard</b> for camera calibration.",
 ])]
hr()
# ---------------- 3 code+install ----------------
S+=[P("3 · Get the code and install everything",H1),
 P("Do this on the computer that will run navigation. It clones the repository and installs all Python "
   "dependencies into one virtual environment.",BODY),
 code(f"git clone {REPO}\n"
      "cd G1_UNITREE_ROBOT_META_REASONING\n\n"
      "python3 -m venv ~/g1env\n"
      "source ~/g1env/bin/activate          # Windows: ~/g1env/Scripts/activate\n"
      "python -m pip install --upgrade pip\n"
      "pip install -r requirements.txt       # robot link + navigation + metrics + plots"),
 P("System tool for the robot link (one-time, outside pip):",H2),
 code("# macOS:\nbrew install ios-webkit-debug-proxy\n"
      "# Ubuntu:\nsudo apt install -y usbmuxd libimobiledevice-utils ios-webkit-debug-proxy"),
 P("Only on the Ubuntu GPU machine, also install the heavy vision models (Part D needs these):",H2),
 code("pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124   # match your CUDA\n"
      "pip install -r requirements-perception.txt"),
 callout("<b>Checkpoint 3:</b> <font face='Courier'>source ~/g1env/bin/activate</font> then "
         "<font face='Courier'>python -c \"import numpy, matplotlib, websocket, requests, cv2\"</font> runs "
         "with no error.",LGRN,GREEN)]
S.append(PageBreak())
# ---------------- 4 robot/app ----------------
S+=[P("4 · Part A — Prepare the robot and the app",H1),
 bullets([
  "Power on the G1; let it stand in walk mode (remote).",
  "iPhone Unitree app → connect to the robot → <b>SLAM / map screen</b> → <b>load your saved map</b>.",
  "<b>Relocalise</b>: follow the app until the live laser dots line up with the map walls.",
  "Make sure the <b>camera view is ON</b>. Leave the app on that screen (don't open Safari's inspector on it).",
 ]),
 callout("<b>Checkpoint A:</b> robot standing, map loaded, laser dots on the walls (relocalised), camera visible.",LGRN,GREEN)]
hr()
# ---------------- 5 connect ----------------
S+=[P("5 · Part B — Connect the computer to the robot",H1),
 P("Exposes the app's page to Python over USB on port <b>9221</b>.",BODY),
 P("macOS",H2),
 code("# iPhone: Settings → Safari → Advanced → Web Inspector = ON; trust the computer\n"
      "ios_webkit_debug_proxy\n"
      "curl -s http://localhost:9221/json     # should list the device + the app page"),
 P("Ubuntu",H2),
 code("sudo systemctl enable --now usbmuxd\n"
      "idevice_id -l        # device UDID -> USB OK\n"
      "idevicepair pair     # \"SUCCESS: Paired\"\n"
      "# iPhone: Settings → Safari → Advanced → Web Inspector = ON\n"
      "ios_webkit_debug_proxy -c null:9221,:9222-9322 -d\n"
      "curl -s http://localhost:9221/json | head"),
 callout("If Ubuntu shows the device but no app page (newer iOS), build the proxy from source "
         "(<font face='Courier'>docs/SETUP_UBUNTU.md</font> §2) or use run-topology B.",LBLUE,NAVY),
 Spacer(1,4),
 callout("<b>Checkpoint B:</b> <font face='Courier'>curl http://localhost:9221/json</font> shows the app's WebView page.",LGRN,GREEN)]
hr()
# ---------------- 6 link check ----------------
S+=[P("6 · Part C — Quick link check",H1),
 code("source ~/g1env/bin/activate\n"
      "python g1_goto.py reloccheck     # move the robot by hand: x/y/yaw change, laser count > 0, camera frame"),
 callout("<b>Checkpoint C:</b> <font face='Courier'>reloccheck</font> prints a live pose, non-zero laser "
         "points, and a camera frame.",LGRN,GREEN)]
S.append(PageBreak())
# ---------------- 7 perception ----------------
S+=[P("7 · Part D — GPU perception server (Ubuntu)",H1),
 P("Turns the camera frame into depth so the robot sees obstacles the LiDAR misses (e.g. a doorway table). "
   "Run on the GPU machine (installed in Part 3).",BODY),
 P("Prove the pipeline first with no GPU (stub)",H2),
 code("python perception_server.py --stub --port 8008 &\n"
      "curl -s http://localhost:8008/health     # {\"ok\": true, \"mode\": \"stub\", ...}"),
 P("Then the real server (models download on first run)",H2),
 code("python perception_server.py --host 127.0.0.1 --port 8008 \\\n"
      "    --depth depth_anything_v2 --seg segformer --det yolo \\\n"
      "    --fx FX --fy FY --cx CX --cy CY --cam-h H --cam-pitch PITCH\n"
      "curl -s http://localhost:8008/health     # mode \"gpu\", lists the GPU(s)"),
 callout("<b>Checkpoint D:</b> stub <font face='Courier'>/health</font> works, then the real server returns mode \"gpu\".",LGRN,GREEN)]
hr()
# ---------------- 8 calibration ----------------
S+=[P("8 · Part E — Camera calibration",H1),
 P("Optional: send a sharper 640-px frame — in <font face='Courier'>g1_nav_v2.py</font> "
   "<font face='Courier'>CAM_JS</font>, change <font face='Courier'>W=320</font> to "
   "<font face='Courier'>W=640</font>. Calibrate at the same width.",BODY),
 code("python calibrate_cam.py grab 20          # move a checkerboard around the camera view\n"
      "python calibrate_cam.py intrinsics 9 6 25   # 9x6 inner corners, 25 mm squares -> --fx --fy --cx --cy\n"
      "# measure: --cam-h (camera height, m) and --cam-pitch (deg, neg=down; start -10)\n"
      "python calibrate_cam.py rangecheck 127.0.0.1:8008   # box at a measured distance -> compare"),
 callout("<b>Checkpoint E:</b> reproj error &lt; 0.5 px and <font face='Courier'>rangecheck</font> matches a tape measure.",LGRN,GREEN)]
hr()
# ---------------- 9 waypoints ----------------
S+=[P("9 · Part F — Capture the waypoints A and B",H1),
 code("python g1_goto.py waypoint A     # drive the robot to A with the remote, then Ctrl+C to save\n"
      "python g1_goto.py waypoint B     # drive to B, then Ctrl+C\n"
      "python g1_goto.py listwp         # check A and B are stored"),
 callout("<b>Checkpoint F:</b> <font face='Courier'>listwp</font> shows A and B in the current map.",LGRN,GREEN)]
S.append(PageBreak())
# ---------------- 10 run ----------------
S+=[P("10 · Part G — Run the navigation with vision",H1),
 P("Run from a desktop session (the live window needs X11; not headless SSH).",BODY),
 P("All-in-one (Ubuntu)",H2),
 code("export G1_PERC=127.0.0.1:8008\npython g1_goto.py gotoviz B"),
 P("Split (robot on Mac, vision on Ubuntu)",H2),
 code("# Ubuntu:  python perception_server.py --host 0.0.0.0 --port 8008 ...\n"
      "# Mac (use the Ubuntu IP):\nexport G1_PERC=192.168.1.50:8008\npython g1_goto.py gotoviz B"),
 P("What you should see",H2),
 bullets([
  "On start: <font face='Courier'>[perc] http://...:8008 -&gt; OK</font> (if NO RESPONDE, it still runs with the basic heuristic).",
  "A window: map + planned path, live laser, camera, and a metrics strip.",
  "On arrival: <font face='Courier'>LLEGADO a 'B' ... error=... m</font>. Data saved under <font face='Courier'>dataset/</font>.",
 ]),
 P("Headless (no window): <font face='Courier'>python g1_goto.py goto B</font>.",SMALL),
 callout("<b>Checkpoint G:</b> robot drives A→B, console reports LLEGADO, a new file appears in <font face='Courier'>dataset/</font>.",LGRN,GREEN)]
hr()
# ---------------- 11 metrics ----------------
S+=[P("11 · Part H — What the robot tells you about itself",H1),
 P("Each run reports three 0..1 signals so you judge it from data, not by eye:",BODY),
 bullets([
  "<b>clearance</b> — free space ahead (perception). 1 = open, 0 = blocked.",
  "<b>progression</b> — how fast it actually reaches the goal (performance). 1 = full speed, 0 = stalled.",
  "<b>sensing reliability</b> — how much it trusts its own perception (self-capacity): LiDAR noise + "
  "localisation confidence. 1 = stable, low = noisy/unsure.",
 ]),
 P("Live: bottom-right window panel + console (<font face='Courier'>clear= prog= rel=</font>). "
   "Saved in every sample. Picture of a run:",BODY),
 code("python plot_metrics.py dataset/<run>.json    # the three metrics vs time + path coloured by clearance"),
 callout("Read it: all three high = healthy. clearance low AND progression low = stuck on an obstacle. "
         "reliability low = noisy sensing there (where the GPU vision helps).",LBLUE,NAVY)]
S.append(PageBreak())
# ---------------- 12 noise ----------------
S+=[P("12 · Part I — Capture the real sensor noise",H1),
 P("Hold the robot still: any change in the readings is then noise, not motion.",BODY),
 code("python g1_goto.py noisecheck 20      # keep the robot STANDING STILL for 20 s"),
 P("Saves <font face='Courier'>dataset/&lt;ts&gt;_noise.json</font> + a PNG: laser point-count jitter, "
   "forward-clearance noise (m), pose drift while still (cm), localisation/reliability, and a "
   "battery/temperature/motor snapshot.",BODY),
 callout("<b>Checkpoint I:</b> small pose drift (cm) + small clearance noise (m) = good sensing.",LGRN,GREEN)]
hr()
# ---------------- 13 health ----------------
S+=[P("13 · Part J — Hardware health",H1),
 P("Each run logs battery %, battery/CPU temperature, and the temperature + error flag of every joint motor (~1 Hz).",BODY),
 code("python plot_health.py dataset/<run>.json    # battery, temps, and a per-joint motor-temp heatmap"),
 callout("<b>Safety link:</b> if a joint's temperature climbs across runs, or battery drops below ~80%, stop "
         "and rest/charge — both degrade balance and motion.",LBLUE,NAVY)]
hr()
# ---------------- 14 troubleshooting ----------------
S+=[P("14 · Troubleshooting",H1)]
rows=[["Symptom","Likely cause → fix"],
 ["curl localhost:9221/json: no app page","iPhone not trusted / Web Inspector off / iOS too new for the apt proxy. Re-trust; Web Inspector ON; on Ubuntu build the proxy from source (SETUP_UBUNTU §2) or use the split setup."],
 ["reloccheck: empty laser / no pose","App not on the map screen, camera off, or not relocalised. Redo Part A."],
 ["\"RELOCALIZACION DUDOSA ... NO navego\"","Robot thinks it is far from the map. Relocalise until the dots match the walls, then retry."],
 ["[perc] NO RESPONDE","Perception server not running / wrong IP:port. Start it, check /health, verify G1_PERC."],
 ["pip install fails on torch","Install torch with the CUDA-matched --index-url FIRST (Part 3), then requirements-perception.txt."],
 ["Robot spins / wanders","Low battery (&gt;80%), bad relocalisation, or wrong obstacle reading. Stop with L2+B and restart."],
 ["rangecheck distance wrong","Camera not calibrated for the frame size, or wrong height/pitch. Redo Part E at the SAME width as CAM_JS."],
 ["Viz window does not open","Headless SSH. Run from the desktop, or use 'goto' instead of 'gotoviz'."]]
t=Table(rows,colWidths=[2.0*inch,4.6*inch])
t.setStyle(TableStyle([("BACKGROUND",(0,0),(-1,0),NAVY),("TEXTCOLOR",(0,0),(-1,0),colors.white),
 ("FONTNAME",(0,0),(-1,0),"Helvetica-Bold"),("FONTSIZE",(0,0),(-1,-1),8.4),("VALIGN",(0,0),(-1,-1),"TOP"),
 ("GRID",(0,0),(-1,-1),0.5,colors.HexColor("#CCCCCC")),("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.white,GRAY]),
 ("LEFTPADDING",(0,0),(-1,-1),6),("RIGHTPADDING",(0,0),(-1,-1),6),("TOPPADDING",(0,0),(-1,-1),5),("BOTTOMPADDING",(0,0),(-1,-1),5)]))
S+=[t, Spacer(1,12)]
# ---------------- 15 quick ref ----------------
S+=[P("15 · Quick reference",H1),
 code("# 0. Code + venv (once):\n"
      f"git clone {REPO}\n"
      "cd G1_UNITREE_ROBOT_META_REASONING\n"
      "python3 -m venv ~/g1env && source ~/g1env/bin/activate\n"
      "pip install -r requirements.txt        # (+ torch & requirements-perception.txt on the GPU box)\n\n"
      "# 1. App: load map, RELOCALISE, camera ON, standing. Remote in hand (L2+B=stop).\n"
      "# 2. Robot link:\nios_webkit_debug_proxy            # Ubuntu: ... -c null:9221,:9222-9322 -d\n"
      "# 3. Vision (GPU box):\npython perception_server.py --host 127.0.0.1 --port 8008 \\\n"
      "      --depth depth_anything_v2 --seg segformer --det yolo --fx FX --fy FY --cx CX --cy CY --cam-h H --cam-pitch PITCH\n"
      "# 4. Waypoints:\npython g1_goto.py waypoint A   # then B\n"
      "# 5. Run:\nexport G1_PERC=127.0.0.1:8008\npython g1_goto.py gotoviz B\n"
      "# 6. After: python plot_metrics.py dataset/<run>.json ; python plot_health.py dataset/<run>.json"),
 Spacer(1,6),
 P("More detail: <font face='Courier'>docs/SETUP_UBUNTU.md</font>, <font face='Courier'>docs/METRICS.md</font>, "
   "<font face='Courier'>docs/PERCEPTION_UPGRADE.md</font>. Questions: message the instructor.",SMALL)]

def footer(c,d):
    c.saveState(); c.setFont("Helvetica",7.5); c.setFillColor(colors.HexColor("#888"))
    c.drawString(0.9*inch,0.45*inch,"Unitree G1 — Student Setup & Run Guide")
    c.drawRightString(7.6*inch,0.45*inch,"Page %d"%c.getPageNumber()); c.restoreState()
SimpleDocTemplate(OUT,pagesize=letter,topMargin=0.7*inch,bottomMargin=0.7*inch,
                  leftMargin=0.9*inch,rightMargin=0.9*inch,title="G1 Student Setup & Run Guide"
                  ).build(S,onFirstPage=footer,onLaterPages=footer)
print("wrote", OUT)
