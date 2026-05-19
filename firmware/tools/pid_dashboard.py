"""
================================================================================
  Velocity PID Tuner Dashboard — 1-DOF Robot
  Monitor + Live PID Tuning via Modbus RTU (minimalmodbus 2.x)
================================================================================
  Install:  pip install minimalmodbus dash dash-bootstrap-components plotly
  Run:      python pid_dashboard.py
  Open:     http://127.0.0.1:8050
================================================================================
  Register Map
  ── WRITE (PC → STM32) ──
    0x10  Kp × 100   (uint16)   e.g. Kp=7.0  → 700
    0x11  Ki × 100   (uint16)   e.g. Ki=1.0  → 100
    0x12  Kd × 100   (uint16)   e.g. Kd=0.0  → 0 (ค่า 0 = ไม่อัปเดต)
    0x13  speed × 10 (uint16)   e.g. 3 rad/s → 30
    0x14  period(ms) (uint16)   e.g. 3000 ms
  ── READ (STM32 → PC) ──
    0x20  ref_qd × 100  (int16)
    0x21  qd_out × 100  (int16)
    0x22  V_in   × 100  (int16)
================================================================================
"""

import time
import threading
from collections import deque

import minimalmodbus
import serial
import serial.tools.list_ports

import dash
from dash import dcc, html, Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objs as go

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────────────────────────────────────
SLAVE_ID   = 21
BAUD       = 19200
TIMEOUT_S  = 0.3
POLL_HZ    = 15
MAX_PTS    = 600

REG_KP          = 0x10
REG_KI          = 0x11
REG_KD          = 0x12
REG_TUNE_SPEED  = 0x13
REG_TUNE_PERIOD = 0x14
REG_REF_QD      = 0x20   # อ่าน 3 registers ต่อเนื่อง: ref, act, vin

# ─────────────────────────────────────────────────────────────────────────────
#  SHARED STATE
# ─────────────────────────────────────────────────────────────────────────────
data = {
    "t":   deque(maxlen=MAX_PTS),
    "ref": deque(maxlen=MAX_PTS),
    "act": deque(maxlen=MAX_PTS),
    "vin": deque(maxlen=MAX_PTS),
}
data_lock  = threading.Lock()
instrument = None
connected  = False
conn_port  = ""
last_err   = ""
last_vals  = {"ref": 0.0, "act": 0.0, "vin": 0.0,
              "kp": 7.0,  "ki": 1.0,  "kd": 0.0}
t0 = time.time()


# ─────────────────────────────────────────────────────────────────────────────
#  MODBUS HELPERS
# ─────────────────────────────────────────────────────────────────────────────
def to_signed16(val: int) -> int:
    return val - 65536 if val > 32767 else val


def list_com_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def connect_modbus(port: str):
    global instrument, connected, conn_port, last_err
    try:
        inst = minimalmodbus.Instrument(port, SLAVE_ID)
        inst.serial.baudrate = BAUD
        inst.serial.parity   = serial.PARITY_EVEN
        inst.serial.bytesize = 8
        inst.serial.stopbits = 1
        inst.serial.timeout  = TIMEOUT_S
        inst.mode = minimalmodbus.MODE_RTU
        instrument = inst
        connected  = True
        conn_port  = port
        last_err   = ""
        print(f"[Modbus] Connected: {port} @ {BAUD} 8E1  slave={SLAVE_ID}")
    except Exception as e:
        connected = False
        last_err  = str(e)
        print(f"[Modbus] Connect error: {e}")


def write_pid(kp: float, ki: float, kd: float):
    """เขียน Kp/Ki/Kd ลง STM32 ผ่าน FC16 (write multiple registers)"""
    if not connected or instrument is None:
        return "not connected"
    try:
        kp_raw = max(1, int(kp * 100))
        ki_raw = max(1, int(ki * 100))
        kd_raw = max(1, int(kd * 100))
        # minimalmodbus 2.x: write_registers ใช้ FC16 เป็น default
        instrument.write_registers(REG_KP, [kp_raw, ki_raw, kd_raw])
        last_vals.update(kp=kp, ki=ki, kd=kd)
        print(f"[PID] Kp={kp:.2f}  Ki={ki:.2f}  Kd={kd:.2f}")
        return "ok"
    except Exception as e:
        print(f"[PID] write error: {e}")
        return str(e)


# ─────────────────────────────────────────────────────────────────────────────
#  READ THREAD
# ─────────────────────────────────────────────────────────────────────────────
def read_loop():
    global last_err
    while True:
        if connected and instrument:
            try:
                # อ่าน 3 registers ต่อเนื่อง: 0x20, 0x21, 0x22
                regs = instrument.read_registers(REG_REF_QD, 3, functioncode=3)
                ref = to_signed16(regs[0]) / 100.0
                act = to_signed16(regs[1]) / 100.0
                vin = to_signed16(regs[2]) / 100.0
                t_now = time.time() - t0
                with data_lock:
                    data["t"].append(t_now)
                    data["ref"].append(ref)
                    data["act"].append(act)
                    data["vin"].append(vin)
                last_vals.update(ref=ref, act=act, vin=vin)
                last_err = ""
            except Exception as e:
                last_err = str(e)
        time.sleep(1.0 / POLL_HZ)


# ─────────────────────────────────────────────────────────────────────────────
#  DASH APP
# ─────────────────────────────────────────────────────────────────────────────
app = dash.Dash(__name__,
                external_stylesheets=[dbc.themes.CYBORG],
                title="PID Tuner — 1-DOF Robot")

CLR_BG   = "#0f0f1a"
CLR_CARD = "#1a1a2e"
CLR_BDR  = "#2a2a4e"
CLR_REF  = "#ff6b6b"
CLR_ACT  = "#51cf66"
CLR_VIN  = "#ffd43b"
CLR_TXT  = "#e0e0e0"
CLR_MUT  = "#888888"

CARD = {"backgroundColor": CLR_CARD, "border": f"1px solid {CLR_BDR}",
        "borderRadius": "8px", "padding": "16px", "marginBottom": "12px"}
LBL  = {"color": CLR_MUT, "fontSize": "11px", "fontWeight": "600",
        "letterSpacing": "0.05em", "marginBottom": "4px"}
INP  = {"backgroundColor": "#0f0f1a", "color": CLR_TXT,
        "border": f"1px solid {CLR_BDR}", "borderRadius": "4px"}

GLAYOUT = dict(
    paper_bgcolor=CLR_CARD, plot_bgcolor="#0f0f1a",
    font=dict(color=CLR_TXT, size=11),
    margin=dict(l=52, r=10, t=28, b=30),
    xaxis=dict(gridcolor="#222", color=CLR_MUT, title="Time (s)"),
    yaxis=dict(gridcolor="#222", color=CLR_MUT),
    legend=dict(orientation="h", x=0, y=1.18,
                bgcolor="rgba(0,0,0,0)", font=dict(size=11)),
    hovermode="x unified",
)


def metric(label, mid, unit="", color=CLR_TXT):
    return html.Div([
        html.Div(label, style=LBL),
        html.Div([
            html.Span("—", id=mid,
                      style={"fontSize": "22px", "fontWeight": "700",
                             "color": color}),
            html.Span(f" {unit}", style={"color": CLR_MUT, "fontSize": "12px"}),
        ])
    ], style={"textAlign": "center", "padding": "8px 4px"})


# ── Auto-detect ports for dropdown ──────────────────────────────────────────
_ports = list_com_ports()
_port_opts = [{"label": p, "value": p} for p in _ports] if _ports \
             else [{"label": "ไม่พบ COM port", "value": ""}]

app.layout = dbc.Container(fluid=True,
    style={"backgroundColor": CLR_BG, "minHeight": "100vh", "padding": "16px"},
    children=[

    # HEADER
    dbc.Row([
        dbc.Col(html.H4("Velocity PID Tuner — 1-DOF Robot",
                        style={"color": CLR_TXT, "margin": "0",
                               "fontSize": "18px"}), width="auto"),
        dbc.Col(html.Div(id="hdr-status",
                         style={"color": CLR_MUT, "fontSize": "12px",
                                "textAlign": "right", "paddingTop": "6px"}),
                style={"textAlign": "right"}),
    ], align="center",
       style={"backgroundColor": CLR_CARD, "border": f"1px solid {CLR_BDR}",
              "borderRadius": "8px", "padding": "10px 16px",
              "marginBottom": "14px"}),

    dbc.Row([

        # ── LEFT PANEL ───────────────────────────────────────────────────────
        dbc.Col(width=3, children=[

            # CONNECTION
            html.Div(style=CARD, children=[
                html.Div("MODBUS RTU CONNECTION", style={**LBL, "marginBottom": "10px"}),
                dcc.Dropdown(id="dd-port", options=_port_opts,
                             value=_ports[0] if _ports else "",
                             clearable=False,
                             style={"backgroundColor": "#0f0f1a",
                                    "color": "#000", "marginBottom": "8px",
                                    "fontSize": "12px"}),
                dbc.Row([
                    dbc.Col(dbc.Button("เชื่อมต่อ", id="btn-connect",
                                       color="success", size="sm",
                                       className="w-100"), width=7),
                    dbc.Col(dbc.Button("Refresh", id="btn-refresh",
                                       color="secondary", size="sm",
                                       className="w-100"), width=5),
                ], className="g-1"),
                html.Div(id="conn-status",
                         style={"color": CLR_MUT, "fontSize": "11px",
                                "marginTop": "6px", "textAlign": "center",
                                "minHeight": "16px"}),
            ]),

            # PID GAINS
            html.Div(style=CARD, children=[
                html.Div("VELOCITY PID GAINS", style={**LBL, "marginBottom": "12px"}),

                *[html.Div([
                    dbc.Row([
                        dbc.Col(html.Div(n, style=LBL), width=3),
                        dbc.Col(dbc.Input(id=f"in-{k}", type="number",
                                         value=v, step=s, min=0, max=mx,
                                         style={**INP, "height": "28px",
                                                "padding": "2px 8px"}), width=9),
                    ], align="center", className="mb-1"),
                    dcc.Slider(id=f"sl-{k}", min=0, max=mx, step=s, value=v,
                               marks={0: {"label": "0",
                                          "style": {"color": CLR_MUT}},
                                      mx: {"label": str(mx),
                                           "style": {"color": CLR_MUT}}},
                               tooltip={"placement": "bottom"},
                               className="mb-2"),
                ]) for n, k, v, mx, s in [
                    ("Kp", "kp", 7.0, 30.0, 0.1),
                    ("Ki", "ki", 1.0,  5.0, 0.05),
                    ("Kd", "kd", 0.0,  1.0, 0.01),
                ]],

                dbc.Button("Apply PID", id="btn-pid",
                           color="primary", size="sm", className="w-100 mt-1"),
                html.Div(id="pid-status",
                         style={"color": CLR_ACT, "fontSize": "11px",
                                "marginTop": "6px", "textAlign": "center",
                                "minHeight": "16px"}),
            ]),

            # TUNE PARAMS
            html.Div(style=CARD, children=[
                html.Div("TUNE WAVEFORM", style={**LBL, "marginBottom": "10px"}),
                dbc.Row([
                    dbc.Col(html.Div("Speed (rad/s)", style=LBL), width=6),
                    dbc.Col(html.Div("Period (ms)", style=LBL), width=6),
                ]),
                dbc.Row([
                    dbc.Col(dbc.Input(id="in-speed", type="number",
                                     value=3.0, step=0.5, min=0.5, max=20,
                                     style={**INP, "height": "28px",
                                            "padding": "2px 8px"}), width=6),
                    dbc.Col(dbc.Input(id="in-period", type="number",
                                     value=3000, step=500, min=500, max=10000,
                                     style={**INP, "height": "28px",
                                            "padding": "2px 8px"}), width=6),
                ], className="g-1 mb-2"),
                dbc.Button("Apply Waveform", id="btn-wave",
                           color="warning", size="sm", className="w-100"),
                html.Div(id="wave-status",
                         style={"color": CLR_VIN, "fontSize": "11px",
                                "marginTop": "6px", "textAlign": "center",
                                "minHeight": "16px"}),
            ]),

            # REGISTER MAP
            html.Div(style={**CARD, "fontSize": "10px", "color": CLR_MUT},
                     children=[
                html.Div("REGISTER MAP", style={**LBL, "marginBottom": "8px"}),
                *[html.Div(t) for t in [
                    "WRITE  0x10  Kp × 100",
                    "WRITE  0x11  Ki × 100",
                    "WRITE  0x12  Kd × 100",
                    "WRITE  0x13  speed × 10",
                    "WRITE  0x14  period (ms)",
                    "READ   0x20  ref_qd × 100",
                    "READ   0x21  qd_out × 100",
                    "READ   0x22  V_in   × 100",
                ]],
            ]),
        ]),

        # ── RIGHT PANEL ──────────────────────────────────────────────────────
        dbc.Col(width=9, children=[

            # METRICS ROW
            html.Div(style={**CARD, "marginBottom": "12px"}, children=[
                dbc.Row([
                    dbc.Col(metric("REF VEL",   "m-ref", "rad/s", CLR_REF)),
                    dbc.Col(metric("ACT VEL",   "m-act", "rad/s", CLR_ACT)),
                    dbc.Col(metric("ERROR",      "m-err", "rad/s", "#ff9f43")),
                    dbc.Col(metric("V_IN",       "m-vin", "V",     CLR_VIN)),
                    dbc.Col(metric("Kp", "m-kp", color=CLR_TXT)),
                    dbc.Col(metric("Ki", "m-ki", color=CLR_TXT)),
                    dbc.Col(metric("Kd", "m-kd", color=CLR_TXT)),
                ])
            ]),

            # VELOCITY GRAPH
            html.Div(style={**CARD, "padding": "12px"}, children=[
                dcc.Graph(id="g-vel", style={"height": "300px"},
                          config={"displayModeBar": False}),
            ]),

            # VOLTAGE GRAPH
            html.Div(style={**CARD, "padding": "12px"}, children=[
                dcc.Graph(id="g-vin", style={"height": "160px"},
                          config={"displayModeBar": False}),
            ]),

            # ERROR LOG
            html.Div(id="err-log",
                     style={"color": "#e03131", "fontSize": "11px",
                            "fontFamily": "monospace", "minHeight": "18px",
                            "padding": "4px 8px"}),
        ]),
    ]),

    dcc.Interval(id="tick", interval=100, n_intervals=0),
    dcc.Store(id="store-ports", data=_ports),
])


# ─────────────────────────────────────────────────────────────────────────────
#  CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────

# ── Refresh port list ────────────────────────────────────────────────────────
@app.callback(
    Output("dd-port",     "options"),
    Output("dd-port",     "value"),
    Output("store-ports", "data"),
    Input("btn-refresh",  "n_clicks"),
    prevent_initial_call=True,
)
def refresh_ports(_):
    ports = list_com_ports()
    opts  = [{"label": p, "value": p} for p in ports] \
            if ports else [{"label": "ไม่พบ COM port", "value": ""}]
    return opts, (ports[0] if ports else ""), ports


# ── Connect ──────────────────────────────────────────────────────────────────
@app.callback(
    Output("conn-status", "children"),
    Output("hdr-status",  "children"),
    Input("btn-connect",  "n_clicks"),
    State("dd-port",      "value"),
    prevent_initial_call=True,
)
def on_connect(_, port):
    if not port:
        return (html.Span("ไม่พบ COM port", style={"color": CLR_REF}),
                html.Span("ไม่พบ COM port", style={"color": CLR_REF}))
    connect_modbus(port)
    if connected:
        txt = f"Connected  {port}  {BAUD} 8E1  slave={SLAVE_ID}"
        col = CLR_ACT
    else:
        txt = f"Error: {last_err}"
        col = CLR_REF
    return (html.Span(txt, style={"color": col, "fontSize": "11px"}),
            html.Span(("● " if connected else "✕ ") + txt,
                      style={"color": col, "fontSize": "11px"}))


# ── Slider → Input sync ──────────────────────────────────────────────────────
for _k in ("kp", "ki", "kd"):
    @app.callback(Output(f"in-{_k}", "value"),
                  Input(f"sl-{_k}", "value"),
                  prevent_initial_call=True)
    def _sync(v, __k=_k): return v


# ── Apply PID ────────────────────────────────────────────────────────────────
@app.callback(
    Output("pid-status", "children"),
    Input("btn-pid",     "n_clicks"),
    State("in-kp", "value"),
    State("in-ki", "value"),
    State("in-kd", "value"),
    prevent_initial_call=True,
)
def on_apply_pid(_, kp, ki, kd):
    kp, ki, kd = float(kp or 0), float(ki or 0), float(kd or 0)
    result = write_pid(kp, ki, kd)
    if result == "ok":
        return html.Span(f"OK  Kp={kp:.2f}  Ki={ki:.2f}  Kd={kd:.2f}",
                         style={"color": CLR_ACT})
    return html.Span(f"Error: {result}", style={"color": CLR_REF})


# ── Apply Waveform ───────────────────────────────────────────────────────────
@app.callback(
    Output("wave-status", "children"),
    Input("btn-wave",     "n_clicks"),
    State("in-speed",     "value"),
    State("in-period",    "value"),
    prevent_initial_call=True,
)
def on_apply_wave(_, speed, period):
    if not connected or instrument is None:
        return html.Span("Not connected", style={"color": CLR_REF})
    try:
        spd_raw = max(1, int(float(speed or 3) * 10))
        per_raw = max(100, int(float(period or 3000)))
        instrument.write_registers(REG_TUNE_SPEED, [spd_raw, per_raw])
        return html.Span(f"OK  speed={speed} rad/s  period={period} ms",
                         style={"color": CLR_VIN})
    except Exception as e:
        return html.Span(f"Error: {e}", style={"color": CLR_REF})


# ── Graph + Metrics update ───────────────────────────────────────────────────
@app.callback(
    Output("g-vel",  "figure"),
    Output("g-vin",  "figure"),
    Output("m-ref",  "children"),
    Output("m-act",  "children"),
    Output("m-err",  "children"),
    Output("m-vin",  "children"),
    Output("m-kp",   "children"),
    Output("m-ki",   "children"),
    Output("m-kd",   "children"),
    Output("err-log","children"),
    Input("tick",    "n_intervals"),
)
def update(_):
    with data_lock:
        t   = list(data["t"])
        ref = list(data["ref"])
        act = list(data["act"])
        vin = list(data["vin"])

    # Velocity graph
    fv = go.Figure()
    if t:
        fv.add_trace(go.Scatter(x=t, y=ref, name="ref_qd",
                                line=dict(color=CLR_REF, width=1.5),
                                mode="lines"))
        fv.add_trace(go.Scatter(x=t, y=act, name="qd_out",
                                line=dict(color=CLR_ACT, width=2.0),
                                mode="lines"))
        fv.add_hline(y=0, line_color="#333", line_width=1)

        # ย้าย x window ตามเวลาล่าสุด
        t_win = max(t[-1] - t[0], 10.0)
        fv.update_xaxes(range=[t[-1] - t_win, t[-1]])

    fv.update_layout(**GLAYOUT,
                     yaxis_title="Velocity (rad/s)",
                     title=dict(text="Velocity Tracking",
                                font=dict(size=12, color=CLR_MUT), x=0.01))

    # Voltage graph
    fvin = go.Figure()
    if t:
        fvin.add_trace(go.Scatter(x=t, y=vin, name="V_in",
                                  line=dict(color=CLR_VIN, width=1.5),
                                  fill="tozeroy",
                                  fillcolor="rgba(255,212,59,0.07)",
                                  mode="lines"))
        fvin.add_hline(y= 24, line_color=CLR_REF, line_dash="dot", line_width=1)
        fvin.add_hline(y=-24, line_color=CLR_REF, line_dash="dot", line_width=1)
        fvin.update_xaxes(range=[t[-1] - max(t[-1]-t[0], 10.0), t[-1]])

    vin_layout = {**GLAYOUT,
                  "yaxis": dict(range=[-26, 26], gridcolor="#222",
                                color=CLR_MUT, title="V_in (V)"),
                  "title": dict(text="Motor Voltage",
                                font=dict(size=12, color=CLR_MUT), x=0.01)}
    fvin.update_layout(**vin_layout)

    rv  = last_vals["ref"]
    av  = last_vals["act"]
    vv  = last_vals["vin"]
    err = rv - av

    def f(v, d=2): return f"{v:+.{d}f}"

    err_span = html.Span(f(err),
                         style={"color": CLR_REF if abs(err) > 0.5 else CLR_ACT})

    err_log = (f"[poll error] {last_err}") if last_err else ""

    return (
        fv, fvin,
        f(rv), f(av),
        err_span,
        f(vv, 1),
        f"{last_vals['kp']:.1f}",
        f"{last_vals['ki']:.2f}",
        f"{last_vals['kd']:.2f}",
        err_log,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 64)
    print("  Velocity PID Tuner Dashboard  (Modbus RTU / minimalmodbus)")
    print("=" * 64)
    ports = list_com_ports()
    print(f"  COM ports: {ports}")
    print()
    print("  1. Flash firmware ลง STM32 ก่อน (CubeIDE → Run)")
    print("  2. เปิด browser:  http://127.0.0.1:8050")
    print("  3. เลือก COM port แล้วกด 'เชื่อมต่อ'")
    print()

    threading.Thread(target=read_loop, daemon=True).start()
    app.run(debug=False, host="127.0.0.1", port=8050)
