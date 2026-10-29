#!/usr/bin/env python3
"""Viser web UI for the Robotiq 2F-85: live 3D view plus control panel.

    ./gripper_web.py            then open the printed URL

The 3D scene shows two grippers: the solid one follows the gripper's measured
position, the translucent one shows the commanded target, so a grip that stops
early on an object is visible as a gap between the two.

All serial traffic happens in a single polling thread. GUI callbacks only drop
requests into a queue / set a target, so Modbus transactions are never
interleaved and no lock is needed around the port.
"""

import argparse
import functools
import queue
import threading
import time
from pathlib import Path

import numpy as np
import viser
import viser.uplot as uplot
import yourdfpy
from viser.extras import ViserUrdf

from robotiq_2f85 import (DEFAULT_PORT, POS_CLOSED, POS_OPEN, STROKE_MM,
                          GripperError, Robotiq2F85)

HERE = Path(__file__).parent
URDF_PATH = HERE / "robotiq_2f85.urdf"
# Meshes are vendored into this folder, so it moves and copies as one unit.
MESH_ROOT = str(HERE)

DRIVER_MAX_RAD = 0.834   # robotiq_2f_85_right_driver_joint upper limit
PLOT_SECONDS = 20.0
POLL_HZ = 15.0


def opening_to_angle(opening_mm: float) -> float:
    """Map finger opening in mm to the URDF's single actuated joint angle."""
    frac = 1.0 - max(0.0, min(STROKE_MM, opening_mm)) / STROKE_MM
    return DRIVER_MAX_RAD * frac


def pos_to_opening(position: int) -> float:
    span = POS_CLOSED - POS_OPEN
    return max(0.0, min(STROKE_MM, STROKE_MM * (POS_CLOSED - position) / span))


def opening_to_pos(opening_mm: float) -> int:
    span = POS_CLOSED - POS_OPEN
    return int(round(POS_CLOSED - span * opening_mm / STROKE_MM))


class GripperApp:
    def __init__(self, port: str, host: str, web_port: int):
        self.gripper = Robotiq2F85(port=port)
        self.commands: queue.Queue = queue.Queue()
        self.target_opening = STROKE_MM
        self.target_dirty = False
        self.speed = 255
        self.force = 100
        self.running = True
        self.last_error = ""

        self.t0 = time.time()
        self.times: list = []
        self.currents: list = []

        self.server = viser.ViserServer(host=host, port=web_port)
        self._build_scene()
        self._build_gui()

    # ---- scene -----------------------------------------------------------
    def _load_urdf(self):
        return yourdfpy.URDF.load(
            str(URDF_PATH),
            filename_handler=functools.partial(
                yourdfpy.filename_handler_magic, dir=MESH_ROOT),
        )

    def _build_scene(self):
        self.server.scene.set_up_direction("+z")
        # measured pose
        self.urdf_actual = ViserUrdf(
            self.server, self._load_urdf(), root_node_name="/actual")
        # commanded pose, translucent
        self.urdf_target = ViserUrdf(
            self.server, self._load_urdf(), root_node_name="/target",
            mesh_color_override=(0.25, 0.55, 1.0, 0.28))
        self.server.scene.add_grid(
            "/grid", width=0.4, height=0.4, position=(0.0, 0.0, -0.02))

    # ---- gui -------------------------------------------------------------
    def _build_gui(self):
        gui = self.server.gui

        with gui.add_folder("Link"):
            self.g_link = gui.add_text("Status", initial_value="connecting...",
                                       disabled=True)
            self.g_fault = gui.add_text("Fault", initial_value="-", disabled=True)

        with gui.add_folder("Control"):
            b_act = gui.add_button("Activate", icon=viser.Icon.PLAYER_PLAY)
            b_open = gui.add_button("Open")
            b_close = gui.add_button("Close")
            b_stop = gui.add_button("Stop", icon=viser.Icon.HAND_STOP)
            self.g_target = gui.add_slider(
                "Target (mm)", min=0.0, max=STROKE_MM, step=0.5,
                initial_value=STROKE_MM)
            self.g_speed = gui.add_slider("Speed", min=0, max=255, step=1,
                                          initial_value=255)
            self.g_force = gui.add_slider("Force", min=0, max=255, step=1,
                                          initial_value=100)

        with gui.add_folder("Measured"):
            self.g_open_mm = gui.add_number("Opening (mm)", initial_value=0.0,
                                            disabled=True)
            self.g_pos = gui.add_number("Raw position", initial_value=0,
                                        disabled=True)
            self.g_cur = gui.add_number("Current (mA)", initial_value=0.0,
                                        disabled=True)
            self.g_obj = gui.add_text("Object", initial_value="-", disabled=True)

        self.g_plot = gui.add_uplot(
            data=(np.zeros(1), np.zeros(1)),
            series=(
                uplot.Series(label="t (s)"),
                uplot.Series(label="current (mA)", stroke="#4f8ff7", width=2),
            ),
            title="Motor current",
            aspect=1.6,
        )

        @b_act.on_click
        def _(_):
            self.commands.put(("activate", None))

        @b_open.on_click
        def _(_):
            self.g_target.value = STROKE_MM

        @b_close.on_click
        def _(_):
            self.g_target.value = 0.0

        @b_stop.on_click
        def _(_):
            self.commands.put(("stop", None))

        @self.g_target.on_update
        def _(_):
            self.target_opening = self.g_target.value
            self.target_dirty = True

        @self.g_speed.on_update
        def _(_):
            self.speed = int(self.g_speed.value)

        @self.g_force.on_update
        def _(_):
            self.force = int(self.g_force.value)

    # ---- polling thread --------------------------------------------------
    def _drain_commands(self):
        while True:
            try:
                kind, _ = self.commands.get_nowait()
            except queue.Empty:
                return
            if kind == "activate":
                self.gripper.activate()
            elif kind == "stop":
                self.gripper.stop()

    def poll_loop(self):
        period = 1.0 / POLL_HZ
        while self.running:
            t_start = time.time()
            try:
                self._drain_commands()
                if self.target_dirty:
                    self.target_dirty = False
                    self.gripper.move(opening_to_pos(self.target_opening),
                                      speed=self.speed, force=self.force,
                                      wait=False)
                st = self.gripper.get_status()
                self._publish(st)
                self.last_error = ""
            except GripperError as e:
                msg = str(e)
                if msg != self.last_error:
                    self.last_error = msg
                    self.g_link.value = "DISCONNECTED"
                    self.g_fault.value = msg[:90]
            except Exception as e:  # serial dropped entirely
                self.g_link.value = f"{type(e).__name__}"
            time.sleep(max(0.0, period - (time.time() - t_start)))

    def _publish(self, st):
        self.urdf_actual.update_cfg(np.array([opening_to_angle(st.opening_mm)]))
        self.urdf_target.update_cfg(
            np.array([opening_to_angle(self.target_opening)]))

        self.g_link.value = "ready" if st.ready else (
            "powered, NOT activated" if st.activated else "not activated")
        self.g_fault.value = st.fault
        self.g_open_mm.value = round(st.opening_mm, 1)
        self.g_pos.value = st.position
        self.g_cur.value = round(st.current_mA)
        self.g_obj.value = ("OBJECT DETECTED" if st.object_detected
                            else ("moving" if st.moving else "at target"))

        t = time.time() - self.t0
        self.times.append(t)
        self.currents.append(st.current_mA)
        while self.times and t - self.times[0] > PLOT_SECONDS:
            self.times.pop(0)
            self.currents.pop(0)
        self.g_plot.data = (np.array(self.times), np.array(self.currents))

    def run(self):
        thread = threading.Thread(target=self.poll_loop, daemon=True)
        thread.start()
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            thread.join(timeout=2.0)
            self.gripper.disconnect()


def main():
    ap = argparse.ArgumentParser(description="Viser web UI for the Robotiq 2F-85")
    ap.add_argument("--port", default=DEFAULT_PORT, help="serial port")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8080)
    args = ap.parse_args()
    GripperApp(args.port, args.host, args.web_port).run()


if __name__ == "__main__":
    main()
