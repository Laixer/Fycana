#!/usr/bin/env python3

import os
import time
import math

from pyglonax.excavator import Excavator, ExcavatorAdapter
from pyglonax.util import get_config

from rich.table import Table
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.console import Group
from rich.align import Align
from rich.text import Text
from rich import box

config = get_config()

excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])
adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])


def format_angle(value=None) -> Text:
    text = Text()

    if value is None:
        text.append("inf", style="bright_black")
        return text

    text.append("{:8.3f}".format(value), style="chartreuse2")
    text.append("rad", style="bright_black")
    text.append(" ")
    text.append("{:7.2f}".format(math.degrees(value)), style="yellow1")
    text.append("°", style="bright_black")

    return text


def format_percent(value=None) -> Text:
    text = Text()

    if value is None:
        return text

    text.append(
        "{:>.1f}".format(value), style="red3" if value > 100 or value < 0 else "white"
    )
    text.append("%", style="bright_black")

    return text


def format_coord(value) -> Text:
    text = Text(style="grey62")

    if value is None:
        return text

    text.append("{:5.2f}".format(value))

    return text


def make_layout() -> Layout:
    """Define the layout."""

    layout = Layout(name="root")

    layout.split(
        Layout(name="header", size=3),
        Layout(name="main", ratio=1),
        Layout(name="footer", size=3),
    )
    layout["main"].split_row(
        Layout(name="side"), Layout(name="body", ratio=3, minimum_size=50)
    )
    layout["side"].split(
        Layout(name="engine"), Layout(name="vms"), Layout(name="location")
    )
    layout["body"].split(
        Layout(name="encoder"), Layout(name="origin"), Layout(name="effector")
    )

    return layout


class Header:
    """Display header connection status and host."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(justify="left")
        grid.add_column(justify="center", ratio=1)
        grid.add_column(justify="right")

        grid.add_row(
            "[green3]Connected"
            if adapter.status == adapter.ConnectionState.CONNECTED
            else "[red3]Disconnected",
            f"{excavator.name} [b]{excavator.model}[/b]",
            adapter.host,
        )

        return Panel(grid, style="on grey11")


class Footer:
    """Display header connection status and host."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(justify="left")
        grid.add_column(justify="center", ratio=1)
        grid.add_column(justify="right")

        grid.add_row(
            "Laixer Equipment B.V.", "", "{:.3f}s".format(adapter.signal_elapsed_time)
        )

        return Panel(grid, style="grey62 on grey3")


class EnginePanel:
    """Display engine parameters."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(ratio=1)
        grid.add_column(justify="right")

        if "driver_demand" in adapter.engine:
            grid.add_row(
                "Driver demand", format_percent(adapter.engine["driver_demand"])
            )
        else:
            grid.add_row("Driver demand")
        if "actual_engine" in adapter.engine:
            grid.add_row(
                "Actual engine", format_percent(adapter.engine["actual_engine"])
            )
        else:
            grid.add_row("Actual engine")
        if "rpm" in adapter.engine:
            grid.add_row("RPM", str(adapter.engine["rpm"]), style="bold bright_yellow")
        else:
            grid.add_row("RPM", style="bold bright_yellow")

        return Panel(grid, title="[magenta3][ Engine ]", style="on grey11")


class VMSPanel:
    """Display engine parameters."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)

        grid.add_column(ratio=1)
        grid.add_column(justify="right", width=5)
        grid.add_column(justify="right", width=5)
        grid.add_column(justify="right", width=5)

        if "cpu_1" in adapter.vms:
            grid.add_row(
                "CPU",
                format_percent(adapter.vms["cpu_1"]),
                format_percent(adapter.vms["cpu_5"]),
                format_percent(adapter.vms["cpu_15"]),
            )
        else:
            grid.add_row("CPU")
        if "memory" in adapter.vms:
            grid.add_row("Memory", "", "", format_percent(adapter.vms["memory"]))
        else:
            grid.add_row("Memory")
        if "swap" in adapter.vms:
            grid.add_row("Swap", "", "", format_percent(adapter.vms["swap"]))
        else:
            grid.add_row("Swap")

        return Panel(grid, title="[magenta3][ VMS ]", style="on grey11")


class EncoderTable:
    def __rich__(self):
        table = Table(box=box.MINIMAL, pad_edge=False, show_edge=False, expand=True)

        table.add_column("Joint Encoder", no_wrap=True, min_width=10)
        table.add_column("Position", justify="right", style="grey66")
        table.add_column("Bounds Lower", justify="right")
        table.add_column("Relative Angle", justify="right")
        table.add_column("Percentage", justify="right")
        table.add_column("Bounds Upper", justify="right")

        for joint in excavator.joints:
            encoder_name = joint.name[:-6]
            if encoder_name in adapter.encoder:
                normal = joint.normalize(adapter.encoder[encoder_name]["angle"])

                table.add_row(
                    joint.name,
                    "{:3.3f}".format(adapter.encoder[encoder_name]["position"]),
                    format_angle(joint.lower_bound),
                    format_angle(adapter.encoder[encoder_name]["angle"]),
                    format_percent(normal * 100),
                    format_angle(joint.upper_bound),
                )

        return table


class OriginGrid:
    def __rich__(self) -> Table:
        grid = Table(box=None, pad_edge=False, show_edge=False, expand=True)

        grid.add_column()
        grid.add_column("X", justify="right", width=5)
        grid.add_column("Y", justify="right", width=5)
        grid.add_column("Z", justify="right", width=5)
        grid.add_column("Roll", justify="right", width=20)
        grid.add_column("Pitch", justify="right", width=20)
        grid.add_column("Yaw", justify="right", width=20)

        for idx, joint in enumerate(excavator.joints):
            grid.add_row(
                (" " * idx) + joint.name,
                format_coord(joint.origin_translation[0])
                if joint.origin_translation is not None
                else "-",
                format_coord(joint.origin_translation[1])
                if joint.origin_translation is not None
                else "-",
                format_coord(joint.origin_translation[2])
                if joint.origin_translation is not None
                else "-",
                format_angle(joint.origin_orientation[0])
                if joint.origin_orientation is not None
                else "-",
                format_angle(joint.origin_orientation[1])
                if joint.origin_orientation is not None
                else "-",
                format_angle(joint.origin_orientation[2])
                if joint.origin_orientation is not None
                else "-",
                style="bold" if joint.type != "fixed" else "",
            )

        return grid


class KinematicGrid:
    def __rich__(self) -> Table:
        grid = Table(box=None, pad_edge=False, show_edge=False, expand=True)

        grid.add_column()
        grid.add_column("X", justify="right", width=5)
        grid.add_column("Y", justify="right", width=5)
        grid.add_column("Z", justify="right", width=5)
        grid.add_column("Absolute Roll", justify="right", width=20)
        grid.add_column("Absolute Pitch", justify="right", width=20)
        grid.add_column("Absolute Yaw", justify="right", width=20)

        import numpy as np

        effector = excavator.forward_kinematics2()
        for idx, joint in enumerate(excavator.joints):
            grid.add_row(
                (" " * idx) + joint.name,
                format_coord(effector[idx][0]),
                format_coord(effector[idx][1]),
                format_coord(effector[idx][2]),
                format_angle(effector[idx][3]),
                format_angle(effector[idx][4]),
                format_angle(effector[idx][5]),
            )

        grid.add_row()

        effector = excavator.forward_kinematics2(joint_name="attachment_joint")
        grid.add_row(
            "Effector point",
            format_coord(effector[0]),
            format_coord(effector[1]),
            format_coord(effector[2]),
            format_angle(effector[3]),
            format_angle(effector[4]),
            format_angle(effector[5]),
            style="bold bright_yellow",
        )

        return grid


layout = make_layout()
layout["header"].update(Header())
layout["encoder"].update(
    Panel(EncoderTable(), title="[magenta3][ Encoders ]", style="on grey15")
)
layout["engine"].update(EnginePanel())
layout["vms"].update(VMSPanel())
layout["footer"].update(Footer())
layout["origin"].update(
    Panel(
        OriginGrid(),
        title="[magenta3][ Origin ]",
        style="on grey15",
    )
)
layout["effector"].update(
    Panel(KinematicGrid(), title="[magenta3][ Kinematics ]", style="on grey15")
)


with Live(layout, refresh_per_second=20) as live:
    adapter.start()

    try:
        while True:
            if adapter.is_initialized():
                excavator.frame = excavator.frame_joint.clip(
                    adapter.encoder["frame"]["angle"]
                )
                excavator.boom = excavator.boom_joint.clip(
                    adapter.encoder["boom"]["angle"]
                )
                excavator.arm = excavator.arm_joint.clip(
                    adapter.encoder["arm"]["angle"]
                )
                excavator.attachment = excavator.attachment_joint.clip(
                    adapter.encoder["attachment"]["angle"]
                )

            if adapter.status == adapter.ConnectionState.DISCONNECTED:
                adapter.restart()

            live.update(layout)
            time.sleep(0.1)
    except KeyboardInterrupt:
        adapter.stop()
