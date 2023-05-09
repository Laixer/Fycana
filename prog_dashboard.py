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

config = get_config()

excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])
adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])


def format_angle(value=None) -> Text:
    text = Text()

    if value is None:
        text.append("inf", style="bright_black")
        return text

    text.append("{:7.2f}".format(value), style="chartreuse2")
    text.append("rad", style="bright_black")
    text.append(" ")
    text.append("{:7.2f}".format(math.degrees(value)), style="yellow1")
    text.append("°", style="bright_black")

    return text


def format_angle_low(value=None) -> Text:
    text = Text()

    if value is None:
        text.append("inf", style="bright_black")
        return text

    text.append("{:7.2f}".format(value), style="chartreuse4")
    text.append("rad", style="bright_black")
    text.append(" ")
    text.append("{:7.2f}".format(math.degrees(value)), style="yellow3")
    text.append("°", style="bright_black")

    return text


def format_percent(value=None, style="white") -> Text:
    text = Text()

    if value is None:
        return text

    text.append("{:>.1f}".format(value), style=style)
    text.append("%", style="bright_black")

    return text


def print_point(x, y, z) -> Text:
    text = Text()

    text.append(" X:", style="bright_black")
    text.append("{:6.2f}".format(x))
    text.append(" Y:", style="bright_black")
    text.append("{:6.2f}".format(y))
    text.append(" Z:", style="bright_black")
    text.append("{:6.2f}".format(z))

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
        Layout(name="side"), Layout(name="body", ratio=2, minimum_size=50)
    )
    layout["side"].split(Layout(name="telemetric"), Layout(name="box2"))
    layout["telemetric"].split_row(Layout(name="engine"), Layout(name="vms"))
    layout["body"].split(Layout(name="woei"), Layout(name="encoder"))
    layout["woei"].split_row(Layout(name="trans"), Layout(name="or"))

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

        return Panel(grid, title="[bright_cyan][ Engine ]", style="on grey11")


class VMSPanel:
    """Display engine parameters."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(ratio=1)
        grid.add_column(justify="right")
        grid.add_column(justify="right")
        grid.add_column(justify="right")

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

        return Panel(grid, title="[bright_cyan][ VMS ]", style="on grey11")


class EncoderTable:
    """Display header with clock."""

    def __rich__(self):
        table = Table(show_edge=False, expand=True)

        table.add_column("Encoder", no_wrap=True, min_width=10)
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
                    format_angle_low(joint.lower_bound),
                    format_angle(adapter.encoder[encoder_name]["angle"]),
                    format_percent(
                        normal * 100,
                        style="red3" if normal > 1 else "white",
                    ),
                    format_angle_low(joint.upper_bound),
                )

        return table


class KinematicGrid:
    """Display header with clock."""

    def __rich__(self) -> Table:
        grid = Table.grid(expand=True)

        grid.add_column()
        grid.add_column("X", justify="right", width=5)
        grid.add_column("Y", justify="right", width=5)
        grid.add_column("Z", justify="right", width=5)

        for joint in excavator.joints:
            if joint.origin_translation is not None:
                grid.add_row(
                    f"{joint.name}",
                    "{:>.2f}".format(joint.origin_translation[0]),
                    "{:>.2f}".format(joint.origin_translation[1]),
                    "{:>.2f}".format(joint.origin_translation[2]),
                    style="grey62",
                )
            else:
                grid.add_row(f"{joint.name}", "-", "-", "-", style="grey62")

        return grid


class Kinematic2Grid:
    """Display header with clock."""

    def __rich__(self) -> Table:
        grid = Table.grid(expand=True)

        grid.add_column()
        grid.add_column("X", justify="right", width=5)
        grid.add_column("Y", justify="right", width=5)
        grid.add_column("Z", justify="right", width=5)

        for joint in excavator.joints:
            if joint.origin_orientation is not None:
                grid.add_row(
                    f"{joint.name}",
                    "{:>.2f}".format(joint.origin_orientation[0]),
                    "{:>.2f}".format(joint.origin_orientation[1]),
                    "{:>.2f}".format(joint.origin_orientation[2]),
                    style="grey62",
                )
            else:
                grid.add_row(f"{joint.name}", "-", "-", "-", style="grey62")

        return grid


class MotionGrid:
    """Display header with clock."""

    def __rich__(self) -> Table:
        grid = Table.grid(expand=True)

        grid.add_column()
        grid.add_column("X", justify="right", width=5)
        grid.add_column("Y", justify="right", width=5)
        grid.add_column("Z", justify="right", width=5)

        effector = excavator.forward_kinematics(joint_name="frame_joint")
        grid.add_row(
            f"Frame point",
            "{:>.2f}".format(effector[0]),
            "{:>.2f}".format(effector[1]),
            "{:>.2f}".format(effector[2]),
            style="grey62",
        )

        effector = excavator.forward_kinematics(joint_name="boom_joint")
        grid.add_row(
            f"Boom point",
            "{:>.2f}".format(effector[0]),
            "{:>.2f}".format(effector[1]),
            "{:>.2f}".format(effector[2]),
            style="grey62",
        )

        effector = excavator.forward_kinematics(joint_name="arm_joint")
        grid.add_row(
            f"Arm point",
            "{:>.2f}".format(effector[0]),
            "{:>.2f}".format(effector[1]),
            "{:>.2f}".format(effector[2]),
            style="grey62",
        )

        effector = excavator.forward_kinematics()
        grid.add_row(
            "Effector point",
            "{:>.2f}".format(effector[0]),
            "{:>.2f}".format(effector[1]),
            "{:>.2f}".format(effector[2]),
            style="bold bright_yellow",
        )

        return grid


layout = make_layout()
layout["header"].update(Header())
layout["encoder"].update(
    Panel(EncoderTable(), title="[bright_cyan][ Encoders ]", style="on grey19")
)
layout["engine"].update(EnginePanel())
layout["vms"].update(VMSPanel())
layout["footer"].update(Footer())
layout["trans"].update(
    Panel(
        KinematicGrid(), title="[bright_cyan][ Origin Translation ]", style="on grey11"
    )
)
layout["or"].update(
    Panel(
        Kinematic2Grid(), title="[bright_cyan][ Origin Orientation ]", style="on grey11"
    )
)
layout["box2"].update(
    Panel(MotionGrid(), title="[bright_cyan][ Effector AGL ]", style="on grey11")
)


with Live(layout, refresh_per_second=20) as live:
    adapter.start()

    try:
        while True:
            if adapter.is_initialized():
                frame_joint = excavator.frame_joint
                if frame_joint.iswithin_bounds(adapter.encoder["frame"]["angle"]):
                    excavator.frame = adapter.encoder["frame"]["angle"]
                else:
                    if adapter.encoder["frame"]["angle"] > frame_joint.upper_bound:
                        excavator.frame = frame_joint.upper_bound
                    else:
                        excavator.frame = frame_joint.lower_bound

                boom_joint = excavator.boom_joint
                if boom_joint.iswithin_bounds(adapter.encoder["boom"]["angle"]):
                    excavator.boom = adapter.encoder["boom"]["angle"]
                else:
                    if adapter.encoder["boom"]["angle"] > boom_joint.upper_bound:
                        excavator.boom = boom_joint.upper_bound
                    else:
                        excavator.boom = boom_joint.lower_bound

                arm_joint = excavator.arm_joint
                if arm_joint.iswithin_bounds(adapter.encoder["arm"]["angle"]):
                    excavator.arm = adapter.encoder["arm"]["angle"]
                else:
                    if adapter.encoder["arm"]["angle"] > arm_joint.upper_bound:
                        excavator.arm = arm_joint.upper_bound
                    else:
                        excavator.arm = arm_joint.lower_bound

                attachment_joint = excavator.attachment_joint
                if attachment_joint.iswithin_bounds(
                    adapter.encoder["attachment"]["angle"]
                ):
                    excavator.attachment = adapter.encoder["attachment"]["angle"]
                else:
                    if (
                        adapter.encoder["attachment"]["angle"]
                        > attachment_joint.upper_bound
                    ):
                        excavator.attachment = attachment_joint.upper_bound
                    else:
                        excavator.attachment = attachment_joint.lower_bound

            if adapter.status == adapter.ConnectionState.DISCONNECTED:
                adapter.restart()

            live.update(layout)
            time.sleep(0.1)
    except KeyboardInterrupt:
        adapter.stop()
