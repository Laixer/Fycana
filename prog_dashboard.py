import os
import time
import math

from pyglonax.excavator import Excavator, ExcavatorAdapter

from rich.table import Table
from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.console import Group
from rich.align import Align
from rich.text import Text


def from_local_path(file, sub_dir=None):
    script_dir = os.path.dirname(__file__)
    if sub_dir is None:
        rel_path = file
    else:
        rel_path = os.path.join(sub_dir, file)
    abs_file_path = os.path.join(script_dir, rel_path)
    return abs_file_path


excavator = Excavator.from_urdf(from_local_path("volvo_ec240cl.urdf", "urdf"))
adapter = ExcavatorAdapter()


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


def format_percent(value=None) -> Text:
    text = Text()

    if value is None:
        return text

    text.append("{:>.1f}".format(value))
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
            "[green3]Connected",
            f"{excavator.name} [b]{excavator.model}[/b]",
            adapter.host,
        )

        return Panel(grid, style="on grey11")


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

        grid.add_row("CPU", format_percent(12))
        grid.add_row("Memory", format_percent(45))

        return Panel(grid, title="[bright_cyan][ VMS ]", style="on grey11")


class EncoderTable:
    """Display header with clock."""

    def __rich__(self):
        table = Table(show_edge=False, expand=True)

        table.add_column("Encoder", no_wrap=True, min_width=10)
        table.add_column("Position", justify="right", style="bright_black")
        table.add_column("Bounds Lower", justify="right")
        table.add_column("Angle", justify="right")
        table.add_column("Percentage", justify="right")
        table.add_column("Bounds Upper", justify="right")

        if "frame" in adapter.encoder:
            frame_joint = excavator.frame_joint

            table.add_row(
                "Frame",
                "{:6.1f}".format(adapter.encoder["frame"]["position"]),
                format_angle(frame_joint.lower_bound),
                format_angle(adapter.encoder["frame"]["angle"]),
                format_percent(
                    frame_joint.normalize(adapter.encoder["frame"]["angle"]) * 100
                ),
                format_angle(frame_joint.upper_bound),
            )
        else:
            table.add_row("Frame")

        if "boom" in adapter.encoder:
            boom_joint = excavator.boom_joint

            table.add_row(
                "Boom",
                "{:6.1f}".format(adapter.encoder["boom"]["position"]),
                format_angle(boom_joint.lower_bound),
                format_angle(adapter.encoder["boom"]["angle"]),
                format_percent(
                    boom_joint.normalize(adapter.encoder["boom"]["angle"]) * 100
                ),
                format_angle(boom_joint.upper_bound),
            )
        else:
            table.add_row("Boom")

        if "arm" in adapter.encoder:
            arm_joint = excavator.arm_joint

            table.add_row(
                "Arm",
                "{:6.1f}".format(adapter.encoder["arm"]["position"]),
                format_angle(arm_joint.lower_bound),
                format_angle(adapter.encoder["arm"]["angle"]),
                format_percent(
                    arm_joint.normalize(adapter.encoder["arm"]["angle"]) * 100
                ),
                format_angle(arm_joint.upper_bound),
            )
        else:
            table.add_row("Arm")

        if "attachment" in adapter.encoder:
            attachment_joint = excavator.attachment_joint

            table.add_row(
                "Attachment",
                "{:6.1f}".format(adapter.encoder["attachment"]["position"]),
                format_angle(),
                format_angle(adapter.encoder["attachment"]["angle"]),
                format_percent(
                    attachment_joint.normalize(adapter.encoder["attachment"]["angle"])
                    * 100
                ),
                format_angle(),
            )
        else:
            table.add_row("Attachment")

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
layout["footer"].update(Panel("Laixer Equipment B.V.", style="grey62 on grey3"))
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
                excavator.frame = adapter.encoder["frame"]["angle"]
                excavator.boom = adapter.encoder["boom"]["angle"]
                excavator.arm = adapter.encoder["arm"]["angle"]
                excavator.attachment = adapter.encoder["attachment"]["angle"]

            live.update(layout)
            time.sleep(0.1)
    except KeyboardInterrupt:
        adapter.stop()
