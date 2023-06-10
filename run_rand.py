#!/usr/bin/env python3

import os
import sys
import random
import traceback
import argparse
import configparser


from pyglonax.executor import ExcavatorExecutor


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--no-supervisor", action="store_false")
    parser.add_argument("-t", "--trace", action="store_true")
    parser.add_argument("-c", "--config", default="config.ini", help="config file")

    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config)

    host = config["glonax"]["host"]
    port = config["glonax"]["port"]

    robot = dict(config["robot"])
    kinematics = dict(config["kinematics"])

    executor = ExcavatorExecutor(
        host=f"{host}:{port}",
        supervisor=args.no_supervisor,
        trace=args.trace,
        **robot,
        **kinematics,
    )
    try:
        executor.start()

        frame_device = executor.excavator.frame_joint.device
        boom_device = executor.excavator.boom_joint.device
        arm_device = executor.excavator.arm_joint.device
        attachment_device = executor.excavator.attachment_joint.device

        idx = 0
        while True:
            random_frame = random.uniform(
                frame_device.lower_bound, frame_device.upper_bound
            )
            random_boom = random.uniform(
                boom_device.lower_bound, boom_device.upper_bound
            )
            random_arm = random.uniform(arm_device.lower_bound, arm_device.upper_bound)
            random_attachment = random.uniform(
                attachment_device.lower_bound, attachment_device.upper_bound
            )

            target = executor.articulation_chain.forward_kinematics(
                joint_parameters=[
                    random_frame,
                    random_boom - 1.047,
                    random_arm,
                    random_attachment - 0.962,
                ]
            )

            executor.solve_target(idx, target)
            idx += 1
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
    finally:
        executor.stop()
