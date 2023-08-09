#!/usr/bin/env python3

import json
import traceback
import numpy as np
import argparse
import configparser


from pyglonax.executor import ExcavatorExecutor
from pyglonax.util import format_euler_tuple


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("program", help="program file")
    parser.add_argument("-n", "--no-supervisor", action="store_false")
    parser.add_argument("-t", "--trace", action="store_true")
    parser.add_argument("-c", "--config", default="config.ini", help="config file")

    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config)

    host = config["glonax"]["host"]
    port = config["glonax"]["port"]

    program_file = open(args.program)
    json_file = json.load(program_file)
    program = np.array(json_file)
    program_file.close()

    robot = dict(config["robot"])
    kinematics = dict(config["kinematics"])

    executor = ExcavatorExecutor(
        host=host,
        port=port,
        supervisor=args.no_supervisor,
        trace=args.trace,
        **robot,
        **kinematics,
    )
    try:
        executor.start()

        print()
        print("Program:")
        for idx, target in enumerate(program):
            print(f"{idx}", format_euler_tuple(target))

        for idx, target in enumerate(program):
            executor.solve_target(idx, target)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
    finally:
        executor.stop()
