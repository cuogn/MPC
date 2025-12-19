import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--demo",
        type=str,
        default=None,
        choices=["open_loop", "current_loop", "speed_pid", "speed_mpc", "compare_pid_mpc"],
    )
    parser.add_argument("--gui", action="store_true", help="Launch realtime GUI")
    args = parser.parse_args()

    if args.gui:
        from app.gui.app import run_gui
        run_gui()
        return

    if args.demo is None:
        print("Nothing to do. Use --gui or --demo <name>.")
        return

    from app.sim.simulator import (
        run_open_loop_demo,
        run_current_loop_demo,
        run_speed_pid_demo,
        run_speed_mpc_demo,
        run_compare_pid_mpc_demo,
    )

    if args.demo == "open_loop":
        run_open_loop_demo()
    elif args.demo == "current_loop":
        run_current_loop_demo()
    elif args.demo == "speed_pid":
        run_speed_pid_demo()
    elif args.demo == "speed_mpc":
        run_speed_mpc_demo()
    else:
        run_compare_pid_mpc_demo()

if __name__ == "__main__":
    main()
