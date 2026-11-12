"""Stand-in for dev_scripts/mirror_leader.py in test mode: no serial ports are opened."""

import time

if __name__ == "__main__":
    print("aligned", flush=True)
    print("mirroring, Ctrl+C to stop", flush=True)
    try:
        while True:
            time.sleep(1.0)
            print("F1=2047  F2=2047  F3=2047  F4=2047  F5=2047  F6=2047", flush=True)
    except KeyboardInterrupt:
        print("interrupted", flush=True)
    finally:
        print("follower keeps torque (holding pose); run with --release to let it go limp", flush=True)
