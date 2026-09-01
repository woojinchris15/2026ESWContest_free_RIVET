#!/usr/bin/env python3

import time
import subprocess

CHIP = "gpiochip4"
LINE = "6"

LOSS_CONFIRM_SEC = 5.0
POLL_INTERVAL = 0.5


def read_power():
    result = subprocess.run(
        ["/usr/bin/gpioget", CHIP, LINE],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        print(f"[X1208] GPIO read error: {result.stderr.strip()}")
        return None

    value = result.stdout.strip()

    if value == "1":
        return True

    if value == "0":
        return False

    print(f"[X1208] Unexpected GPIO value: {value}")
    return None


def poweroff():
    print("[X1208] External power lost for 5 seconds.")
    print("[X1208] Shutting down CUBIC...")

    subprocess.run(
        ["/usr/bin/systemctl", "poweroff"],
        check=False
    )


def main():
    print("[X1208] Power monitor started.")
    print("[X1208] GPIO6: HIGH=AC OK, LOW=AC LOST")

    loss_start = None

    while True:

        power_ok = read_power()

        # GPIO 자체 읽기 실패
        if power_ok is None:
            time.sleep(POLL_INTERVAL)
            continue

        # 외부 전원 정상
        if power_ok:
            if loss_start is not None:
                print("[X1208] External power restored. Shutdown cancelled.")

            loss_start = None

        # 외부 전원 상실
        else:
            if loss_start is None:
                loss_start = time.monotonic()
                print("[X1208] External power loss detected.")

            elapsed = time.monotonic() - loss_start

            if elapsed >= LOSS_CONFIRM_SEC:
                poweroff()
                return

        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    main()
