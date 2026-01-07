#!/usr/bin/env python3
import time, math, select, gpiod
from collections import deque

# ====== PINS (avoid Pololu pins) ======
L_A, L_B = 16, 26   # Left encoder A/B
R_A, R_B = 17, 27   # Right encoder A/B
GPIOCHIP = "gpiochip4"

# ====== ENCODER/WHEEL PARAMS (tune here if needed) ======
MOTOR_PPR_PER_CH = 7      # pulses per motor rev per channel (typical for these 775 encoders)
QUAD_MULT        = 4      # quadrature decoding x4
GEAR_RATIO       = 71.2   # gearbox ratio (title shows ~71:1)
CPR_WHEEL        = MOTOR_PPR_PER_CH * QUAD_MULT * GEAR_RATIO  # ~1994 by default

WHEEL_DIAMETER_M = 0.100  # 100 mm wheel
WHEEL_CIRC_M     = math.pi * WHEEL_DIAMETER_M

PRINT_PERIOD_S   = 0.5
RATE_WINDOW_S    = 0.5    # rolling window for cps estimate

# ====== Quadrature decode table ======
# State is (A<<1)|B in {0..3}; valid transitions give +1/-1
DELTA = {
    (0,1): +1, (1,3): +1, (3,2): +1, (2,0): +1,  # forward
    (1,0): -1, (3,1): -1, (2,3): -1, (0,2): -1,  # reverse
}

class QuadCounter:
    def __init__(self, line_a: gpiod.Line, line_b: gpiod.Line, name="wheel"):
        self.la, self.lb = line_a, line_b
        a, b = self.la.get_value(), self.lb.get_value()
        self.prev = (a << 1) | b
        self.count = 0
        self.name = name
        self.history = deque(maxlen=2000)  # (t, count)

    def on_edge(self):
        a, b = self.la.get_value(), self.lb.get_value()
        curr = (a << 1) | b
        self.count += DELTA.get((self.prev, curr), 0)
        self.prev = curr
        t = time.monotonic()
        self.history.append((t, self.count))

    def cps(self, window=RATE_WINDOW_S):
        if not self.history:
            return 0.0
        now = time.monotonic()
        # find oldest sample within window
        oldest_t, oldest_c = self.history[0]
        for t, c in reversed(self.history):
            if now - t >= window:
                oldest_t, oldest_c = t, c
                break
        dt = max(1e-6, now - oldest_t)
        return (self.count - oldest_c) / dt

def req_edges(line: gpiod.Line):
    line.request(consumer="enc_test", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

def main():
    print(f"Opening {GPIOCHIP} …")
    chip = gpiod.Chip(GPIOCHIP)

    l_a = chip.get_line(L_A); l_b = chip.get_line(L_B)
    r_a = chip.get_line(R_A); r_b = chip.get_line(R_B)
    for ln in (l_a, l_b, r_a, r_b):
        req_edges(ln)

    left  = QuadCounter(l_a, l_b, "left")
    right = QuadCounter(r_a, r_b, "right")

    fds = {
        l_a.event_get_fd(): (l_a, left),
        l_b.event_get_fd(): (l_b, left),
        r_a.event_get_fd(): (r_a, right),
        r_b.event_get_fd(): (r_b, right),
    }

    print("Listening for encoder edges… (Ctrl+C to stop)")
    last = time.monotonic()
    try:
        while True:
            rlist, _, _ = select.select(list(fds.keys()), [], [], 0.1)
            for fd in rlist:
                line, which = fds[fd]
                while line.event_wait(0):
                    _ = line.event_read()
                    which.on_edge()

            now = time.monotonic()
            if now - last >= PRINT_PERIOD_S:
                l_cps = left.cps()
                r_cps = right.cps()

                # Convert to wheel RPM and linear m/s
                l_rps = l_cps / CPR_WHEEL
                r_rps = r_cps / CPR_WHEEL
                l_rpm = l_rps * 60.0
                r_rpm = r_rps * 60.0

                l_ms = l_rps * WHEEL_CIRC_M
                r_ms = r_rps * WHEEL_CIRC_M

                print(f"L cnt {left.count:7d}  cps {l_cps:8.1f}  rpm {l_rpm:7.2f}  m/s {l_ms:6.3f}   "
                      f"R cnt {right.count:7d}  cps {r_cps:8.1f}  rpm {r_rpm:7.2f}  m/s {r_ms:6.3f}")
                last = now
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        for ln in (l_a, l_b, r_a, r_b):
            try: ln.release()
            except Exception: pass
        chip.close()

if __name__ == "__main__":
    main()
