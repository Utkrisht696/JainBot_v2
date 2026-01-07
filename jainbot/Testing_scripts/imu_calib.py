#!/usr/bin/env python3
import time, sys, math, yaml
import numpy as np
from smbus2 import SMBus

# I2C addresses for your board
ADDR_ISM = 0x6B   # ISM330DHCX (accel+gyro)
ADDR_MMC = 0x30   # MMC5983MA  (mag)

bus = SMBus(1)

# ---------------- ISM330DHCX helpers ----------------
def twos16(x): 
    return x-65536 if x & 0x8000 else x

def ism_write(reg, val): 
    bus.write_byte_data(ADDR_ISM, reg, val)

def ism_read_block(start, n): 
    return bus.read_i2c_block_data(ADDR_ISM, start, n)

def ism_init():
    # CTRL3_C: IF_INC=1 (auto-increment), BDU=1
    # 0x44 = 0b0100_0100
    ism_write(0x12, 0x44); time.sleep(0.01)

    # ACCEL: CTRL1_XL
    # 0x50: ODR=104 Hz, FS=±2 g (per ISM330DHCX FS bits), BW default
    # Sensitivity at ±2 g = 0.061 mg/LSB
    ism_write(0x10, 0x50)

    # GYRO: CTRL2_G
    # 0x5C: ODR=104 Hz, FS=±2000 dps  (70 mdps/LSB)
    ism_write(0x11, 0x5C); time.sleep(0.01)

def read_accel_g():
    # OUTX_L_A..OUTZ_H_A = 0x28..0x2D
    d = ism_read_block(0x28, 6)
    # ±2 g sensitivity: 0.061 mg/LSB = 0.000061 g/LSB
    k = 0.061 / 1000.0
    ax = twos16((d[1]<<8) | d[0]) * k
    ay = twos16((d[3]<<8) | d[2]) * k
    az = twos16((d[5]<<8) | d[4]) * k
    return np.array([ax, ay, az], dtype=float)

def read_gyro_dps():
    # OUTX_L_G..OUTZ_H_G = 0x22..0x27
    d = ism_read_block(0x22, 6)
    # ±2000 dps sensitivity: 70 mdps/LSB = 0.07 dps/LSB
    k = 70.0 / 1000.0
    gx = twos16((d[1]<<8) | d[0]) * k
    gy = twos16((d[3]<<8) | d[2]) * k
    gz = twos16((d[5]<<8) | d[4]) * k
    return np.array([gx, gy, gz], dtype=float)

# ---------------- MMC5983MA helpers ----------------
def mmc_write(reg, val): 
    bus.write_byte_data(ADDR_MMC, reg, val)

def mmc_read_block(start, n): 
    return bus.read_i2c_block_data(ADDR_MMC, start, n)

def mmc_init():
    # Basic conf; we’ll trigger single conversions each read
    mmc_write(0x09, 0x08)   # TM_M enable bit (per datasheet usage)
    time.sleep(0.01)

def read_mag_uT():
    """
    Read MMC5983MA 18-bit X/Y/Z, center around mid-scale (131072),
    convert to microtesla, and FLIP Z to align with ISM330DHCX frame.
    """
    # Trigger single magnetic measurement
    mmc_write(0x09, 0x01)
    time.sleep(0.003)  # small wait (optionally poll status reg 0x08)

    d = mmc_read_block(0x00, 7)
    # Register order: Xout0, Xout1, Yout0, Yout1, Zout0, Zout1, XYZout2
    # 18-bit packing: [MSB(8)=out0][next(8)=out1][LSB(2)=XYZout2 bits]
    x = (d[0] << 10) | (d[1] << 2) | ((d[6] >> 6) & 0x03)
    y = (d[2] << 10) | (d[3] << 2) | ((d[6] >> 4) & 0x03)
    z = (d[4] << 10) | (d[5] << 2) | ((d[6] >> 2) & 0x03)

    # Convert from unsigned to signed around mid-scale (18-bit)
    MID = 131072  # 2^17
    xs = x - MID
    ys = y - MID
    zs = z - MID

    # 18-bit sensitivity: 16384 counts/G  => 100 µT / 16384 ≈ 0.0061035 µT/LSB
    UTPERLSB = 100.0 / 16384.0
    mx = xs * UTPERLSB
    my = ys * UTPERLSB
    mz = zs * UTPERLSB

    # Align mag with IMU: magnetometer Z is opposite → flip Z
    return np.array([mx, my, -mz], dtype=float)

# --------------- guided sampling utils ---------------
def collect_samples(fn, N=400, note="collecting...", dt=0.005):
    buf = []
    for i in range(N):
        buf.append(fn())
        time.sleep(dt)
        if (i+1) % max(1, N//10) == 0:
            print(f"{note} {i+1}/{N}")
    return np.vstack(buf)

def wait_enter(msg):
    print("\n" + msg)
    input("Press ENTER when ready...")

# --------------- calibration routines ----------------
def calibrate_gyro():
    print("\n=== Gyro bias calibration ===")
    wait_enter("Place the ROBOT completely still on a solid surface. Do NOT move it for ~6 seconds.")
    samples = collect_samples(read_gyro_dps, N=600, note="Gyro")
    bias = samples.mean(axis=0)  # dps
    print(f"Gyro bias [dps]: {bias}")
    return bias.tolist()

POSES = [
    ("Front/Nose UP",  "Tilt robot so the **front (nose) points straight up**"),
    ("Back/Tail UP",   "Tilt robot so the **back (tail) points straight up**"),
    ("Left side UP",   "Roll robot so the **left side faces up**"),
    ("Right side UP",  "Roll robot so the **right side faces up**"),
    ("Top UP",         "Keep robot in normal pose so the **top faces up**"),
    ("Bottom UP",      "Flip robot upside down so the **bottom faces up**"),
]

def dominant_axis(v):
    i = int(np.argmax(np.abs(v)))
    s = 1.0 if v[i] >= 0 else -1.0
    return i, s  # axis index 0/1/2 and sign

def calibrate_accel_robot_faces():
    """
    Prompts for six ROBOT poses and auto-detects which sensor axis aligns with +/−g
    for each pose. Computes per-axis offset and scale.
    """
    print("\n=== Accel calibration with ROBOT poses (auto-maps axes) ===")
    readings = {}
    doms = {}  # pose -> (axis, sign)
    for tag, instr in POSES:
        wait_enter(f"{instr}.\nKeep it still during capture.")
        S = collect_samples(read_accel_g, N=300, note=f"Accel {tag}", dt=0.006)
        m = S.mean(axis=0)
        readings[tag] = m
        i, s = dominant_axis(m)
        doms[tag] = (i, s)
        print(f"{tag}: mean={m.round(4)}  → dominant axis {i} with sign {int(s)}")

    # We expect each sensor axis to appear once with +1g and once with -1g across the 6 poses
    plus = [None, None, None]
    minus = [None, None, None]
    for tag, (i, s) in doms.items():
        if s > 0 and plus[i] is None: plus[i] = readings[tag][i]
        elif s < 0 and minus[i] is None: minus[i] = readings[tag][i]

    # Sanity fallback if a pose wasn't held cleanly
    for i in range(3):
        if plus[i] is None or minus[i] is None:
            print(f"\n[WARN] Axis {i} did not get both +g and -g poses clearly.")
            vals = [readings[t][i] for t in readings]
            plus[i]  = max(vals) if plus[i] is None else plus[i]
            minus[i] = min(vals) if minus[i] is None else minus[i]

    offsets = np.zeros(3)
    scales  = np.ones(3)
    for i in range(3):
        p, n = plus[i], minus[i]
        offsets[i] = (p + n) / 2.0
        denom = abs(p - n)
        scales[i]  = 2.0 / denom if denom > 1e-6 else 1.0

    print(f"\nAccel offset [g]: {offsets}")
    print(f"Accel scale  [-]: {scales}")
    return offsets.tolist(), scales.tolist(), doms

def calibrate_mag_ellipsoid(duration_s=35):
    print("\n=== Magnetometer calibration (hard/soft iron) ===")
    print("Rotate the robot slowly through ALL orientations for about "
          f"{duration_s} seconds (draw a big 3D figure-8 in the air).")
    wait_enter("Keep moving throughout the collection window.")
    t0 = time.time()
    data = []
    while time.time() - t0 < duration_s:
        data.append(read_mag_uT())
        time.sleep(0.02)
        if int(time.time() - t0) % 5 == 0:
            sys.stdout.write("\rSeconds captured: %d" % int(time.time() - t0))
            sys.stdout.flush()
    print("\nProcessing magnetometer samples...")

    M = np.vstack(data)   # N x 3
    bias = M.mean(axis=0) # hard-iron offset (µT)
    D = M - bias
    # Soft-iron correction via eigendecomposition of covariance (sphering)
    C = np.cov(D.T)                  # 3x3
    eigvals, eigvecs = np.linalg.eigh(C)
    s = (np.sqrt(eigvals.max()) / np.sqrt(eigvals)).astype(float)
    S = np.diag(s)
    A = eigvecs @ S @ eigvecs.T      # soft-iron correction matrix

    # Quality check: normalized radii
    Dcal = (A @ D.T).T
    radii = np.linalg.norm(Dcal, axis=1)
    print(f"Mag radii after cal → mean={radii.mean():.2f} µT, std={radii.std():.2f} µT")
    return bias.tolist(), A.tolist()

# ------------------------ main ------------------------
def main():
    print("Initializing sensors...")
    ism_init(); mmc_init()

    # sanity read
    try:
        _ = read_accel_g(); _ = read_gyro_dps(); _ = read_mag_uT()
    except Exception as e:
        print("I2C read failed. Check connections/addresses:", e); sys.exit(1)

    gyro_bias = calibrate_gyro()
    accel_off, accel_scale, pose_map = calibrate_accel_robot_faces()
    mag_bias, mag_A = calibrate_mag_ellipsoid(duration_s=35)

    calib = {
        "gyro_bias_dps": gyro_bias,
        "accel_offset_g": accel_off,
        "accel_scale": accel_scale,          # per-axis, element-wise
        "mag_bias_uT": mag_bias,
        "mag_A_3x3": mag_A,                  # 3x3 list (use as matrix)
        "pose_map_info": {k: [int(v[0]), int(v[1])] for k, v in pose_map.items()},
        "notes": (
            "Apply as:\n"
            "  accel_cal_g = (accel_raw_g - accel_offset_g) * accel_scale (element-wise)\n"
            "  gyro_cal_dps = gyro_raw_dps - gyro_bias_dps\n"
            "  mag_cal_uT = A @ (mag_raw_uT - mag_bias_uT)\n"
            "Mag Z was flipped in the reader to align frames."
        ),
    }

    with open("imu_calib.yaml", "w") as f:
        yaml.safe_dump(calib, f, sort_keys=False)
    print("\n✅ Saved calibration → imu_calib.yaml")
    print("Next: load this file in your reader/ROS node to correct raw data before fusion.")

if __name__ == "__main__":
    try:
        main()
    finally:
        bus.close()
