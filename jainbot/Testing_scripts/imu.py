#!/usr/bin/env python3
import time
from smbus2 import SMBus

# I2C addresses
ADDR_ISM = 0x6B     # ISM330DHCX (accel+gyro)
ADDR_MMC = 0x30     # MMC5983MA  (mag)

bus = SMBus(1)

def twos_comp(val, bits=16):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

# ---------- ISM330DHCX init ----------
# CTRL3_C (0x12): IF_INC=1 (auto-increment), BDU=1
bus.write_byte_data(ADDR_ISM, 0x12, 0x44)
time.sleep(0.01)

# CTRL1_XL (0x10): ODR_XL=104 Hz (0x4), FS_XL=±4g (0x2) -> 0b0101_0000 = 0x50
bus.write_byte_data(ADDR_ISM, 0x10, 0x50)

# CTRL2_G (0x11): ODR_G=104 Hz (0x4), FS_G=2000 dps (0x3) -> 0b0101_1100 = 0x5C
bus.write_byte_data(ADDR_ISM, 0x11, 0x5C)
time.sleep(0.01)

def read_ism_accel():
    # OUTX_L_A .. OUTZ_H_A = 0x28..0x2D
    d = bus.read_i2c_block_data(ADDR_ISM, 0x28, 6)
    ax = twos_comp((d[1] << 8) | d[0]) * 0.122 / 1000.0  # g (±4g)
    ay = twos_comp((d[3] << 8) | d[2]) * 0.122 / 1000.0
    az = twos_comp((d[5] << 8) | d[4]) * 0.122 / 1000.0
    return ax, ay, az

def read_ism_gyro():
    # OUTX_L_G .. OUTZ_H_G = 0x22..0x27
    d = bus.read_i2c_block_data(ADDR_ISM, 0x22, 6)
    gx = twos_comp((d[1] << 8) | d[0]) * 70.0 / 1000.0   # dps (±2000)
    gy = twos_comp((d[3] << 8) | d[2]) * 70.0 / 1000.0
    gz = twos_comp((d[5] << 8) | d[4]) * 70.0 / 1000.0
    return gx, gy, gz

def ism_status():
    # STATUS_REG (0x1E): GDA (bit1), XLDA (bit0)
    return bus.read_byte_data(ADDR_ISM, 0x1E)

# ---------- MMC5983MA init ----------
# Put in a basic measurement mode
bus.write_byte_data(ADDR_MMC, 0x09, 0x08)
time.sleep(0.01)

def read_mmc():
    # trigger single measurement
    bus.write_byte_data(ADDR_MMC, 0x09, 0x01)
    time.sleep(0.01)
    d = bus.read_i2c_block_data(ADDR_MMC, 0x00, 7)
    mx = ((d[1] << 10) | (d[0] << 2) | ((d[6] >> 6) & 0x03))
    my = ((d[3] << 10) | (d[2] << 2) | ((d[6] >> 4) & 0x03))
    mz = ((d[5] << 10) | (d[4] << 2) | ((d[6] >> 2) & 0x03))
    if mx >= 2**19: mx -= 2**20
    if my >= 2**19: my -= 2**20
    if mz >= 2**19: mz -= 2**20
    return mx*0.25, my*0.25, mz*0.25  # µT

print("Reading IMU... (Ctrl+C to stop)")
try:
    while True:
        st = ism_status()
        gx, gy, gz = read_ism_gyro()
        ax, ay, az = read_ism_accel()
        mx, my, mz = read_mmc()
        # Optional: show status bits to confirm new data flags
        gda = (st >> 1) & 1
        xlda = st & 1
        print(f"[GDA:{gda} XLDA:{xlda}] "
              f"Gyro[dps]:({gx:+.1f},{gy:+.1f},{gz:+.1f})  "
              f"Accel[g]:({ax:+.3f},{ay:+.3f},{az:+.3f})  "
              f"Mag[µT]:({mx:+.1f},{my:+.1f},{mz:+.1f})")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    bus.close()
