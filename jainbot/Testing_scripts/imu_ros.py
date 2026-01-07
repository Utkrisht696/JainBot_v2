#!/usr/bin/env python3
import os, time, math, yaml
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from builtin_interfaces.msg import Time as RosTime
from smbus2 import SMBus

# I2C addresses (defaults for your board)
ADDR_ISM = 0x6B   # ISM330DHCX accel+gyro
ADDR_MMC = 0x30   # MMC5983MA mag

# ---------------- Utility math ----------------
def _twos16(x): return x - 65536 if x & 0x8000 else x

def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    q = np.array([w, x, y, z], float)
    return q / (np.linalg.norm(q) + 1e-12)

def initial_quat_from_accel_mag(accel_g, mag_uT):
    # Normalize
    a = accel_g / (np.linalg.norm(accel_g) + 1e-9)
    m = mag_uT  / (np.linalg.norm(mag_uT) + 1e-9)
    # Roll & pitch from gravity (Z-up)
    roll  = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.sqrt(a[1]**2 + a[2]**2))
    # Tilt-compensated yaw from mag
    sr, cr = math.sin(roll),  math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    mx2 = m[0]*cp + m[2]*sp
    my2 = m[0]*sr*sp + m[1]*cr - m[2]*sr*cp
    yaw = math.atan2(-my2, mx2)  # flip sign if your frame needs it
    return euler_to_quat(roll, pitch, yaw)  # (w,x,y,z)

# ---------------- ISM330DHCX ----------------
class ISM330DHCX:
    def __init__(self, bus: SMBus, addr=ADDR_ISM):
        self.bus = bus
        self.addr = addr

    def _w(self, reg, val): self.bus.write_byte_data(self.addr, reg, val)
    def _rblk(self, start, n): return self.bus.read_i2c_block_data(self.addr, start, n)

    def init(self):
        # CTRL3_C: IF_INC=1 (auto-increment), BDU=1
        self._w(0x12, 0x44); time.sleep(0.01)
        # Accel: 104 Hz, ±2 g (0.061 mg/LSB)
        self._w(0x10, 0x50)
        # Gyro : 104 Hz, ±2000 dps (0.07 dps/LSB)
        self._w(0x11, 0x5C); time.sleep(0.01)

    def read_accel_g(self):
        d = self._rblk(0x28, 6)  # OUTX_L_A..OUTZ_H_A
        k = 0.061/1000.0
        ax = _twos16((d[1]<<8)|d[0]) * k
        ay = _twos16((d[3]<<8)|d[2]) * k
        az = _twos16((d[5]<<8)|d[4]) * k
        return np.array([ax, ay, az], float)

    def read_gyro_dps(self):
        d = self._rblk(0x22, 6)  # OUTX_L_G..OUTZ_H_G
        k = 70.0/1000.0
        gx = _twos16((d[1]<<8)|d[0]) * k
        gy = _twos16((d[3]<<8)|d[2]) * k
        gz = _twos16((d[5]<<8)|d[4]) * k
        return np.array([gx, gy, gz], float)

# ---------------- MMC5983MA ----------------
class MMC5983MA:
    def __init__(self, bus: SMBus, addr=ADDR_MMC):
        self.bus = bus
        self.addr = addr

    def _w(self, reg, val): self.bus.write_byte_data(self.addr, reg, val)
    def _rblk(self, start, n): return self.bus.read_i2c_block_data(self.addr, start, n)

    def init(self):
        # Basic config; we'll trigger single conversions per read
        self._w(0x09, 0x08); time.sleep(0.01)

    def read_uT(self):
        # Trigger single magnetic measurement
        self._w(0x09, 0x01); time.sleep(0.003)
        d = self._rblk(0x00, 7)   # Xout0, Xout1, Yout0, Yout1, Zout0, Zout1, XYZout2
        x = (d[0] << 10) | (d[1] << 2) | ((d[6] >> 6) & 0x03)
        y = (d[2] << 10) | (d[3] << 2) | ((d[6] >> 4) & 0x03)
        z = (d[4] << 10) | (d[5] << 2) | ((d[6] >> 2) & 0x03)
        MID = 131072  # 2^17
        xs, ys, zs = x - MID, y - MID, z - MID
        uT_per_lsb = 100.0/16384.0   # ≈0.0061035 µT/LSB
        mx = xs*uT_per_lsb
        my = ys*uT_per_lsb
        mz = zs*uT_per_lsb
        # Align mag Z with IMU (datasheet note: Z is opposite) → flip Z
        return np.array([mx, my, -mz], float)

# ---------------- Full Madgwick (with magnetometer) ----------------
class Madgwick:
    def __init__(self, beta=0.1, q=None):
        self.beta = beta
        self.q = np.array([1., 0., 0., 0.]) if q is None else q.astype(float)

    def update(self, gyro_rads, accel_ms2, mag_uT, dt):
        q1, q2, q3, q4 = self.q  # (w, x, y, z)
        gx, gy, gz = gyro_rads
        ax, ay, az = accel_ms2
        mx, my, mz = mag_uT

        # Normalize accelerometer
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm > 0.0:
            ax, ay, az = ax/norm, ay/norm, az/norm
        else:
            ax, ay, az = 0.0, 0.0, 0.0  # fall back if invalid

        # Normalize magnetometer
        norm = math.sqrt(mx*mx + my*my + mz*mz)
        if norm > 0.0:
            mx, my, mz = mx/norm, my/norm, mz/norm
        else:
            mx, my, mz = 0.0, 0.0, 0.0

        # Auxiliary variables
        _2q1 = 2.0*q1; _2q2 = 2.0*q2; _2q3 = 2.0*q3; _2q4 = 2.0*q4
        _2q1q3 = 2.0*q1*q3; _2q3q4 = 2.0*q3*q4
        q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3; q4q4 = q4*q4

        # Reference direction of Earth's magnetic field
        hx = 2.0*mx*(0.5 - q3q3 - q4q4) + 2.0*my*(q2*q3 - q1*q4) + 2.0*mz*(q2*q4 + q1*q3)
        hy = 2.0*mx*(q2*q3 + q1*q4) + 2.0*my*(0.5 - q2q2 - q4q4) + 2.0*mz*(q3*q4 - q1*q2)
        _2bx = math.sqrt(hx*hx + hy*hy)
        _2bz = 2.0*mx*(q2*q4 - q1*q3) + 2.0*my*(q1*q2 + q3*q4) + 2.0*mz*(0.5 - q2q2 - q3q3)

        # Gradient descent step (s1..s4)
        s1 = (-_2q3*(2.0*(q2*q4 - q1*q3) - ax) + _2q2*(2.0*(q1*q2 + q3*q4) - ay)
              - _2bz*q3*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (-_2bx*q4 + _2bz*q2)*(2.0*(q2*q3 - q1*q4) - my)
              + _2bx*q3*(2.0*(q1*q3 + q2*q4) - mz))
        s2 = (_2q4*(2.0*(q2*q4 - q1*q3) - ax) + _2q1*(2.0*(q1*q2 + q3*q4) - ay)
              - 4.0*q2*(1 - 2.0*(q2q2 + q3q3) - az)
              + _2bz*q4*(_2bx*(0.5 - q3q3 - q4q4) + _2bz*(q2*q4 - q1*q3) - mx)
              + (_2bx*q3 + _2bz*q1)*(2.0*(q2*q3 - q1*q4) - my)
              + (_2bx*q4 - 2.0*_2bz*q2)*(2.0*(q1*q3 + q2*q4) - mz))
        s3 = (-_2q1*(2.0*(q2*q4 - q1*q3) - ax) + _2q4*(2.0*(q1*q2 + q3*q4) - ay)
              - 4.0*q3*(1 - 2.0*(q2q2 + q3q3) - az)
              + (-2.0*_2bx*q3 + _2bz*q1)*(2.0*(q2*q3 - q1*q4) - my)
              + (_2bx*q2 + _2bz*q4)*(2.0*(q1*q3 + q2*q4) - mz))
        s4 = (_2q2*(2.0*(q2*q4 - q1*q3) - ax) + _2q3*(2.0*(q1*q2 + q3*q4) - ay)
              + (-2.0*_2bx*q4 + _2bz*q2)*(2.0*(q2*q3 - q1*q4) - my)
              + (_2bx*q1 + _2bz*q3)*(2.0*(q1*q3 + q2*q4) - mz))

        norm_s = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm_s > 0.0:
            s1, s2, s3, s4 = s1/norm_s, s2/norm_s, s3/norm_s, s4/norm_s

        # Rate of change of quaternion
        qDot1 = 0.5 * (-q2*gx - q3*gy - q4*gz) - self.beta*s1
        qDot2 = 0.5 * ( q1*gx + q3*gz - q4*gy) - self.beta*s2
        qDot3 = 0.5 * ( q1*gy - q2*gz + q4*gx) - self.beta*s3
        qDot4 = 0.5 * ( q1*gz + q2*gy - q3*gx) - self.beta*s4

        # Integrate & normalize
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        q = np.array([q1, q2, q3, q4])
        self.q = q / (np.linalg.norm(q) + 1e-12)
        return self.q

    @staticmethod
    def to_quat_msg(q):
        # ROS wants x,y,z,w
        return (float(q[1]), float(q[2]), float(q[3]), float(q[0]))

# ---------------- ROS 2 Node ----------------
class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('calib_yaml', 'imu_calib.yaml')
        self.declare_parameter('use_madgwick', True)
        self.declare_parameter('madgwick_beta', 0.1)

        bus_id       = int(self.get_parameter('i2c_bus').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        rate_hz      = float(self.get_parameter('rate_hz').value)
        calib_path   = str(self.get_parameter('calib_yaml').value)
        self.use_madgwick = bool(self.get_parameter('use_madgwick').value)
        beta         = float(self.get_parameter('madgwick_beta').value)

        # Load calibration
        if not os.path.exists(calib_path):
            raise FileNotFoundError(f"Calibration file not found: {calib_path}")
        with open(calib_path) as f:
            calib = yaml.safe_load(f)
        self.a_off = np.array(calib['accel_offset_g'], float)
        self.a_scl = np.array(calib['accel_scale'], float)
        self.g_bias = np.array(calib['gyro_bias_dps'], float)
        self.m_bias = np.array(calib['mag_bias_uT'], float)
        self.A_mag  = np.array(calib['mag_A_3x3'], float)

        # I2C + sensors
        self.bus = SMBus(bus_id)
        self.ism = ISM330DHCX(self.bus); self.ism.init()
        self.mmc = MMC5983MA(self.bus);  self.mmc.init()

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/mag/data', 10)

        # Seed orientation from a calibrated static read (accel+mag)
        a0_g  = self.ism.read_accel_g()
        m0_uT = self.mmc.read_uT()
        a0_cal = (a0_g - self.a_off) * self.a_scl
        m0_cal = self.A_mag @ (m0_uT - self.m_bias)

        # Madgwick (full, with magnetometer)
        self.madgwick = Madgwick(beta=beta) if self.use_madgwick else None
        if self.madgwick is not None:
            q0 = initial_quat_from_accel_mag(a0_cal, m0_cal)
            self.madgwick.q = q0  # (w,x,y,z)

        self.last_t = time.monotonic()
        self.dt_cap = 0.1
        self.timer = self.create_timer(max(0.001, 1.0/rate_hz), self._on_timer)
        self.get_logger().info(f"IMU node started | frame_id={self.frame_id} | rate={rate_hz} Hz | "
                               f"Madgwick={'ON' if self.use_madgwick else 'OFF'} | calib={calib_path}")

    def destroy_node(self):
        try: self.bus.close()
        except Exception: pass
        return super().destroy_node()

    @staticmethod
    def _stamp_now():
        t = time.time()
        sec = int(t); nsec = int((t-sec)*1e9)
        return RosTime(sec=sec, nanosec=nsec)

    def _on_timer(self):
        try:
            a_g   = self.ism.read_accel_g()
            g_dps = self.ism.read_gyro_dps()
            m_uT  = self.mmc.read_uT()
        except Exception as e:
            self.get_logger().warn(f"I2C read failed: {e}")
            return

        # Apply calibration
        a_cal_g = (a_g - self.a_off) * self.a_scl
        g_cal_dps = (g_dps - self.g_bias)
        g_cal_rads = np.deg2rad(g_cal_dps)
        m_cal_uT = self.A_mag @ (m_uT - self.m_bias)

        stamp = self._stamp_now()

        # Publish MagneticField (Tesla)
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = float(m_cal_uT[0] * 1e-6)
        mag_msg.magnetic_field.y = float(m_cal_uT[1] * 1e-6)
        mag_msg.magnetic_field.z = float(m_cal_uT[2] * 1e-6)
        mag_msg.magnetic_field_covariance[0] = -1.0
        self.pub_mag.publish(mag_msg)

        # Build Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        # Orientation (Madgwick with magnetometer)
        if self.madgwick is not None:
            now = time.monotonic()
            dt = max(1e-3, min(self.dt_cap, now - self.last_t))
            self.last_t = now
            q = self.madgwick.update(g_cal_rads, a_cal_g*9.80665, m_cal_uT, dt)  # rad/s, m/s^2, µT
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = Madgwick.to_quat_msg(q)
            imu_msg.orientation_covariance = [0.05,0,0, 0,0.05,0, 0,0,0.05]
        else:
            imu_msg.orientation_covariance = [-1.0,0,0, 0,-1.0,0, 0,0,-1.0]

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(g_cal_rads[0])
        imu_msg.angular_velocity.y = float(g_cal_rads[1])
        imu_msg.angular_velocity.z = float(g_cal_rads[2])
        imu_msg.angular_velocity_covariance = [0.02,0,0, 0,0.02,0, 0,0,0.02]

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(a_cal_g[0] * 9.80665)
        imu_msg.linear_acceleration.y = float(a_cal_g[1] * 9.80665)
        imu_msg.linear_acceleration.z = float(a_cal_g[2] * 9.80665)
        imu_msg.linear_acceleration_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]

        self.pub_imu.publish(imu_msg)

# ---------------- Main ----------------
def main():
    rclpy.init()
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
