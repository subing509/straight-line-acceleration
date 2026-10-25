#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Function: ROS parsing INS570D.
    Version: V1.0.5 - 简化版，直接解析
"""

import sys
import os
import rospy
import serial
import struct
import math
from timeit import default_timer
from sensor_msgs.msg import Imu, NavSatFix

print("=== daoyuan.py 启动 ===")
print("Python 版本: {}".format(sys.version))

def print_hex(data, length=20):
    """打印十六进制"""
    hex_str = ' '.join(['%02X' % ord(c) if isinstance(c, str) else '%02X' % c for c in data[:length]])
    return hex_str

class FrameLog(object):
    """Write every original record, but bound console and flush frequency."""

    def __init__(self, stream, log_info, log_interval=1.0,
                 flush_interval=1.0, clock=None):
        self.stream = stream
        self.log_info = log_info
        self.log_interval = float(log_interval)
        self.flush_interval = float(flush_interval)
        for value in (self.log_interval, self.flush_interval):
            if math.isnan(value) or math.isinf(value) or value <= 0:
                raise ValueError('Logging intervals must be finite and positive')
        self.clock = clock or default_timer
        self.last_tick = self.clock()
        self.last_flush = self.last_tick
        self.last_log = None
        self.pending_records = 0
        self.summary_records = 0
        self.latest = None
        self.closed = False

    def record(self, packet_count, latitude, longitude, heading, altitude):
        if self.closed:
            raise ValueError('Frame log is closed')
        # Preserve the original per-frame file format and all parsed records.
        self.stream.write('%.7f\t%.7f\t%.2f\n' % (latitude, longitude, heading))
        self.pending_records += 1
        self.summary_records += 1
        self.latest = (packet_count, heading, latitude, longitude, altitude)
        self.tick()

    def tick(self):
        if self.closed:
            return
        now = self.clock()
        rewound = now < self.last_tick
        self.last_tick = now
        if rewound:
            # Python 2's default timer may use wall time. Flush outstanding
            # records once and restart both intervals after a clock correction.
            if self.pending_records:
                self.stream.flush()
                self.pending_records = 0
            self.last_flush = now
            self.last_log = None
        if self.pending_records and now-self.last_flush >= self.flush_interval:
            self.stream.flush()
            self.pending_records = 0
            self.last_flush = now
        if (self.summary_records and
                (self.last_log is None or now-self.last_log >= self.log_interval)):
            packet_count, heading, latitude, longitude, altitude = self.latest
            self.log_info('IMU/GPS packets=%d (+%d), heading=%.2f deg, '
                          'lat=%.7f, lon=%.7f, altitude=%.2f',
                          packet_count, self.summary_records, heading,
                          latitude, longitude, altitude)
            self.summary_records = 0
            self.last_log = now

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.stream.flush()
        finally:
            self.stream.close()


def main():
    rospy.init_node("serial_port", anonymous=True)

    GPS_pub = rospy.Publisher("GPS_data", NavSatFix, queue_size=10)
    imu_pub = rospy.Publisher("imu_data", Imu, queue_size=50)

    imu_msg = Imu()
    gps_msg = NavSatFix()
    
    gps_msg.header.frame_id = "gps"
    imu_msg.header.frame_id = "imu"
    gps_msg.status.status = 0
    gps_msg.status.service = 1

    # 打开串口
    try:
        ser = serial.Serial(
            port='/dev/imu',
            baudrate=230400,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,
	    rtscts=False
        )
    except Exception as e:
        rospy.logerr("串口打开失败: %s", e)
        return -1

    out_file = None
    frame_log = None
    try:
        rospy.loginfo("串口打开成功: /dev/imu, 230400")
        # 创建目录
        dir_path = "~/nvidia/nrt_ws/"
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
    
        out_file = open(dir_path + "/True_lon_lat.txt", "w")
        frame_log = FrameLog(
            out_file, rospy.loginfo,
            log_interval=rospy.get_param('~log_interval', 1.0),
            flush_interval=rospy.get_param('~log_flush_interval', 1.0))
        rospy.loginfo("文件打开成功")

        buffer = ""
        packet_count = 0
    
        rospy.loginfo("等待串口数据...")

        while not rospy.is_shutdown():
            try:
                if ser.in_waiting > 0:
                    # 读取数据
                    data = ser.read(ser.in_waiting)
                    buffer += data
                
                    # 查找 BD DB 0B
                    while True:
                        # 查找帧头位置
                        pos = buffer.find('\xBD\xDB\x0B')
                        if pos == -1:
                            # 没有找到帧头，保留最后10字节防止截断
                            if len(buffer) > 100:
                                buffer = buffer[-10:]
                            break
                    
                        # 检查是否有足够数据
                        if len(buffer) < pos + 64:
                            break
                    
                        # 找到数据包
                        packet_count += 1
                    
                        # 提取64字节数据包
                        packet = buffer[pos:pos+64]
                    
                        try:
                            # 解析数据 (使用ord处理Python2的字符串)
                            # 横滚角 (偏移3-4)
                            roll_raw = (ord(packet[4]) << 8) | ord(packet[3])
                            if roll_raw & 0x8000:
                                roll_raw = roll_raw - 0x10000
                            roll = roll_raw * 360.0 / 32768.0
                        
                            # 俯仰角 (偏移5-6)
                            pitch_raw = (ord(packet[6]) << 8) | ord(packet[5])
                            if pitch_raw & 0x8000:
                                pitch_raw = pitch_raw - 0x10000
                            pitch = pitch_raw * 360.0 / 32768.0
                        
                            # 航向角 (偏移7-8)
                            heading_raw = (ord(packet[8]) << 8) | ord(packet[7])
                            if heading_raw & 0x8000:
                                heading_raw = heading_raw - 0x10000
                            heading = heading_raw * 360.0 / 32768.0
                        
                            # 纬度 (偏移21-24, 小端32位)
                            lat_raw = (ord(packet[24]) << 24) | (ord(packet[23]) << 16) | (ord(packet[22]) << 8) | ord(packet[21])
                            if lat_raw & 0x80000000:
                                lat_raw = lat_raw - 0x100000000
                            latitude = lat_raw * 0.0000001
                        
                            # 经度 (偏移25-28, 小端32位)
                            lon_raw = (ord(packet[28]) << 24) | (ord(packet[27]) << 16) | (ord(packet[26]) << 8) | ord(packet[25])
                            if lon_raw & 0x80000000:
                                lon_raw = lon_raw - 0x100000000
                            longitude = lon_raw * 0.0000001
                        
                            # 高度 (偏移29-32, 小端32位)
                            alt_raw = (ord(packet[32]) << 24) | (ord(packet[31]) << 16) | (ord(packet[30]) << 8) | ord(packet[29])
                            if alt_raw & 0x80000000:
                                alt_raw = alt_raw - 0x100000000
                            altitude = alt_raw * 0.001
                        
                            # 发布IMU
                            imu_msg.header.stamp = rospy.Time.now()
                            imu_msg.orientation.z = heading
                            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
                            imu_pub.publish(imu_msg)
                        
                            # 发布GPS
                            gps_msg.header.stamp = rospy.Time.now()
                            gps_msg.latitude = latitude
                            gps_msg.longitude = longitude
                            gps_msg.altitude = altitude
                            gps_msg.position_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
                            gps_msg.position_covariance_type = 1
                            GPS_pub.publish(gps_msg)
                        
                            # 写入文件
                            frame_log.record(packet_count, latitude, longitude,
                                             heading, altitude)
                        
                        except Exception as e:
                            rospy.logerr("解析错误: %s", e)
                    
                        # 移除已处理的数据包
                        buffer = buffer[pos+64:]
                    
                else:
                    rospy.sleep(0.01)
                # Flush on schedule even when no new serial bytes arrive.
                frame_log.tick()
                
            except Exception as e:
                rospy.logerr("主循环错误: %s", e)
                rospy.sleep(0.1)

    finally:
        try:
            if frame_log is not None:
                frame_log.close()
            elif out_file is not None:
                try:
                    out_file.flush()
                finally:
                    out_file.close()
        finally:
            ser.close()
    return 0

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr("未预期错误: %s", e)
