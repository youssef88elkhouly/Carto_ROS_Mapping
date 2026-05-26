import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LD06Decoder(Node):
    def __init__(self):
        super().__init__("ld06_decoder")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        self.ser = serial.Serial(
            self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
        )

        self.buffer = bytearray()

        self.ranges = [float("inf")] * 360
        self.intensities = [0.0] * 360

        self.valid_packets = 0
        self.bad_packets = 0

        self.timer = self.create_timer(0.001, self.read_loop)
        self.publish_timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value),
            self.publish_scan,
        )

        self.get_logger().info(
            f"LD06 real decoder started on {self.port} @ {self.baudrate}"
        )

    def read_loop(self):
        data = self.ser.read(4096)
        if data:
            self.buffer.extend(data)

        while len(self.buffer) >= 47:
            # Find LD06 packet header: 0x54 0x2C
            header_index = self.buffer.find(b"\x54\x2c")

            if header_index < 0:
                self.buffer.clear()
                return

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < 47:
                return

            packet = bytes(self.buffer[:47])
            del self.buffer[:47]

            self.parse_packet(packet)

    def parse_packet(self, packet: bytes):
        if len(packet) != 47:
            self.bad_packets += 1
            return

        if packet[0] != 0x54 or packet[1] != 0x2C:
            self.bad_packets += 1
            return

        # LD06 packet format:
        # byte 0      = 0x54
        # byte 1      = 0x2C
        # bytes 2-3   = speed
        # bytes 4-5   = start angle, centi-degrees
        # bytes 6-41  = 12 points, each 3 bytes:
        #               distance low, distance high, intensity
        # bytes 42-43 = end angle, centi-degrees
        # bytes 44-45 = timestamp
        # byte 46     = crc

        start_angle = int.from_bytes(packet[4:6], byteorder="little") / 100.0
        end_angle = int.from_bytes(packet[42:44], byteorder="little") / 100.0

        angle_diff = end_angle - start_angle
        if angle_diff < 0:
            angle_diff += 360.0

        # 12 measurements, so 11 gaps between first and last
        angle_step = angle_diff / 11.0 if angle_diff <= 180.0 else 0.0

        for i in range(12):
            base = 6 + i * 3

            distance_mm = int.from_bytes(packet[base:base + 2], byteorder="little")
            intensity = packet[base + 2]

            angle_deg = start_angle + angle_step * i
            if angle_deg >= 360.0:
                angle_deg -= 360.0

            distance_m = distance_mm / 1000.0

            # Convert angle 0..360 to scan index for angle_min=-pi, angle_max=pi
            signed_angle = angle_deg
            if signed_angle > 180.0:
                signed_angle -= 360.0

            index = int(round(signed_angle + 180.0))

            if 0 <= index < 360:
                if 0.12 <= distance_m <= 4.0:
                    self.ranges[index] = distance_m
                    self.intensities[index] = float(intensity)
                else:
                    self.ranges[index] = float("inf")
                    self.intensities[index] = 0.0

        self.valid_packets += 1

        if self.valid_packets % 500 == 0:
            self.get_logger().info(
                f"valid_packets={self.valid_packets}, bad_packets={self.bad_packets}"
            )

    def publish_scan(self):
        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.radians(1.0)

        msg.time_increment = 0.0
        msg.scan_time = 0.1

        msg.range_min = 0.12
        msg.range_max = 4.0

        msg.ranges = list(self.ranges)
        msg.intensities = list(self.intensities)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LD06Decoder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
