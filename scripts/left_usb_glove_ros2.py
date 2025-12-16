import time
import serial
from serial.tools import list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Constants
MAX_PROTOCOL_DATA_SIZE = 64

# Protocol states
WAIT_ON_HEADER_0 = 0
WAIT_ON_HEADER_1 = 1
WAIT_ON_BYTE_COUNT = 2
WAIT_ON_DATA = 3
WAIT_ON_LRC = 4

# Protocol byte name
DATA_CNT_BYTE_NUM = 0
DATA_START_BYTE_NUM = 1


# Glove bus context
class OGlove:
    def __init__(self, serial_port, timeout):
        """
        Initialize OGlove.

        Parameters
        ----------
        serial_port : serial.Serial
            Serial port object
        timeout : int
            Timeout in milliseconds
        """
        self.serial_port = serial_port
        self.timeout = timeout
        self.is_whole_packet = False
        self.decode_state = WAIT_ON_HEADER_0
        self.packet_data = bytearray(MAX_PROTOCOL_DATA_SIZE + 2)  # Including byte_cnt, data[], lrc
        self.byte_count = 0

    @staticmethod
    def calc_lrc(lrc_bytes, lrc_byte_count):
        """Calculate the LRC for a given sequence of bytes"""
        lrc = 0
        for i in range(lrc_byte_count):
            lrc ^= lrc_bytes[i]
        return lrc

    def on_data(self, data):
        """State machine to decode the packet"""
        if self is None or self.is_whole_packet:
            return

        if self.decode_state == WAIT_ON_HEADER_0:
            if data == 0x55:
                self.decode_state = WAIT_ON_HEADER_1

        elif self.decode_state == WAIT_ON_HEADER_1:
            self.decode_state = WAIT_ON_BYTE_COUNT if data == 0xAA else WAIT_ON_HEADER_0

        elif self.decode_state == WAIT_ON_BYTE_COUNT:
            self.packet_data[DATA_CNT_BYTE_NUM] = data
            self.byte_count = data

            if self.byte_count > MAX_PROTOCOL_DATA_SIZE:
                self.decode_state = WAIT_ON_HEADER_0
            elif self.byte_count > 0:
                self.decode_state = WAIT_ON_DATA
            else:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_DATA:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM] - self.byte_count] = data
            self.byte_count -= 1

            if self.byte_count == 0:
                self.decode_state = WAIT_ON_LRC

        elif self.decode_state == WAIT_ON_LRC:
            self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]] = data
            self.is_whole_packet = True
            self.decode_state = WAIT_ON_HEADER_0

        else:
            self.decode_state = WAIT_ON_HEADER_0

    def get_data(self, resp_bytes) -> bool:
        """Retrieve a complete packet from the serial port and validate it"""
        if self is None or self.serial_port is None:
            return False

        wait_start = time.time()
        wait_timeout = wait_start + self.timeout / 1000

        while not self.is_whole_packet:
            while self.serial_port.in_waiting > 0:
                data_bytes = self.serial_port.read(self.serial_port.in_waiting)
                for ch in data_bytes:
                    self.on_data(ch)
                if self.is_whole_packet:
                    break

            if (not self.is_whole_packet) and (wait_timeout < time.time()):
                self.decode_state = WAIT_ON_HEADER_0
                return False

        # Validate LRC
        lrc = self.calc_lrc(self.packet_data, self.packet_data[DATA_CNT_BYTE_NUM] + 1)
        if lrc != self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]]:
            self.is_whole_packet = False
            return False

        # Copy response data
        if resp_bytes is not None:
            packet_byte_count = self.packet_data[DATA_CNT_BYTE_NUM]
            resp_bytes.clear()
            resp_data = self.packet_data[DATA_START_BYTE_NUM : DATA_START_BYTE_NUM + packet_byte_count]
            resp_bytes.extend(resp_data)

        self.is_whole_packet = False
        return True


def find_comport():
    """自动查找可用串口"""
    ports = list_ports.comports()
    for port in ports:
        if "USB" in port.description or "Serial" in port.description:
            return port.device
    return None


class GlovePublisher(Node):
    def __init__(self):
        super().__init__('left_usb_glove_publisher')
        self.data_pub = self.create_publisher(JointState, '~/glove_finger_data', 10)
        
        # configure serial port
        self.serial_port = serial.Serial(
            port=find_comport() or "/dev/ttyACM0",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        
        self.get_logger().info(f"Using serial port: {self.serial_port.name}")
        self.oglove = OGlove(serial_port=self.serial_port, timeout=2000)
        self.glove_data = bytearray()
        
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.oglove.get_data(self.glove_data):
            left_or_right = None
            offset = 0
            finger_data = []

            if len(self.glove_data) & 0x01 == 1:
                left_or_right = self.glove_data[0]
                offset = 1

            for i in range(int(len(self.glove_data) / 2)):
                raw_value = (self.glove_data[offset + i * 2]) | (self.glove_data[offset + i * 2 + 1] << 8)
                finger_data.append(raw_value)

            # publish JointState message
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = "left_glove"
            
            joint_msg.name = [
                "thumb", 
                "index", 
                "middle", 
                "ring", 
                "little", 
                "thumb_root"
            ]
            
            float_data = [float(val) for val in finger_data]
            
            # publish JointState message
            if len(float_data) > len(joint_msg.name):
                joint_msg.position = float_data[:len(joint_msg.name)]
            else:
                joint_msg.position = float_data + [0.0]*(len(joint_msg.name)-len(float_data))  # 补0.0而非0
            
            joint_msg.velocity = []
            joint_msg.effort = []
            
            self.data_pub.publish(joint_msg)
            
            # log output if wrong side
            if left_or_right == 1:
                self.get_logger().info(
                    f"Warning! Published - Side wrong: {left_or_right}, Data: {dict(zip(joint_msg.name, joint_msg.position))}"
                )

    def destroy_node(self):
        self.serial_port.close()
        self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    glove_publisher = GlovePublisher()
    
    try:
        rclpy.spin(glove_publisher)
    except KeyboardInterrupt:
        glove_publisher.get_logger().info("User interrupted")
    finally:
        glove_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

