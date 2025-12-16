import time
import asyncio
import serial
import threading
from serial.tools import list_ports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

MAX_PROTOCOL_DATA_SIZE = 64
WAIT_ON_HEADER_0 = 0
WAIT_ON_HEADER_1 = 1
WAIT_ON_BYTE_COUNT = 2
WAIT_ON_DATA = 3
WAIT_ON_LRC = 4
DATA_CNT_BYTE_NUM = 0
DATA_START_BYTE_NUM = 1
NUM_FINGERS = 6

class OGlove:
    def __init__(self, serial_port, timeout):
        self.serial_port = serial_port
        self.timeout = timeout
        self.is_whole_packet = False
        self.decode_state = WAIT_ON_HEADER_0
        self.packet_data = bytearray(MAX_PROTOCOL_DATA_SIZE + 2)
        self.byte_count = 0
        self.send_buf = bytearray(MAX_PROTOCOL_DATA_SIZE + 4)
        self._cali_min = [65535] * NUM_FINGERS
        self._cali_max = [0] * NUM_FINGERS
        self._glove_raw_data = bytearray()
        self._offset = 0
    
    def clamp(self, n, smallest, largest):
        return max(smallest, min(n, largest))
    
    def interpolate(self, n, from_min, from_max, to_min, to_max):
        return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

    @staticmethod
    def calc_lrc(lrc_bytes, lrc_byte_count):
        lrc = 0
        for i in range(lrc_byte_count):
            lrc ^= lrc_bytes[i]
        return lrc

    def on_data(self, data):
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
            
        lrc = self.calc_lrc(self.packet_data, self.packet_data[DATA_CNT_BYTE_NUM] + 1)
        if lrc != self.packet_data[DATA_START_BYTE_NUM + self.packet_data[DATA_CNT_BYTE_NUM]]:
            self.is_whole_packet = False
            return False
        
        if resp_bytes is not None:
            packet_byte_count = self.packet_data[DATA_CNT_BYTE_NUM]
            resp_bytes.clear()
            resp_data = self.packet_data[DATA_START_BYTE_NUM : DATA_START_BYTE_NUM + packet_byte_count]
            resp_bytes.extend(resp_data)

        self.is_whole_packet = False
        return True
    
    async def start(self) -> bool:
        left_or_right = None

        self.get_data(self._glove_raw_data)
        len_glove_data = len(self._glove_raw_data)
        if len_glove_data % 2 == 0:
            print(f"left_or_right None")
        else:
            left_or_right = self._glove_raw_data[0]
            self._offset = 1
        
        if left_or_right == 0:
            print(f"Warning! Published - Side wrong: {left_or_right}")

        for _ in range(512):
            self.get_data(self._glove_raw_data)
            glove_data = []
            
            for i in range(int(len(self._glove_raw_data) / 2)):
                glove_data.append((self._glove_raw_data[self._offset + i * 2]) | (self._glove_raw_data[self._offset + i * 2 + 1] << 8))
            glove_data_sum = [0 for _ in range(len(glove_data))]

            for _ in range(len(glove_data)):
                for i in range(len(glove_data)):
                    glove_data_sum[i] += glove_data[i]
                    
            for i in range(len(glove_data)):
                temp = glove_data_sum[i] / len(glove_data)
                self._cali_min[i] = min(self._cali_min[i], temp)
                self._cali_max[i] = max(self._cali_max[i], temp)

        for i in range(NUM_FINGERS):
            print(f"Min/Max of finger {i}: {self._cali_min[i]}--{self._cali_max[i]}")
            if self._cali_min[i] >= self._cali_max[i]:
                return False
        return True

    async def get_position(self):
        finger_data = [0 for _ in range(NUM_FINGERS)]

        if self.get_data(self._glove_raw_data):
            glove_data = []

            for i in range(int(len(self._glove_raw_data) / 2)):
                glove_data.append((self._glove_raw_data[self._offset + i * 2]) | (self._glove_raw_data[self._offset + i * 2 + 1] << 8))
            
            glove_data_sum = [0 for _ in range(len(glove_data))]

            for j in range(len(glove_data)):
                for i in range(len(glove_data)):
                    glove_data_sum[i] += glove_data[i]

            for i in range(NUM_FINGERS):
                glove_data[i] = (glove_data[i] * 3 + glove_data_sum[i] / len(glove_data)) / 4
                finger_data[i] = round(self.interpolate(glove_data[i], self._cali_min[i], self._cali_max[i], 65535, 0))
                finger_data[i] = self.clamp(finger_data[i], 0, 65535)
        else:
            finger_data = []
        return finger_data

def find_comport():
    ports = list_ports.comports()
    for port in ports:
        if "USB" in port.description or "Serial" in port.description:
            return port.device
    return None

class GlovePublisher(Node):
    def __init__(self):
        super().__init__('right_usb_glove_publisher')
        self.data_pub = self.create_publisher(JointState, '~/glove_finger_data', 10)
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

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.loop_thread.start()
        self.timer = self.create_timer(0.01, self.timer_callback)

    def run_event_loop(self):
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    def timer_callback(self):
        asyncio.run_coroutine_threadsafe(self.async_get_and_publish(), self.loop)

    async def async_get_and_publish(self):
        finger_data = await self.oglove.get_position()

        if finger_data:

            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.header.frame_id = "right_glove"
            joint_msg.name = ["thumb", "index", "middle", "ring", "little", "thumb_root"]
            float_data = [float(val) for val in finger_data]
            if len(float_data) > len(joint_msg.name):
                joint_msg.position = float_data[:len(joint_msg.name)]
            else:
                joint_msg.position = float_data + [0.0]*(len(joint_msg.name)-len(float_data))
            joint_msg.velocity = []
            joint_msg.effort = []
            self.data_pub.publish(joint_msg)
            

    def destroy_node(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        self.serial_port.close()
        self.get_logger().info("Serial port closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    glove_publisher = GlovePublisher()
    executor = MultiThreadedExecutor()
    print("start calibration, please keep making a fist and spreading movements...")
    init_success = asyncio.run(glove_publisher.oglove.start())
    if not init_success:
        print("Failed to initialize, exit.")
        glove_publisher.destroy_node()
        rclpy.shutdown()
        exit(-1)
    try:
        rclpy.spin(glove_publisher, executor=executor)
    except KeyboardInterrupt:
        glove_publisher.get_logger().info("User interrupted")
    finally:
        glove_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
