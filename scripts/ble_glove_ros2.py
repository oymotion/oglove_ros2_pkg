import time
import rclpy
import asyncio
import threading
import os
import sys
import re
from typing import List, Optional

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)
sys.path.append(parent_dir)

from lib_gforce import gforce
from lib_gforce.gforce import EmgRawDataConfig, SampleResolution

# Device filters
DEV_NAME_PREFIX = "gForceBLE"
DEV_MIN_RSSI = -128

# sample resolution:BITS_8 or BITS_12
SAMPLE_RESOLUTION = 12

# Channel0: thumb, Channel1: index, Channel2: middle, Channel3: ring, Channel4: pinky, Channel5: thumb root
INDEX_CHANNELS = [7, 6, 0, 3, 4, 5]

NUM_FINGERS = 6

class PosInputBleGlove:

    def __init__(self):
        self._gforce_device = gforce.GForce(DEV_NAME_PREFIX, DEV_MIN_RSSI)
        self._emg_data = [0 for _ in range(NUM_FINGERS)]
        self._emg_min = [65535 for _ in range(NUM_FINGERS)]
        self._emg_max = [0 for _ in range(NUM_FINGERS)]
        self._q = None

    def clamp(self, n, smallest, largest):
        return max(smallest, min(n, largest))

    def interpolate(self, n, from_min, from_max, to_min, to_max):
        # 将n从from_min到from_max的范围映射到to_min到to_max的范围
        return (n - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

    async def start(self) -> bool:
        # GForce.connect() may get exception, but we just ignore for gloves
        try:
            await self._gforce_device.connect()
        except Exception as e:
            print(e)

        if self._gforce_device.client == None or not self._gforce_device.client.is_connected:
            exit(-1)

        print("Connected to {0}".format(self._gforce_device.device_name))

        # Set the EMG raw data configuration, default configuration is 8 bits, 16 batch_len
        if SAMPLE_RESOLUTION == 12:
            cfg = EmgRawDataConfig(fs=100, channel_mask=0xFF, batch_len=48, resolution=SampleResolution.BITS_12)
            await self._gforce_device.set_emg_raw_data_config(cfg)

        baterry_level = await self._gforce_device.get_battery_level()
        print("电池电量: {0}%\nDevice baterry level: {0}%".format(baterry_level))

        await self._gforce_device.set_subscription(gforce.DataSubscription.EMG_RAW)
        self._q = await self._gforce_device.start_streaming()
        
        range_valid = True

        return range_valid
    
    def get_device_name(self) -> str:
        short_name = re.search(r'\((.*?)\)', self._gforce_device.device_name)
        return short_name.group(1) if short_name else ''

    async def get_position(self):
        v = await self._q.get()
        # print(v)

        finger_data = [0 for _ in range(NUM_FINGERS)]
        emg_sum = [0 for _ in range(NUM_FINGERS)]

        for j in range(len(v)):
            for i in range(NUM_FINGERS):
                emg_sum[i] += v[j][INDEX_CHANNELS[i]]

        for i in range(NUM_FINGERS):
            self._emg_data[i] = (self._emg_data[i] * 3 + emg_sum[i] / len(v)) / 4

            finger_data[i] = round(self.interpolate(self._emg_data[i], self._emg_min[i], self._emg_max[i], 65535, 0))
            finger_data[i] = self.clamp(finger_data[i], 0, 65535)

        return finger_data

    async def stop(self):
        await self._gforce_device.stop_streaming()
        await self._gforce_device.disconnect()
        print("Disconnected from {0}".format(self._gforce_device.device_name))


class GlovePublisher(Node):
    def __init__(self):
        super().__init__('ble_glove_publisher')
        self.bg = PosInputBleGlove()
        self.data_queue: asyncio.Queue[Optional[List[int]]] = asyncio.Queue(maxsize=10)
        self.latest_data: Optional[List[int]] = None
        
        self.loop = asyncio.new_event_loop()
        self.ble_thread = threading.Thread(
            target=self._run_ble_thread,  # 同步线程函数（调用异步主循环）
            daemon=True
        )
        self.ble_thread.start()
        
        self.queue_timer = self.create_timer(0.005, self._sync_data_from_queue)

    # 1. 同步线程函数：仅负责启动异步主循环
    def _run_ble_thread(self):
        asyncio.set_event_loop(self.loop)
        # 执行异步主循环（run_until_complete 会阻塞到异步函数结束）
        self.loop.run_until_complete(self._ble_async_main_loop())

    # 2. 异步主循环函数（async def 定义，可使用 await）
    async def _ble_async_main_loop(self):
        try:
            await self.bg.start()  # 合法：异步函数内可使用 await
            dev_name = self.bg.get_device_name()
            self.get_logger().info("Bluetooth glove initialized successfully")

            ble_publishr_name = f'glove_{dev_name}/glove_finger_data'
            self.data_pub = self.create_publisher(Int32MultiArray, ble_publishr_name, 10)
            self.publish_timer = self.create_timer(0.01, self._publish_callback)
            
            while True:
                raw_data = await self.bg.get_position()  # 合法：异步函数内可使用 await
                if raw_data is not None:
                    valid_data = self._validate_finger_data(raw_data)
                    if valid_data:
                        if self.data_queue.full():
                            try:
                                self.data_queue.get_nowait()
                            except asyncio.QueueEmpty:
                                pass
                        await self.data_queue.put(valid_data)
                await asyncio.sleep(0.005)
        
        except Exception as e:
            self.get_logger().error(f"Bluetooth async loop failed: {str(e)}", throttle_duration_sec=1.0)
            await self.data_queue.put(None)

    def _sync_data_from_queue(self):
        try:
            while not self.data_queue.empty():
                self.latest_data = self.data_queue.get_nowait()
                if self.latest_data is None:
                    self.get_logger().error("Bluetooth connection lost, stopping data sync")
                    self.queue_timer.cancel()
                    break
        except asyncio.QueueEmpty:
            pass
        except Exception as e:
            self.get_logger().warning(f"Failed to sync data from queue: {str(e)}", throttle_duration_sec=1.0)

    def _validate_finger_data(self, raw_data) -> List[int]:
        valid_data = []
        if not isinstance(raw_data, (list, tuple)):
            self.get_logger().warning(f"Invalid data type (raw): {type(raw_data)}", throttle_duration_sec=1.0)
            return valid_data
        
        for val in raw_data:
            try:
                int_val = int(val)
            except (ValueError, TypeError):
                self.get_logger().warning(f"Skipping invalid value: {val}", throttle_duration_sec=1.0)
                continue
            
            clamped_val = max(-2147483648, min(int_val, 2147483647))
            valid_data.append(clamped_val)
        
        return valid_data

    def _publish_callback(self):
        if self.latest_data is None:
            return
        
        data_msg = Int32MultiArray()
        data_msg.data = self.latest_data
        self.data_pub.publish(data_msg)
        # self.get_logger().info(f"Published: {self.latest_data}", throttle_duration_sec=0.1)

    def destroy_node(self):
        self.queue_timer.cancel()
        self.publish_timer.cancel()
        
        if self.loop.is_running():
            tasks = asyncio.all_tasks(loop=self.loop)
            for task in tasks:
                task.cancel()
            self.loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
        self.loop.close()
        self.get_logger().info("Async event loop closed")
        
        if self.ble_thread.is_alive():
            self.ble_thread.join(timeout=1.0)
            if self.ble_thread.is_alive():
                self.get_logger().warning("Bluetooth thread did not exit cleanly")
        
        super().destroy_node()
        self.get_logger().info("Glove node closed")


def main(args=None):
    rclpy.init(args=args)
    glove_publisher = GlovePublisher()
    
    try:
        rclpy.spin(glove_publisher)
    except KeyboardInterrupt:
        glove_publisher.get_logger().info("User interrupted (Ctrl+C)")
    except Exception as e:
        glove_publisher.get_logger().error(f"Node failed: {str(e)}")
    finally:
        glove_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
