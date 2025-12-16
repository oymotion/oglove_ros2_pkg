# Package Glove

Nodes for USB and BLE Glove.

## 1. Prepare

```BASH
cd /path/to/workspace  # Should be ~/ros2_ws/src

# Create a vertual env for python
virtualenv -p python3 ./venv

# Make sure that colcon doesn’t try to build the venv
touch ./venv/COLCON_IGNORE

# Activate
source ./venv/bin/activate

# Install Module
pip install bleak==0.22.3

```

Edit `~/bashrc` and add virtual env lib path to PYTHONPATH

```BASH
export PYTHONPATH=$PYTHONPATH:~/ros2_ws/src/venv/lib/python3.12/site-packages  # Modify python3.12 to your actual version
source ~/.bashrc
```

## 2. Compile

```BASH
cd /path/to/workspace
colcon build
```

## 3. Node usb_glove_node

Listens to topic 'glove_finger_data'.

### 3.1 Topics

#### 3.1.1 USB glove

| Topic                                          | Description                         |
| ---------------------------------------------- | ----------------------------------- |
| "/left_usb_glove_publisher/glove_finger_data"  | Data collected from the left glove  |
| "/right_usb_glove_publisher/glove_finger_data" | Data collected from the right glove |

#### 3.1.2 BLE glove

| Topic                           | Description                                           |
| ------------------------------- | ----------------------------------------------------- |
| "/glove_xxxx/glove_finger_data" | Data collected from the BLE glove,xxxx is ble address |

### 3.2 Run

```BASH
# Prepare package
source /path/to/workspace/install/setup.bash

```

Insert glove then add permission to users.Run following command every time you plug in your glove

```BASH
sudo chmod o+rw /dev/ttyACM0  # Modify ttyACM0 to your actual device name
# if double gloves
sudo chmod o+rw /dev/ttyACM1  # Modify ttyACM1 to your actual device name
```

Run usb glove node according to your glove side

```BASH
ros2 oglove_ros2 usb_glove_ros2    # If usb glovwe
ros2 oglove_ros2 ble_glove_ros2    # If ble glove
```

#### 3.2.3 Calibration

After the program starts, keep making a fist and spreading movements according to the tips as quickly as possible
