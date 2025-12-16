# Package Glove

Nodes for USB Glove and BLE Glove.

## 1. Prepare

```BASH
cd /path/to/workspace  # Should be ~/ros2_ws/src

# Create a vertual env for python
virtualenv -p python3 ./venv

# Make sure that colcon doesn’t try to build the venv
touch ./venv/COLCON_IGNORE

# Activate
source ./venv/bin/activate

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

| Topic                                          | Description                         |
| ---------------------------------------------- | ----------------------------------- |
| "/left_usb_glove_publisher/glove_finger_data"  | Data collected from the left glove  |
| "/right_usb_glove_publisher/glove_finger_data" | Data collected from the right glove |

### 3.2 Run

```BASH
# Prepare package
source /path/to/workspace/install/setup.bash

```

#### 3.2.1 Use single glove


Insert glove then add permission to users.Run following command every time you plug in your glove

```BASH
sudo chmod o+rw /dev/ttyACM0  # Modify ttyACM0 to your actual device name
```

Run usb glove node according to your glove side

```BASH
ros2 run glove_ros2 left_usb_glove_node    # If left glove
# or
ros2 run glove_ros2 right_usb_glove_node   # If right glove
```

#### 3.2.2 Use a pair of gloves

Insert one glove then add permission to users.Run following command every time you plug in your glove

```BASH
sudo chmod o+rw /dev/ttyACM0  # Modify ttyACM0 to your actual device name
```

Run one glove program first if ACM0 is your left glove, else run right one program

```BASH
ros2 run glove_ros2 left_usb_glove_node    # If left glove
# or
ros2 run glove_ros2 right_usb_glove_node   # If right glove
```

Insert the other glove

```BASH
sudo chmod o+rw /dev/ttyACM1  # Modify ttyACM1 to your actual device name
```

Modify code in left_usb_glove_node.py or right_usb_glove_node.py

```bash
port=find_comport() or "/dev/ttyACM1"   # Modift param when init port
```

run the other glove program

```BASH
ros2 run glove_ros2 right_usb_glove_node   # If second glove is rightt
# or
ros2 run glove_ros2 left_usb_glove_node    # If second glove is left
```
## 4. Node ble_glove_node

Listens to topic 'glove_finger_data'.

### 4.1 Topics

| Topic                           | Description                                           |
| ------------------------------- | ----------------------------------------------------- |
| "/glove_xxxx/glove_finger_data" | Data collected from the BLE glove,xxxx is ble address |

### 4.2 Run

Prepare bleak module
```BASH
pip install bleak==0.22.3
```

```BASH
# Prepare package
source /path/to/workspace/install/setup.bash

# Run this command for each glove
ros2 run glove_ros2 ble_glove_node
```
