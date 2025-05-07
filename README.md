[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS 2 Humble

> This project is based on [mangdangroboticsclub/mini_pupper_ros](https://github.com/mangdangroboticsclub/mini_pupper_ros). Please refer to this work for installation and basic usage.

Additional work and features will be detailed here, please refer to the different section.

## Face detection

This face detection use Facenet on Pytorch using a camera. The detection are published on the topic _/detected\_faces_

The device used for face detection is recommended to be a remote one, not necessarily Mini Pupper.
Run camera launch file or robot bringup on device with camera:
```sh
ros2 launch mini_pupper_driver camera.launch.py 
```
or
```sh
ros2 launch mini_pupper_bringup bringup.launch.py
```

Run the face detection on the device of your choice that can access ROS2 topics.

First download facenet-pytorch:
```sh
pip install facenet-pytorch
```
Then run the face detection:
```sh
ros2 launch mini_pupper_recognition face_recognition.launch.py
```

A rqt plugin has been added to monitor and visualize the detections:
1. In rqt, go to Plugins/Debugging/Image Overlay
2. The image from camera should be displayed with topic _/image\_raw_
3. Add an overlay by pressing + button
4. Choose _vision\_msgs/msg/Detection2DArray_
5. Add topic name _/detected\_faces_


## Remote screen display

The display of images or videos is made via ros2 topic.

**Run the subscriber** linked to the display logic **on the Mini Pupper** with:

```sh
ros2 launch mini_pupper_driver display_interface.launch.py
```

On remote device **run publisher** and indicate the **path of the media** (png, jpg, gif, mp4):

```sh
ros2 run media_publisher media_sender_node path/to/media.png
```

The screen on Mini Pupper should display the media

<details>
<summary><b> Explanation and behavior </b></summary>

The subscriber use by default CompressedImage as message type.
The publisher uses Image Transport package to handle transport type smoothly. It means it will adapt the message type to the topic listening message type.


Currently the only way to change compressed images to raw images is to change logic in [the python display interface launched on Mini Pupper](https://github.com/ais-lab/ais-pupper/blob/a6a78cb6ef4282ac0bd4ad1f206755a7a022fb39/mini_pupper_driver/mini_pupper_driver/display_interface.py#L18):
```python
self.sub = self.create_subscription(CompressedImage, 'mini_pupper_lcd/image/compressed', self.callback, 10)

# Replace by this subscriber if image not compressed needed
#self.sub = self.create_subscription(Image, 'mini_pupper_lcd/image/raw', self.callback, 10)
```
</details>