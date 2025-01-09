# Surface PC
Grab a Surface tablet and make sure it is running ubuntu. If not go into Windows -> Settings -> Recovery -> Advanced Startup -> Restart Now -> Use a Device -> Ubuntu

If you wish to program on your own computer rather than on the Surface, you can connect to it through SSH. Run
```
ifconfig
```
in a terminal (on the Surface) and find the IP address for the WiFi or go into the WiFi settings to find it.

On your own PC click the button in the lower left corner and click on "Connect to Host..." and write robot@"IP address of Surface". Select Linux when asked and the password is "iir_robot".

# Task 1: Connecting a Camera
Connect a USB camera to the Surface and run
```
v4l2-ctl --list-devices
```
You will see whichever "/dev/videoX" belongs to the USB camera. There are multiple "/dev/videoX" per camera. Often the one with the lowest number is the correct one, but try higher ones if not.

Run
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/videoX
```
This will start the usb_cam node with a parameter "video_device" as the USB camera.

If the PC does not have usb_cam installed, run
```
sudo apt install ros-humble-usb-cam
```
with iir_robot as the password

In a new terminal run
```
rqt
```
And go to Plugins -> Visualization -> Image Plot and make sure the camera is working.
## Optional: Calibrating a Camera
If you wish to calibrate the camera, you can do that. [This repository](https://github.com/ros-perception/image_pipeline/tree/humble) can calibrate the camera, with documentation [here](http://docs.ros.org/en/rolling/p/image_pipeline/). You can run the command
```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/image_raw camera:=/camera_info
```
which will make the node listen to the "image_raw" and "camera_info" topics and start to calibrate.

Go through the calibration protocol described [here](http://docs.ros.org/en/rolling/p/camera_calibration/index.html#http://) and find a checkerboard sheet and set "size" and "square" according to the tutorial. When the camera is calibrated, you will get a "tar.gz" file which contains the calibration. Put the files in `/home/robot/.ros/camera_info` and run the "usb_cam" node again. It should say that it has found the calibration file. If not, rename the file and move it to the folder which it is looking for.
# Task 2: Gaussian Blur

## Task 2a
Create a new node, either in a new package or an existing one. The new node should have one subscriber subscribing to "image_raw" and one publisher "blurred_image". Both of them should send/receive a sensor_msgs/Image

To create a package with a node the command is
```
ros2 pkg create --build-type ament_python (or ament_cmake if you want to do C++) --node-name gaussian_blur_node gaussian_blur
```
This will create a new package called `gaussian_blur` and begin a basic node which is called `gaussian_blur_node` which you can find in the src folder (in C++)  or the gaussian_blur folder (in Python).

You can use this (Python) code to get you started

``` python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node.
        """
        super().__init__('gaussian_blur')

        # Subscribe to a image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the canny edge detection image
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()
    

    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image and publishes the edges as an image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return

        # Convert back to ROS Image message
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(edges, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        # Publish the filtered image
        self.publisher.publish(edge_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    image_blur_node = GaussianBlurNode()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Every time the node receives an image message from "image_raw" the node should take the image add a Gaussian blur to it and publish the blurred image to "blurred_image". The blur should be a 5x5 kernel

Tips: You can use OpenCV2 to do a Gaussian blur, since it has a functon for it. You can also use [CVBridge](https://github.com/ros-perception/vision_opencv/tree/humble) to easily convert Image messages into OpenCV2 arrays and back.

To finish, open a new Image Plot plugin in rqt and look for the "output_image" topic.
## Task 2b
Make the blur kernel (which currently is set to 5x5) changable through parameters. Create one parameter called "gaussian_blur" that is initialized to 5, and use that parameter to set the size of the blue kernel.

You can read more about parameters [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html). You can also read about how to implement parameters in [C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) or [Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html).

You should now be able to start the node like this
```
ros2 run your_package gaussian_blur_node --ros-args -p blur_radius:=9
```
The `-p` shows that `blur_radius` is a parameter which should be changed.

You should also now be able to run
```
ros2 param list
```
to see the parameters of the node and
```
ros2 param set /gaussian_blur_node blur_radius 11
```
to change the blur radius to 11.
# Task 3: Canny Edge Detection
## Task 3a
We will now make a new node which performs Canny Edge Detection on an input image. Same as with Task 2, the node should subscribe to "image_raw" and publish "output_edges". Set the upper threshold of the Canny Edge Detection algorithm should be set to 100 while the lower should be set to 200.

Tips: The resulting image from the OpenCV Canny Edge Detection is grayscale and not RGB. So change "bgr8" to "mono8" in the "convert CV2 to ROS2 Image message" part of the code:
``` python
try:
	edge_msg = self.bridge.cv2_to_imgmsg(edges, "mono8")
except Exception as e:
	self.get_logger().error('Failed to convert image: %s' % str(e))
	return
```

When running the node, you can check the result in `rqt` by making a new Image Plot which shows the "output_edges".
## Task 3b
Make "lower_threshold" and "upper_threshold" parameters so that you can start the node like this
```
ros2 run your_package canny_edge_detection_node --ros-args -p lower_threshold:=50 -p upper_threshold:=150
```

# Task 4: Putting it together
If you run `rqt_graph` you can see that both the Gaussian blur node and the Canny edge detection node subscribe to "image_raw", and we want to create this into a pipeline. We want the image from the USB camera to go through the Gaussian blur filter to remove noise and then to the Canny Edge Detection algorithm to output the edges. We can change subscribers and publishers without changing the code  by remapping them. You can read about remapping [here](https://docs.ros.org/en/rolling/How-To-Guides/Node-arguments.html).

We can therefore run
```
ros2 run your_package canny_edge_detection_node --ros-args -r /image_raw:=/output_image
```
to make the node subscribe to "output_image" from the Gaussian blur node instead. You can verify this on `rqt_graph`.

Now run both nodes and change the parameters on both to find the parameters that gives the best result.