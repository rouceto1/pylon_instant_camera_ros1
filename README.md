ROS1 version heavily based on [the ROS2 version](https://github.com/fhwedel-hoe/pylon_instant_camera). For more description look there. 

Tested with a Basler a2A1920-51gcPRO camera Ubuntu 20.04 ROS1 Noetic.

The code is done as a nodelet so we can use zero-copy communication between other camera nodes such as debayerisation.

### Usage

#### Topic

If the camera provides pixels in bayer pattern (raw), the default topic name is `image_raw`.  
If the camera provides pixels in RGB (color) or MONO (grayscale), the default topic name is `image`.  
The system is using imageTransport.

#### Launchfile 

Starts a nodelet with the basic driver 
The camera will be opened using its default parameters:

    roslauch pylon_instant_camera_ros1 camera.launch

#### Settings

Dummy config files for camrea calibration and camera settings are provided. 
If no "pfs" file is provided the camera runs on internal settings. 

#### Multiple Cameras

In order to select one of multiple cameras, you may supply one or more parameter to identify the particular camera:

* `serial_number` – Can be viewed in the pylon Viewer.
* `user_defined_name` – Can be set via the pylon Viewer.
* `ip_address` – For ethernet cameras.
* `full_name` – For USB cameras. This value may change upon device re-connect. Use with caution.

These values are also displayed in the log.

### Known Issues

#### Framerate-timeout dependece 
The node waits predefined amount of time for each image. 
If the image is not generated because the camera is set to low framerate it can display warning about dropped iamges. 
Current workaround is to set parameter "grab_timeout" to 1/FPS. 

#### Compression beyond
I was not able to make the compression beyond work properly therefore it is currently not working.
