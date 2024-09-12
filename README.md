# Camera


## Description:
Camera is reponsible for capturing image from CSI or USB and writing them on HDD or Ramdisk with timestamp. Also, makes sure to delete specific number of old files.
contain 3 camera: left, right and center


<details><summary>Create Ramdisk</summary>

1. Create /media/ramdisk/
```
sudo mkdir -p /media/ramdisk/
```
2. Mount /media/ramdisk/
```
sudo mount -t tmpfs -o size=2048M tmpfs /media/ramdisk
```

3. Make the folder writable for other modules (Optional):
```
sudo chown -R jetson /media/*
```

</details>


## build
```bsh
g++ -O3 -Os -std=c++11 -Wall -I/usr/include/opencv4 -o camera camera.cpp  -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d -lgpiod -lyaml-cpp
```


## run
```bsh
 ./camera cam_type1 C_cam L_cam R_cam  visualization  --output-dir  /home/jetson/data/mmm
```
## switchs
```bsh
cam_type1, cam_type2, center_cam, Thermal_cam, CSI_cam, trig_mod, visualization, distortion, --log-dir
```

## Inputs:
3 yaml file for 3 cameras, contain camera calibration parameters and exposure time

## Outputs:

* images of camera on camera folder
* timestamps in a text file 
* yaml file contain propertise of image 


## config output
dir: /media/ramdisk/

output: image_properties.yaml

image_width: 640

image_height: 480

image_format: jpg

compression_quality: 95

horizontal FoV: 86

Vertical FoV: 70


## camera cpp
**Command to build c++ camera module**

* g++ -std=c++11 -Wall -I/usr/include/opencv4 -o camera camera.cpp -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc
* **external_trigger**: v4l2-ctl -d /dev/video0 --set-ctrl=backlight_compensation=2


camera
* how to run: ./camera  cam_type=2 L_cam=off R_cam=off trig_mod=off visualization=on
* how to build: g++ -O3 -Os -std=c++11 -Wall -I/usr/include/opencv4 -o camera camera.cpp -L/usr/lib -lopencv_core -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_calib3d -lgpiod -lyaml-cpp

