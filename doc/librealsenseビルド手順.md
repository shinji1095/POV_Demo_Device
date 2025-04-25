```shell
sudo apt install v4l-utils
```

```shell
$ cd ~/librealsense
$ ./scripts/setup_udev_rules.sh
```

```shell
mkdir build && cd build

cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=$(which python3) \
  -DBUILD_WITH_LIBUSB_BACKEND=ON \
  -DBUILD_NETWORK_DEVICE=OFF \
  -DBUILD_CV_EXAMPLES=OFF \
  -DBUILD_EXAMPLES=OFF

make -j$(nproc)
sudo make install
```

```shell
$ rs-enumerate-devices
Device info: 
    Name                          :     Intel RealSense D455
    Serial Number                 :     141322251250
    Firmware Version              :     5.16.0.1
    Recommended Firmware Version  :     5.16.0.1
    Physical Port                 :     2-2-3
    Debug Op Code                 :     15
    Advanced Mode                 :     YES
    Product Id                    :     0B5C
    Camera Locked                 :     YES
    Usb Type Descriptor           :     2.1
    Product Line                  :     D400
    Asic Serial Number            :     131323060151
    Firmware Update Id            :     131323060151
    Dfu Device Path               : 

Stream Profiles supported by Stereo Module
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Infrared    1280x720       UYVY        @ 5 Hz
    Infrared        |          BGRA8       @ 5 Hz
    Infrared        |          RGBA8       @ 5 Hz
    Infrared        |          BGR8        @ 5 Hz
    Infrared        |          RGB8        @ 5 Hz
    Infrared     848x480       UYVY        @ 10/5 Hz
    Infrared        |          BGRA8       @ 10/5 Hz
    Infrared        |          RGBA8       @ 10/5 Hz
    Infrared        |          BGR8        @ 10/5 Hz
    Infrared        |          RGB8        @ 10/5 Hz
    Infrared     640x480       UYVY        @ 30/15/5 Hz
    Infrared        |          BGRA8       @ 30/15/5 Hz
    Infrared        |          RGBA8       @ 30/15/5 Hz
    Infrared        |          BGR8        @ 30/15/5 Hz
    Infrared        |          RGB8        @ 30/15/5 Hz
    Infrared     640x360       UYVY        @ 30 Hz
    Infrared        |          BGRA8       @ 30 Hz
    Infrared        |          RGBA8       @ 30 Hz
    Infrared        |          BGR8        @ 30 Hz
    Infrared        |          RGB8        @ 30 Hz
    Infrared     480x270       UYVY        @ 60/30/15/5 Hz
    Infrared        |          BGRA8       @ 60/30/15/5 Hz
    Infrared        |          RGBA8       @ 60/30/15/5 Hz
    Infrared        |          BGR8        @ 60/30/15/5 Hz
    Infrared        |          RGB8        @ 60/30/15/5 Hz
    Infrared 1  1280x720       Y8          @ 5 Hz
    Infrared 1   848x480       Y8          @ 10/5 Hz
    Infrared 1   640x480       Y8          @ 30/15/5 Hz
    Infrared 1   640x360       Y8          @ 30 Hz
    Infrared 1   480x270       Y8          @ 60/30/15/5 Hz
    Depth       1280x720       Z16         @ 5 Hz
    Depth        848x480       Z16         @ 10/5 Hz
    Depth        640x480       Z16         @ 30/15/5 Hz
    Depth        640x360       Z16         @ 30 Hz
    Depth        480x270       Z16         @ 60/30/15/5 Hz
    Depth        256x144       Z16         @ 90 Hz

Stream Profiles supported by RGB Camera
 Supported modes:
    STREAM      RESOLUTION     FORMAT      FPS
    Color       1280x800       RGB8        @ 8 Hz
    Color           |          Y8          @ 8 Hz
    Color           |          BGRA8       @ 8 Hz
    Color           |          RGBA8       @ 8 Hz
    Color           |          BGR8        @ 8 Hz
    Color           |          YUYV        @ 8 Hz
    Color       1280x720       RGB8        @ 15/10/5 Hz
    Color           |          Y8          @ 15/10/5 Hz
    Color           |          BGRA8       @ 15/10/5 Hz
    Color           |          RGBA8       @ 15/10/5 Hz
    Color           |          BGR8        @ 15/10/5 Hz
    Color           |          YUYV        @ 15/10/5 Hz
    Color        640x480       RGB8        @ 30/15/5 Hz
    Color           |          Y8          @ 30/15/5 Hz
    Color           |          BGRA8       @ 30/15/5 Hz
    Color           |          RGBA8       @ 30/15/5 Hz
    Color           |          BGR8        @ 30/15/5 Hz
    Color           |          YUYV        @ 30/15/5 Hz
    Color        424x240       RGB8        @ 60/30/15/5 Hz
    Color           |          Y8          @ 60/30/15/5 Hz
    Color           |          BGRA8       @ 60/30/15/5 Hz
    Color           |          RGBA8       @ 60/30/15/5 Hz
    Color           |          BGR8        @ 60/30/15/5 Hz
    Color           |          YUYV        @ 60/30/15/5 Hz

Stream Profiles supported by Motion Module
 Supported modes:
    STREAM      FORMAT         FPS
    Accel       MOTION_XYZ32F  @ 200/100 Hz
    Gyro        MOTION_XYZ32F  @ 400/200 Hz
```

