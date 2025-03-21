# Asynchronous Blob Tracker for Event Cameras

<p align="center">
  <a href="https://youtu.be/L_wJjhcToOU">
    <img src="figures/video_thumbnail.png" alt="Event Blob Tracking: An Asynchronous Real-Time Algorithm" width="500"/>
  </a>
</p>


## For academic use only
Event-based cameras are popular for tracking fast-moving objects due to their high temporal resolution, low latency, and high dynamic range. In this paper, we propose a novel algorithm for tracking event blobs using raw events asynchronously in real time. We introduce the concept of an event blob as a spatio-temporal likelihood of event occurrence where the conditional spatial likelihood is blob-like. Many real-world objects such as car headlights or any quickly moving foreground objects generate event blob data. The proposed algorithm uses a nearest neighbour classifier with a dynamic threshold criteria for data association coupled with an extended Kalman filter to track the event blob state. Our algorithm achieves highly accurate blob tracking, velocity estimation, and shape estimation even under challenging lighting conditions and high-speed motions (> 11000 pixels/s). The microsecond time resolution achieved means that the filter output can be used to derive secondary information such as time-to-contact or range estimation, that will enable applications to real-world problems such as collision avoidance in autonomous driving.

The paper was accepted by the 2024 IEEE Transactions on Robotics (TRO). 

Ziwei Wang, Timothy Molloy, Pieter van Goor and Robert Mahony

[[PDF](https://arxiv.org/abs/2307.10593)] [[IEEE Xplore](https://aus01.safelinks.protection.outlook.com/?url=https%3A%2F%2Fieeexplore.ieee.org%2Fdocument%2F10665915&data=05%7C02%7Cziwei.wang1%40anu.edu.au%7C2e08dda55bb843ef030408dcce0aa7ff%7Ce37d725cab5c46249ae5f0533e486437%7C0%7C0%7C638611792406135514%7CUnknown%7CTWFpbGZsb3d8eyJWIjoiMC4wLjAwMDAiLCJQIjoiV2luMzIiLCJBTiI6Ik1haWwiLCJXVCI6Mn0%3D%7C0%7C%7C%7C&sdata=CPSoyUFhRltD8mFhVy4BSLGgRP4KWhWmAgY6miA1uio%3D&reserved=0)]

## Citation
If you use or discuss our event blob tracking method, please cite our paper as follows:

<pre>

@Article{2024_Wang_AEB_Tracker_TRO,
  author  = {Ziwei Wang and Timothy Molloy and Pieter {van Goor} and Robert Mahony},
  journal = {IEEE Transactions on Robotics},
  title   = {Asynchronous Blob Tracker for Event Cameras},
  year    = {2024},
  volume  = {40},
  pages   = {4750-4767},
  issn    = {1552-3098},
  doi     = {10.1109/TRO.2024.3454410},
}
</pre>


## Code and Data

### Installation
Dependencies:


- [OpenCV](https://opencv.org/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)



Example of installing: 
```
sudo apt update
sudo apt-get install libyaml-cpp-dev
sudo apt install libeigen3-dev
```
Tested Eigen3 version = 3.4. 
Note: You may have to specify the path to your OpenCV/yaml-cpp/Eigen library in `CMakeLists.txt`.

### Build
Our asynchronous comb filter is designed to be built as a cmake project. Assuming all prerequisites are installed and you are in the root folder of the repository, then you can follow these steps to build. 

***Run in release mode to track fast!***



```
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build . -j8
```

### Data
[Click here to download the example dataset.](https://drive.google.com/file/d/1LE6F88P8_UvCMla8c1LcfkGykKmQOedn/view?usp=sharing)


Unzip `data.zip` into `./data`

### Run  

```
    cd build
    ./aeb_tracker -i {config_file_name}
```
Choose the {config_file_name} from `car`, `spin-dot` or `shapes`. E.g., `./aeb-tracker -i car`

### Configures
Note that most of the parameters do not need to be tuned, or they are not sensitive to different scenarios. For your own dataset, you can start with the configuration file of a similar dataset from the provided example data and adjust the parameters accordingly.
You can modify the parameters in configure files in `./configs`.

#### Key parameters require tuning:

1. Kalman filter parameters. `var_` represents state covariances and `q_` for process noise. We initialize targets with zero initial linear and angular velocity, and the initial target size is set to a value of at least two times larger than the maximum expected blob size to make the initial transient of the filter more robust.
See Section V.A. in the paper for more discussions.

2. `dist_threshold`: distance threshold (in pixels) for data association. Tune this number based on the size of the target and the noisy background level. `dist_threshold = 10` is a good number for relatively small targets. Do not use extremely small `dist_threshold` for very fast-moving targets.

#### Key parameters for running and saving data:

1. `select_target_flag`: 0 for using pre-set `position_x_init` and `position_y_init` in the config file; 1 for selecting targets by clicking on the video (press the Enter key to confirm). `n_target` is the number of targets to track. Adjust `n_target` accordingly.

2. Only run parts of the datasets: set `process_ts_start` and `process_ts_end` to control timestamps; or `event_num_start` and `event_num_end` to control event IDs to run. Only use one option and set the other option to `-1`.

3. `alpha`: decay rate of the event-based image reconstructions. Only for display.  

4. `save_video_flag`, `save_image_flag`, `save_track_falg`: 1 for saving video/image/tracks. Saving files slows down the program. Turn off unnecessary saving options.

5. `ref_image_ts_flag`: 1 for loading reference RGB images and timestamps. In the `shapes` dataset, we provide an example of loading and displaying RGB images along with the event-reconstructed images. RGB images and image timestamps are loaded from `{data_set_path}/reproject_rgb/` and `{data_set_path}/image_ts.txt`.


### Example to compute and save Time-to-Contact (TTC):
Run `./aeb-tracker -i car`: TTC results will be saved to file `./data/car-rgb-ref/03/03_ttc.txt`. You can turn off TTC by setting `compute_TTC_flag = 0` in the config file. Please note that the current version only supports computing TTC from a pair of targets (two targets).


### Example to use IMU data:
Run `./aeb-tracker -i shapes`: IMU data will be loaded from file `./data/shapes/08/gyro.txt`. You can turn off IMU data loading by setting `use_gyro_flag = 0` in the config file. IMU data are used for velocity updates in the current version. If you want to test on your own dataset, please modify the IMU and camera parameters, e.g., IMU bias and camera focal length in function `load_gyro` in `main.cpp`.


### Example to click to track:
Run `./aeb-tracker -i spin-dot`: Click target on screen to track. Default target `n_target = 1`, so you don’t need to press the Enter key.
The dot spins from very slow to very fast.



## Notes
Should you have any questions or suggestions, please don't hesitate to get in touch with ms.ziweiwang@gmail.com.

