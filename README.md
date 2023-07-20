# Event Blob Tracking: An Asynchronous Real-Time Algorithm

<p align="center">
  <a href="https://arxiv.org/abs/2208.01710">
    <img src="figures/pipeline.png" alt="Smart Visual Beacons with Asynchronous Optical Communications using Event Cameras" width="1000"/>
  </a>
</p>

<p align="center">
  <a href="https://arxiv.org/abs/2208.01710">
    <img src="figures/system.png" alt="Smart Visual Beacons with Asynchronous Optical Communications using Event Cameras" width="500"/>
  </a>
</p>


## For academic use only
Event-based cameras have become increasingly popular for tracking fast-moving objects due to their high temporal resolution, low latency, and high dynamic range. In this paper, we propose a novel algorithm for tracking event blobs using raw events asynchronously in real time. We introduce the concept of an event blob as a spatio-temporal likelihood of event occurrence where the conditional spatial likelihood is blob-like. Many real-world objects generate event blob data, for example, flickering LEDs such as car headlights or any small foreground object moving against a static or slowly varying background. The proposed algorithm uses a nearest neighbour classifier with a dynamic threshold criteria for data association coupled with a Kalman filter to track the event blob state. Our algorithm achieves highly accurate tracking and event blob shape estimation even under challenging lighting conditions and high-speed motions. The microsecond time resolution achieved means that the filter output can be used to derive secondary information such as time-to-contact or range estimation, that will enable applications to real-world problems such as collision avoidance in autonomous driving.


Ziwei Wang, Timothy Molloy, Pieter van Goor and Robert Mahony

[[arXiv preprint]](https://arxiv.org/abs/2208.01710)

## Citation
If you use or discuss our algorithm or datasets, please cite our paper as follows:
<pre>
@InProceedings{wang22iros,
  author        = {Ziwei Wang and Yonhon Ng and Jack Henderson and Robert Mahony},
  title         = {Smart Visual Beacons with Asynchronous Optical Communications using Event Cameras},
  booktitle     = {"International Conference on Intelligent Robots and Systems (IROS 2022)" },
  year          = {2022}
}
</pre>

## Notes
Should you have any questions or suggestions, please don't hesitate to get in touch with ziwei.wang1@anu.edu.au



# Event-Blob-Tracking
