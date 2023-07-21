# Event Blob Tracking: An Asynchronous Real-Time Algorithm

<p align="center">
  <a href="https://www.youtube.com/watch?v=1W2DC7BI-uc">
    <img src="figures/video_thumbnail.png" alt="Event Blob Tracking: An Asynchronous Real-Time Algorithm" width="500"/>
  </a>
</p>



## For academic use only
Event-based cameras have become increasingly popular for tracking fast-moving objects due to their high temporal resolution, low latency, and high dynamic range. In this paper, we propose a novel algorithm for tracking event blobs using raw events asynchronously in real time. We introduce the concept of an event blob as a spatio-temporal likelihood of event occurrence where the conditional spatial likelihood is blob-like. Many real-world objects generate event blob data, for example, flickering LEDs such as car headlights or any small foreground object moving against a static or slowly varying background. The proposed algorithm uses a nearest neighbour classifier with a dynamic threshold criteria for data association coupled with a Kalman filter to track the event blob state. Our algorithm achieves highly accurate tracking and event blob shape estimation even under challenging lighting conditions and high-speed motions. The microsecond time resolution achieved means that the filter output can be used to derive secondary information such as time-to-contact or range estimation, that will enable applications to real-world problems such as collision avoidance in autonomous driving.


Ziwei Wang, Timothy Molloy, Pieter van Goor and Robert Mahony

[[Preprint Paper](https://arxiv.org/abs/2307.10593)]

## Citation
If you use or discuss our event blob tracking method, please cite our paper as follows:

<pre>

@Article{Wang_2023_Event,
    author    = {Wang, Ziwei and Molloy, Timothy and van Goor, Pieter and Mahony, Robert},
    title     = {Event Blob Tracking: An Asynchronous Real-Time Algorithm},
    journal   = arxiv,
    year      = 2023,
    month     = July,
    url       = {https://arxiv.org/abs/2307.10593},
    arxivid   = {2307.10593}
}
</pre>

## Notes
Should you have any questions or suggestions, please don't hesitate to get in touch with ziwei.wang1@anu.edu.au

