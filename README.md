# davis_tracker
# About this project
This program tracks features using events from an "Event-Camera" such as the DAVIS [1]. The features are first extracted from raw images. This project implements this paper [2] with a few modifications.  <br />

## Results
### Dataset
The code was tested using the dataset provided at http://rpg.ifi.uzh.ch/davis_data.html<br />
The following results were obtained with the *shapes_6dof* dataset from [3].

### Tracking
In this animation, 10 features extracted in the first frame of the dataset are tracked with a precision of approximately 3 pixels in average.<br /><br />
![alt text](https://github.com/thomasjlew/davis_tracker/blob/master/imgs/tracking_shapes_speedx4_new.gif)<br />
Currently, the two following problems arise:
- 1/10 feature on the edge of the triangle is not correctly tracked.<br />
- Another feature moves slowly along the hexagon on the middle right.<br />


Note that they will be fixed using the modifications mentionned in *Further work*.
### Events Registration [2]
This shows the registration sequence as described in [2]. In this animation, the outliers were already removed and are not plotted.<br /><br />
![alt text](https://github.com/thomasjlew/davis_tracker/blob/master/imgs/registration_shapes_new.gif)<br />
**Legend**:<br /> 
-Red stars - model point set<br /> 
-Blue circles - data point set <br />
-Blue cross - feature position

### Plots
Typical obtained plots while tracking are shown below. <br /> <br />
![alt text](https://github.com/thomasjlew/davis_tracker/blob/master/imgs/tracking_1.png)

### Execution time
Currently, plotting functions take up a minimum of 43% of the total processing time, which includes optimizations and removed subplots 2 (edges plot) and 3 (closer view of a patch). The following figure was obtained using the **Matlab profile viewer** and shows the execution time of the program. <br /> <br />
![alt text](https://github.com/thomasjlew/davis_tracker/blob/master/imgs/exec_time.png)

## Further work
- Add the modifications mentionned in [4], such as additionnal *events weights* to cope with the tracking imprecisions
- Fuse this with the complete state estimation algorithm in [4].
- Rewrite the program in C++ using ROS for faster processing. Currently, plotting takes about 30% of the processing time.

## References
- Refer to http://rpg.ifi.uzh.ch/davis_data.html for more information concerning the camera and datasets
- [1] C. Brandli, R. Berner, M. Yang, S.-C. Liu, and T. Delbruck, “A 240x180 130dB 3us Latency Global Shutter Spatiotemporal Vision Sensor,” IEEE J. of Solid-State Circuits, 2014.
- [2] D. Tedaldi, G. Gallego, E. Mueggler, and D. Scaramuzza, “Feature detection and tracking with the dynamic and active-pixel vision sensor (DAVIS),” in Int. Conf. on Event-Based Control, Comm. and Signal Proc. (EBCCSP), Krakow, Poland, Jun. 2016.
- [3] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, D. Scaramuzza, The Event-Camera Dataset and Simulator: Event-based Data for Pose Estimation, Visual Odometry, and SLAM, International Journal of Robotics Research, Vol. 36, Issue 2, pages 142-149, Feb. 2017.
- [4] B. Kueng, E. Mueggler, G. Gallego, D. Scaramuzza, Low-Latency Visual Odometry using Event-based Feature Tracks, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016.
