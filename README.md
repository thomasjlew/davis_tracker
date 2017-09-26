# davis_tracker
Extracts features from usual images and tracks them using events from an "Event-Camera" such as the DAVIS. Implements this paper [1] with a few modifications.  <br /> <br />
Tested using the dataset provided at http://rpg.ifi.uzh.ch/davis_data.html [2]

## References
- Refer to http://rpg.ifi.uzh.ch/davis_data.html for more information concerning the camera and datasets
- [1] D. Tedaldi, G. Gallego, E. Mueggler, and D. Scaramuzza, “Feature detection and tracking with the dynamic and active-pixel vision sensor (DAVIS),” in Int. Conf. on Event-Based Control, Comm. and Signal Proc. (EBCCSP), Krakow, Poland, Jun. 2016.
- [2] E. Mueggler, H. Rebecq, G. Gallego, T. Delbruck, D. ScaramuzzaThe Event-Camera Dataset and Simulator: Event-based Data for Pose Estimation, Visual Odometry, and SLAM, International Journal of Robotics Research, Vol. 36, Issue 2, pages 142-149, Feb. 2017.

## TODO
- Preallocate memory for model points for faster processing
- Move outliers detection after nearest neighbours search for faster processing 
- Others...
