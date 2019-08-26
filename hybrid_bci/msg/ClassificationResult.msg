# if no face is detected, the following is empty
vision_msgs/BoundingBox2D face_region
sensor_msgs/Image face_image
float32[] face_features

# classification result ("failed", "unknown", or a registered name)
string person_name
