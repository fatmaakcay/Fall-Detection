# Fall-Detection

Our prototype implementation operated under the assumption that combining data from multiple sensors would produce a more accurate depiction of the context. Thus we measured distance data using both static ultrasonic and rotating LIDAR-lite sensors. Our project is the first instance of using any form of LIDAR for fall detection. Linear distance data expresses the distance between the sensor and the nearest object without identifying that object in any way. This property allowed our system to detect the presence and height of a person without being able to recreate their image, which allowed us to preserve user privacy in their
bathtub. As we collected the data, we applied support vector machines (SVM), a type of machine learning, to contiguous sections of data in real time in order to determine if a fall has occurred.