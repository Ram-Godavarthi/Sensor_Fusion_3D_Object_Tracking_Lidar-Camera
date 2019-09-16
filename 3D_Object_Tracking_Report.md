# SFND 3D Object Tracking

**FP.1 Match 3D Objects
Criteria: Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (the boxID property). Matches must be the ones with the highest number of keypoint correspondences.**

- Solution: To accomplish this task, I have taken help of "multimap" function as suggestd in the classroom to store the bouding box ID's. The keypoints assoiciated with the boxes are noted down and the best match is found. 
code : Line 261-307: camFusion_student.cpp 

**FP.2 Compute Lidar-based TTC
Criteria: Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.**

- Solution: From the classroom understanding, Vehicle with Lidar mounted will take the closest point in from it to measure the distance between itself and the preceding vehicles. But when we look into the Lidar points on the preceding vehicle, W ecan observe soem unwanted erroneous points close to the point of interest. When we try to calculate the closest points, error might occur. To avoid this, I have chosen a range in y direction and taken 150 closest points on the preceding vehicle and calculated the mean of all the points. This led to the reduction in the error. 

**FP.3 Associate Keypoint Correspondences with Bounding Boxes
Criteria: Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.**

- Solution: A looping concept is implemented to achieve this. First, all the keypoints within the box are found. Then the Euclidean --distance measure is applied and mean is found. At the end, again a loop is created to remove the matches that are far from the mean -value calculated.

**FP.4 Compute Camera-based TTC
Criteria: Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.**

- Solution: TTC = From classroom understanding, We now how height H of the preceding vehicle can be mapped into the image plane using perspective projection. It is obvious that there is a geometric relation between h, H, d and the focal length f. We could use the distance between all keypoints on the vehicle relative to each other to compute a robust estimate of the height ratio in TTC equation. The ratio of all relative distances between each other can be used to compute a reliable TTC estimate by replacing the height ratios with the mean of all distance ratios. 
Like the lidar TTC estimation, this function uses the median distance ratio to avoid the impact of outliers. Unfortunately this approach is still vulnerable to wild miscalculations (-inf, NaN, etc.) if there are too many mismatched keypoints. 
 
 - TTC = -(1.0 / frameRate) / (1 - meanDistanceRatio);

**Part II: Performance Evaluation**

**FP.5 Performance Evaluation 1
Criteria: Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.**

- Solution: I had got few hiccups in the Lidar TTC but later I removed them by this: 
From the equations below, the shortest distance to preceding car from previous data frame, might have been influenced by some point cloud outliers, resulting in shorter distance than the actual tailgate. Initially I had used 30 closest Lidar points to calculate the mean of closest points inorder to remove utliers. But later I had increased it to 100 then 140. As I increase this number, the TTC time has decreasedby almost 10 seconds compered to previous one. 

- TTC = currentMeanDistance * (1.0 / frameRate) / (previousMeanDistance - currentMeanDistance);


**FP.6 Performance Evaluation 2
Criteria: Run several detector/descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

- Solution: I had taken the top 3 detector -descriptor combinations from the mid term project to accomplish the above tasks.

        DETECTOR/DESCRIPTOR 	    NUMBER OF KEYPOINTS 	     TIME
        FAST+BRIEF 	                1099 keypoints             1.771 ms
        FAST+ORB 	                1071 keypoints 	            1.922 ms
        FAST+BRISK 	                899 keypoints 	            3.045 ms

- From the below table, One can observe that the CameraTTC are way off than the Lidar points in 90% of the frames except for few frames. Out of 3 combinations, FAST-BRISK performs better than other two. 

**Below table shows the TTC with Lidar and Camera of three different descriptors **

    Detector   Descriptor   LidarTTC  CameraTTC
    FAST		BRISK		0.00s	0.00s
    FAST		BRISK		13.18s	12.30s
    FAST		BRISK		14.52s	12.35s
    FAST		BRISK		23.75s	16.62s
    FAST		BRISK		14.98s	12.89s
    FAST		BRISK		11.56s	#NAME?
    FAST		BRISK		12.40s	13.04s
    FAST		BRISK		18.39s	12.04s
    FAST		BRISK		20.40s	11.41s
    FAST		BRISK		13.29s	11.87s
    FAST		BRISK		16.09s	13.35s
    FAST		BRISK		11.39s	12.95s
    FAST		BRISK		10.81s	12.12s
    FAST		BRISK		9.61s	12.78s
    FAST		BRISK		11.20s	11.61s
    FAST		BRISK		8.33s	11.41s
    FAST		BRISK		10.30s	12.26s
    FAST		BRISK		13.78s	9.29s
    FAST		BRISK		8.90s	11.86s
                        
    FAST		BRIEF		0.00s	0.00s
    FAST		BRIEF		13.18s	11.18s
    FAST		BRIEF		14.52s	13.01s
    FAST		BRIEF		23.75s	14.82s
    FAST		BRIEF		14.98s	13.67s
    FAST		BRIEF		11.56s	#NAME?
    FAST		BRIEF		12.40s	41.78s
    FAST		BRIEF		18.39s	12.76s
    FAST		BRIEF		20.40s	12.77s
    FAST		BRIEF		13.29s	13.92s
    FAST		BRIEF		16.09s	16.21s
    FAST		BRIEF		11.39s	13.64s
    FAST		BRIEF		10.81s	12.98s
    FAST		BRIEF		9.61s	13.15s
    FAST		BRIEF		11.20s	11.70s
    FAST		BRIEF		8.33s	12.61s
    FAST		BRIEF		10.30s	13.01s
    FAST		BRIEF		13.78s	11.26s
    FAST		BRIEF		8.90s	13.79s
                        
    FAST		ORB		0.00s	0.00s
    FAST		ORB		13.18s	12.20s
    FAST		ORB		14.52s	12.93s
    FAST		ORB		23.75s	16.63s
    FAST		ORB		14.98s	14.07s
    FAST		ORB		11.56s	#NAME?
    FAST		ORB		12.40s	55.98s
    FAST		ORB		18.39s	12.39s
    FAST		ORB		20.40s	12.19s
    FAST		ORB		13.29s	12.87s
    FAST		ORB		16.09s	16.23s
    FAST		ORB		11.39s	14.14s
    FAST		ORB		10.81s	12.98s
    FAST		ORB		9.61s	13.56s
    FAST		ORB		11.20s	11.30s
    FAST		ORB		8.33s	10.71s
    FAST		ORB		10.30s	11.92s
    FAST		ORB		13.78s	11.95s
    FAST		ORB		8.90s	13.78s

