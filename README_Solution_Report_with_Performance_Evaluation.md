# SFND : Camera Based 2D Feature Tracking 
Camera Based 2D Feature Tracking implemented using C++ as part of Mid-term project submission for Camera section of Sensor Fusion Nanodegree (Udacity).

-------------
### Solution Report : 
- ####  MP.1 Data Buffer Optimization
	- Task:  Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.
	- Implementation : The `dataBufferSize` is set to 2. If the buffer has reached its full limit, then the oldest element is removed from the buffer and the new element is added at the end. 
		```
        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        
        if(dataBuffer.size() < dataBufferSize)
        {
            dataBuffer.push_back(frame);
        }
        else
        {
            dataBuffer.erase(dataBuffer.begin());
            dataBuffer.push_back(frame);
        }
		```



- ####  MP.2 Keypoint Detection
 	- Task:  Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
	- Implementation : The detectors are implemented in the `matching2D_Student.cpp` and `MidTermProject_Camera_Student.cpp`.
	The detector selection is part of `MidTermProject_Camera_Student.cpp`. By setting the string `detectorType` the desired detector function is chosen. The actual detector implementation is part of `matching2D_Student.cpp`. The functions `detKeypointsShiTomasi(...)` and `detKeypointsHarris(...)` implement the SHITOMASI and HARRIS detectors respectively. The function `detKeypointsModern(....)` implements all the other detectors(FAST, BRISK, ORB, AKAZE, and SIFT) with internal selector logic for each detector. 
		```
        string detectorType = "HARRIS"; //set one of the detectorType here: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT
        
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray , false);
        }
        //// -> FAST, BRISK, ORB, AKAZE, SIFT
      	else if(detectorType.compare("FAST") == 0 || detectorType.compare("BRISK") == 0 || detectorType.compare("ORB") == 0  || detectorType.compare("AKAZE") == 0 || detectorType.compare("SIFT") == 0 )
        {
          	detKeypointsModern(keypoints, imgGray ,detectorType, false);
        }
       ```
 - ####  MP.3 Keypoint Removal
 	- Task : Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
 	- Implementation  : Iterate through all the detected keypoints and check if it falls withing the preciding vehicle ROI. Remove all the keypoints which are outside the ROI. 
      ```
        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
			for (auto iter = keypoints.begin(); iter != keypoints.end();) 
            {
				if (!vehicleRect.contains((*iter).pt))
                {
					iter = keypoints.erase(iter);
				}
              else
              {
                ++iter;
              }
              
			}
            cout << "Num. of Keypoints on focus area of vehicle is = "<< keypoints.size()<<endl;

        }
      ```
  - ####  MP.4 Keypoint Descriptors
  	- Task : Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
  	- Implementation : The descriptors are implemented in the `matching2D_Student.cpp` and `MidTermProject_Camera_Student.cpp`.
  	In the `MidTermProject_Camera_Student.cpp` the descriptor is initialised and the `descriptorType` is set. The `descKeypoints(....)` in  `matching2D_Student.cpp` contains the actual keypoint description implementation with selector for `descriptorType`.
    	- In  `MidTermProject_Camera_Student.cpp` : 
       ```
       cv::Mat descriptors;
       string descriptorType = "BRISK"; // BRISK,BRIEF, ORB, FREAK, AKAZE, SIFT
       descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
       ```
       - In  `matching2D_Student.cpp` : 
       ```
         void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
          {
              // select appropriate descriptor
              cv::Ptr<cv::DescriptorExtractor> extractor;
              if (descriptorType.compare("BRISK") == 0)
              {
                  int threshold = 30;        // FAST/AGAST detection threshold score.
                  int octaves = 3;           // detection octaves (use 0 to do single scale)
                  float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
                  extractor = cv::BRISK::create(threshold, octaves, patternScale);
              }
              else if (descriptorType.compare("BRIEF") == 0)
              {
                  extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
              }
              else if (descriptorType.compare("ORB") == 0)
              {
                  extractor = cv::ORB::create();
              }
              else if (descriptorType.compare("FREAK") == 0)
              {
                  extractor = cv::xfeatures2d::FREAK::create();
              }
              else if (descriptorType.compare("AKAZE") == 0)
              {
                  extractor = cv::AKAZE::create();    
              }
              else if (descriptorType.compare("SIFT") == 0)
              {
                  extractor = cv::xfeatures2d::SIFT::create();
              }
              // perform feature description
              double t = (double)cv::getTickCount();
              extractor->compute(img, keypoints, descriptors);
              t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
              cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
          }
  
 - ####  MP.5 Descriptor Matching  and MP.6 Descriptor Distance Ratio 
 	- Task : Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
  	- Implementation : The Brute Force matcher `MAT_BF` and FLANN matcher `MAT_FLANN` are implemented in the `matchDescriptors(....)` function (in `matching2D_Student.cpp`) and matcher type is selectable based on the string set in main file `MidTermProject_Camera_Student.cpp`. The matching task for the selected matcher type is implemented based on the selector type. Nearest neighbor (best match) `SEL_NN` and K-Nearest-Neighbor(KNN) `SEL_KNN` selection is implemented and is selectable based on the string set in main file `MidTermProject_Camera_Student.cpp`. For KNN , k = 2 and the matches are filtered using descriptor distance ratio test where the descriptor distance ratio threshold is 0.8
      ```
      // Find best matches for keypoints in two camera images based on several matching methods
        void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                              std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
        {
            // configure matcher
            bool crossCheck = false;
            cv::Ptr<cv::DescriptorMatcher> matcher;

            if (matcherType.compare("MAT_BF") == 0)
            {
                int normType = cv::NORM_HAMMING;
                matcher = cv::BFMatcher::create(normType, crossCheck);
                cout << "BF matching cross-check=" << crossCheck;
            }
            else if (matcherType.compare("MAT_FLANN") == 0)
            {
                if (descSource.type() != CV_32F)
                { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
                    descSource.convertTo(descSource, CV_32F);
                    descRef.convertTo(descRef, CV_32F);
                }
                matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
                cout << "FLANN matching";
            }

            // perform matching task
            if (selectorType.compare("SEL_NN") == 0)
            { // nearest neighbor (best match)

                double t = (double)cv::getTickCount();
                matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
                t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
                cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

            }
            else if (selectorType.compare("SEL_KNN") == 0)
            { // k nearest neighbors (k=2)
                const int k = 2;
                vector<vector<cv::DMatch>>  knn_matches;

                // DONE : implement k-nearest-neighbor matching
                double t = (double)cv::getTickCount();
                matcher->knnMatch(descSource, descRef, knn_matches,k);
                t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
                cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

                // DONE : filter matches using descriptor distance ratio test
                const float ratio_threshold  = 0.8f;

                for(auto it = knn_matches.begin(); it!= knn_matches.end();++it)
                {
                    if((*it)[0].distance < ratio_threshold*(*it)[1].distance)
                    {
                        matches.push_back((*it)[0]);
                    }
                }
                cout << "# keypoints removed = \t" << knn_matches.size() - matches.size() ;
                cout << "So Final (KNN) with n=" << matches.size() << " matches" <<  endl;
            }
        }
        
  
 - ####  MP.7 Performance Evaluation 1	
 	- Task : Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
  	- Evaluation : The number of keypoints are listed for each image. Only the keypoints on the preceeding vehicle are considered.  

	
  Detector |Img-1|	Img-2|Img-3	|Img-4	|Img-5	|Img-6	|Img-7	|Img-8	|Img-9	|Img-10
----|-----|-----|-------|-------|-------|-----|-------|-------|-------|------
**SHI-Tomasi**	|125|	118|	123|	120|	120|	113|	114|	123|	111|	112
**Harris**		|17	|	14|		18|		21|		26|		43|		18|		31|		26|		34
**FAST**		|149|	152|	150|	155|	149|	149|	156|	150|	138|	143
**BRISK**		|264|	282|	282|	277|	297|	279|	289|	272|	266|	254
**ORB**			|92	|	102|	106|	113|	109|	125|	130|	129|	127|	128
**AKAZE**		|166|	157|	161|	155|	163|	164|	173|	175|	177|	179
**SIFT**		|138|	132|	124|	137|	134|	140|	137|	148|	159|	137

  
 - #### MP.8 Performance Evaluation 2	
 	- Task : Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
  	- Evaluation :
  	The detailed evaluation is documented in the spreadsheet : [Peformance_Evaluation_Data_MP7_MP8_MP9.xlsx](https://github.com/hegde056/SFND_2D_Feature_Matching/blob/master/Peformance_Evaluation_Data_MP7_MP8_MP9.xlsx) . Below is the table with the averages over all 10 images. 

  Detector | Descriptor |Keypoints on preceding vehicle|Time : keypoint detection (ms)|Time : descriptor extraction(ms)|Time :  matching (ms)|Final Matches  
----|------|------|------|-------|------|-----
SHITOMASI|BRISK|117.9|17.47|2.20|0.34|85.22
SHITOMASI|BRIEF|117.9|16.41|1.19|0.29|104.89
SHITOMASI|ORB|117.9|16.33|0.97|0.30|100.89
SHITOMASI|FREAK|117.9|13.04|39.40|0.31|85.33
SHITOMASI|SIFT|117.9|15.40|17.40|2.42|103.22
HARRIS| BRISK|24.8|18.57|1.17|0.12|15.78
HARRIS| BRIEF|24.8|18.31|0.88|0.10|19.22
HARRIS| ORB|24.8|18.66|0.83|0.19|18.00
HARRIS| FREAK|24.8|15.54|38.88|0.12|16.00
HARRIS| SIFT|24.8|19.23|20.43|0.49|18.11
FAST|BRISK|149.1|1.03|2.85|0.93|99.89
FAST|BRIEF|149.1|0.92|1.27|0.41|122.11
FAST|ORB|149.1|0.89|1.15|0.38|119.00
FAST|FREAK|149.1|0.96|42.59|0.41|97.56
FAST|SIFT|149.1|0.96|27.69|2.90|116.56
BRISK|BRISK|276.2|40.71|3.33|1.07|174.44
BRISK|BRIEF|276.2|40.40|1.32|1.03|189.33
BRISK|ORB|276.2|40.25|4.47|0.99|168.22
BRISK|FREAK|276.2|40.42|43.22|0.98|169.33
BRISK|SIFT|276.2|43.90|50.88|5.52|184.33
ORB|BRISK|116.1|8.51|1.60|0.31|83.44
ORB|BRIEF|116.1|7.95|0.88|0.31|60.56
ORB|ORB|116.1|7.69|5.00|0.29|84.78
ORB|FREAK|116.1|8.07|43.68|0.21|46.67
ORB|SIFT|116.1|8.58|52.72|2.14|84.89
AKAZE|BRISK|167|86.26|2.63|0.50|135.00
AKAZE|BRIEF|167|82.41|1.52|0.49|140.67
AKAZE|ORB|167|83.08|4.08|0.55|131.33
AKAZE|FREAK|167|77.43|42.63|0.46|131.89
AKAZE|AKAZE|167|82.35|72.56|0.53|139.89
AKAZE|SIFT|167|84.67|30.29|3.33|141.89
SIFT|BRISK|138.6|114.58|1.83|0.42|65.78
SIFT|BRIEF|138.6|129.37|0.94|0.37|78.00
SIFT|FREAK|138.6|129.38|41.08|0.36|65.89
SIFT|SIFT|138.6|109.03|82.39|2.68|89.56
 
 -  #### MP.9 Performance Evaluation 3	
 	- Task : Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
  	- Evaluation : The detailed evaluation is documented in the spreadsheet : [Peformance_Evaluation_Data_MP7_MP8_MP9.xlsx](https://github.com/hegde056/SFND_2D_Feature_Matching/blob/master/Peformance_Evaluation_Data_MP7_MP8_MP9.xlsx) . The above table lists the average time for keypoint detection and descriptor extraction over all 10 images.
	- TOP3 detector / descriptor combinations are chosen based on the project rubric crieteria of the time it takes for keypoint detection and descriptor extraction. 
	
Detector | Descriptor |Keypoints on preceding vehicle|Time : keypoint detection (ms)|Time : descriptor extraction(ms)|Time :  matching (ms)|Final Matches  
----|------|------|------|-------|------|-----
FAST|ORB|149.1|0.89|1.15|0.38|119.00
FAST|BRIEF|149.1|0.92|1.27|0.41|122.11
FAST|BRISK|149.1|1.03|2.85|0.93|99.89

-------------
	



