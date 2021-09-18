/**
	* Apple Detection Program
	*
	* Sarah Dinh
	*/
	
	#include "opencv2/opencv.hpp"
	#include "opencv2/imgproc.hpp"
	#include "opencv2/highgui.hpp"
	#include <iostream>
	#include <vector>

	using namespace cv;
	using namespace std;
	
	Mat red_mask(Mat im);
	Mat contours(Mat im);
	Mat glare(Mat im);
	Mat ori;

	int main( int argc, char** argv )
	{
	// USE CAMERA TRY
		cv::VideoCapture camera(0);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    // this will contain the image from the webcam
    cv::Mat frame;
        
    // capture the next frame from the webcam
    camera >> frame;
    
    // display the frame until you press a key
    while (1) {
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }
	// USE CAMERA TRY END

	Mat im;
	// Read image
	//im = imread( argv[1]/*, IMREAD_GRAYSCALE*/ );
	resize(im, im, Size(400, 375));
	ori = im;

	// Reduce glare in image
	Mat reduced_glare = glare(im);

	// Find red parts of image
	Mat red_filt;
	red_filt = red_mask(reduced_glare);	

	// Find contours of red filtered image
	Mat cont_draw;
	cont_draw = contours(red_filt);

	// Show results
	Mat final_output;
	final_output = im + cont_draw;
	imshow("original", im);
	//imshow("all functions", final_output);
	//imshow("glare reduced", reduced_glare);
	//imshow("red", red_filt);
	imshow("contours", cont_draw);


	waitKey(0);
	return 0;
	}

Mat glare(Mat im) {
	Mat contoured, gray;
	//contoured = contours(im);
	cvtColor(im, gray, COLOR_BGR2GRAY);

	Mat blurred;
	GaussianBlur(gray, blurred, Size(3,3), 0,0);
	//imshow("gray blur", blurred);

	Mat thresh_img;
	threshold(gray, thresh_img, 190, 255, THRESH_BINARY);
	// any pixels above ... are set to white (255) and the rest to black

	imshow("thresh_img", thresh_img);

	erode(thresh_img, thresh_img, Mat(), Point(-1,-1), 2);
	dilate(thresh_img, thresh_img, Mat(), Point(-1,-1), 2);
	imshow("dilation/erosion", thresh_img);

	Mat result;
	inpaint(im, thresh_img, result, 20, INPAINT_NS);

	return result;
}

// Function to apply red masks
Mat red_mask(Mat im){
	Mat blur, hsv;
	GaussianBlur(im, blur, Size(7,7), 0,0);
	cvtColor(blur, hsv, COLOR_BGR2HSV);
	Mat mask1, mask2, mask3, mask4,red_filt;
	
	// Color ranges for red masks
	inRange(hsv, Scalar(0,120,70), Scalar(10, 255, 255), mask1);
	inRange(hsv, Scalar(170,120,70), Scalar(180, 255, 255), mask2);
	inRange(hsv, Scalar(175,50,20), Scalar(180, 255, 255), mask3);
	inRange(hsv, Scalar(0,50,200), Scalar(5, 255, 255), mask4);

	mask1 = mask1 + mask2 + mask3 + mask4; 

	bitwise_and(im, im, red_filt, mask1);
	return red_filt;
}

Mat contours(Mat im) {
	Mat imgray, canny_output;
	int thresh = 100;
	RNG rng(12345);
	cvtColor(im, imgray, COLOR_BGR2GRAY);

	// Use canny edge detection to find contours of apples
	Canny(im, canny_output, thresh, thresh*2);
	Mat bilateral_im, edge;
	bilateralFilter(im, bilateral_im, 5, 175, 175);	
	Canny(bilateral_im, edge, 75, 200); 

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());

	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
	Mat filt = im;

	// Draw detected contours
	for (size_t i =0; i < contours.size(); i++) {
		Scalar color = Scalar(255,0,0);
		drawContours (drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0);
		//fillPoly(drawing, contours, color);
	}

	Mat dst;
	im.copyTo(dst);
	vector<vector<Point> > apples;
	vector<Point> contour;
	int apple_count = 0; 

	// Find apples based on size of contours
	for (size_t i = 0; i < contours.size(); i++) {
		approxPolyDP(contours[i], contour, arcLength(contours[i], true)*0.01, true);
		if (arcLength(contours[i], true) > 200 && contours.size() > 10 && contourArea(contours[i]) > 60) {
			apples.push_back(contours[i]);
		}

		// Detect and label apples with rectangles
		double area = contourArea(contours[i]);
		Rect r = boundingRect(contours[i]);
		int radius = r.width/2;
		
		/*// Determine if shapes detected are round for object differentiation
		if (abs(1-((double)r.width/r.height)) <= 0.2 && abs(1-(area/(CV_PI*(radius*radius)))) <=0.2) {
			//std::cout << "circle found" << std::endl;	
		} else {
			//std::cout << "no circle" << std::endl;	
		}*/
	}

	// Remove smaller contours (not apples) and fill in big contour shape
	polylines(filt, apples, true, Scalar(0,255,0), 2, FILLED);
	for (size_t i =0; i < apples.size(); i++) {
		boundRect[i] = boundingRect(apples[i]);
		Scalar color = Scalar(255,0,0);
		rectangle(ori, boundRect[i].tl(), boundRect[i].br(), color, 2);
		apple_count ++;
	}
	std::cout << "Apple count is: " << apple_count << std::endl;

	imshow ("filtered contour", ori);
	return filt;
}
