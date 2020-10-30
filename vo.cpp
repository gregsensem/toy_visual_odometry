#include "opencv2/opencv.hpp"
#include "feature_detection.cpp"
#include "absolute_scale.cpp"

enum featureDetector{FAST,GFTT,ORB};

int main()
{
    const char* input = "/home/peter/Documents/data/KITTI/image_0/%06d.png";
    double f = 718.8560;
    cv::Point2d c(607.1928, 185.2157);
    bool use_5pt = false;
    int min_inlier_num = 100;
    int numFrame = 1;

    // Open a file to write camera trajectory
    FILE* camera_traj = fopen("vo_epipolar.xyz", "wt");
    if (camera_traj == NULL) return -1;

    // Open a video and get the initial image
    cv::VideoCapture video;
    if (!video.open(input)) return -1;

    cv::Mat gray_prev;
    video >> gray_prev;
    if (gray_prev.empty())
    {
        video.release();
        return -1;
    }
    if (gray_prev.channels() > 1) cv::cvtColor(gray_prev, gray_prev, cv::COLOR_RGB2GRAY);

    // Run and record monocular visual odometry
    cv::Mat camera_pose = cv::Mat::eye(4, 4, CV_64F);

    cv::Mat traj = cv::Mat::zeros(2000,2000,CV_8UC3);

    //choose the feature detector
    featureDetector choice = FAST;

    while (true)
    {
        // Grab an image from the video
        cv::Mat image, gray;
        video >> image;
        if (image.empty()) break;
        if (image.channels() > 1) cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
        else                      gray = image.clone();

        // Extract optical flow
        std::vector<cv::Point2f> point_prev, point;
        // cv::goodFeaturesToTrack(gray_prev, point_prev, 2000, 0.01, 10);
        // FastFeatureDetection(gray_prev,point_prev);

        switch(choice)
        {
            case FAST:
                FastFeatureDetection(gray_prev,point_prev);
                break;
            case GFTT:
                GoodFeatureDetection(gray_prev,point_prev);
                break;
        }
        
        std::vector<uchar> m_status;
        cv::Mat err;
        cv::calcOpticalFlowPyrLK(gray_prev, gray, point_prev, point, m_status, err);
        gray_prev = gray;

        // Calculate relative pose
        cv::Mat E, inlier_mask;
        if (use_5pt)
        {
            E = cv::findEssentialMat(point_prev, point, f, c, cv::RANSAC, 0.99, 1, inlier_mask);
        }
        else
        {
            cv::Mat F = cv::findFundamentalMat(point_prev, point, cv::FM_RANSAC, 1, 0.99, inlier_mask);
            cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, c.x, 0, f, c.y, 0, 0, 1);
            E = K.t() * F * K;
        }
        cv::Mat R, t;
        int inlier_num = cv::recoverPose(E, point_prev, point, R, t, f, c, inlier_mask);

        //get absolute scale for translation
        double scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

        // Accumulate relative pose if result is reliable
        if ((inlier_num > min_inlier_num) && (scale>0.1))
        {
            t = scale*t;
            cv::Mat T = cv::Mat::eye(4, 4, R.type());
            T(cv::Rect(0, 0, 3, 3)) = R * 1.0;
            T.col(3).rowRange(0, 3) = t * 1.0;
            camera_pose = camera_pose * T.inv();
        }

        // Show the image and write camera pose 
        if (image.channels() < 3) cv::cvtColor(image, image, cv::COLOR_GRAY2RGB);
        for (int i = 0; i < point_prev.size(); i++)
        {
            if (inlier_mask.at<uchar>(i) > 0) cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 0, 255));
            else cv::line(image, point_prev[i], point[i], cv::Vec3b(0, 127, 0));
        }

        //show trajectory
        int x = camera_pose.at<double>(0, 3)+1000;
        int y = -camera_pose.at<double>(2, 3)+1000;
        cv::circle(traj,cv::Point(x,y),1,CV_RGB(255,0,0),2);
        cv::rectangle(traj,cv::Point(10,30),cv::Point(550,50),CV_RGB(0,0,0),CV_FILLED);

        cv::String info = cv::format("Inliers: %d (%d%%),  XYZ: [%.3f, %.3f, %.3f]", inlier_num, 100 * inlier_num / point.size(), camera_pose.at<double>(0, 3), camera_pose.at<double>(1, 3), camera_pose.at<double>(2, 3));
        cv::putText(image, info, cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 3, cv::Vec3b(0, 0, 255),3);

        cv::imshow("trajectory",traj);
        cv::imshow("vo", image);
        fprintf(camera_traj, "%.6f %.6f %.6f\n", camera_pose.at<double>(0, 3), camera_pose.at<double>(1, 3), camera_pose.at<double>(2, 3));
        if (cv::waitKey(1) == 27) break; // 'ESC' key: Exit
    
        numFrame++;
    }

    video.release();
    fclose(camera_traj);
    return 0;
}
