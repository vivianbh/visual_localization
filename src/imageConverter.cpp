#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

using namespace std;

class ImageConverter
{
private:
    ros::NodeHandle nh;
    ros::Subscriber img_sub;
    int imgNum = 0;
    vector<string> fileName_prf = {"camera0_", "camera1_left_", "camera1_right_"};
    
public:
    ImageConverter() {
        cout << "Start convert to image." << endl;
        img_sub = nh.subscribe("/camera_image/detected", 10, &ImageConverter::imageCb, this);
    }

    ~ImageConverter() {}

    void imageCb(const sensor_msgs::Image::ConstPtr& img) {
        cv_bridge::CvImagePtr cv_ptr;
        string number;

        try {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cout << "image hight: " << img->height << endl;
        cout << "image width: " << img->width << endl;
        cout << "image size: " << img->data.size() << endl;

        imageCounter();

        if (imgNum < 10) {
            number = "00" + to_string(imgNum);
        } else if (imgNum >= 10 && imgNum < 100) {
            number = "0" + to_string(imgNum);
        } else {
            number = to_string(imgNum);
        }

        string filename = fileName_prf[0] + number + ".png";
        cv::imwrite(filename, cv_ptr->image);
    }

    void imageCounter() {
        imgNum += 1;
        cout << "image number: " << imgNum << endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();

    return 0;
}