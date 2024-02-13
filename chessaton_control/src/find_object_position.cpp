#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "chessaton_interfaces/srv/box_positions.hpp"

std::shared_ptr<rclcpp::Node> node;
bool tmpFlag = false; 

// Topics
static const std::string IMAGE_TOPIC = "/my_camera/image_raw";
static const std::string POINT_CLOUD2_TOPIC = "/my_camera/points";

 // Publisher
//  ros::Publisher pub;
// rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

// tf2_ros::Buffer tf_buffer;
std::unique_ptr<tf2_ros::Buffer> tf_buffer;

const std::string from_frame = "camera_link_optical";
const std::string to_frame = "world";

cv::Point2f box_centroid;
cv::Point2f target_centroid;

geometry_msgs::msg::Point box_position_base_frame;
geometry_msgs::msg::Point target_position_base_frame;


cv::Mat apply_cv_algorithms(cv::Mat camera_image) {
    // convert the image to grayscale format
    cv::Mat img_gray;
    cv::cvtColor(camera_image, img_gray, cv::COLOR_BGR2GRAY);
    cv::Mat canny_output;
    // numbers for min and max are low because of the lighting in simulation
    cv::Canny(img_gray,canny_output,8,30);

    // show result of canny
    // cv::namedWindow( "view1");
    // cv::imshow("view1", canny_output);
    // cv::waitKey(3);

    return canny_output;
}

std::vector<cv::Point2f> extract_centroids(cv::Mat canny_output) {
    // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    cv::RNG rng(12345);
    cv::Mat drawingt = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
    std::vector<cv::Point> tmp;
    std::vector<std::vector<cv::Point>> tmpContours;
    std::vector<std::vector<cv::Point>> bigBoxContours;
    std::vector<std::vector<cv::Point>> smallBoxContours;
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f> result;

    for( size_t i = 0; i< contours.size(); i++ ) {
        // finding valid areas
        cv::approxPolyDP(contours[i], tmp, 0.008*cv::arcLength(contours[i],true), true);
        tmpContours.push_back(tmp);
    }
    for( size_t i = 0; i< contours.size(); i++ ) {
        // we are trying to find cubes or squares therefore we limit the contours 
        if ((tmpContours[i].size() < 12) && (tmpContours[i].size() > 3)) {
            boundRect[i] = boundingRect( tmpContours[i] );
            // simple and naive way to detect the big square table and the small blue square object according to their area  
            if(boundRect[i].area() > 2000) {
                bigBoxContours.push_back(tmpContours[i]);
            } else if ((boundRect[i].area() > 80) && (boundRect[i].area() < 900)) {
                smallBoxContours.push_back(tmpContours[i]);
            }
            // uncomment these if you want to see all the areas detected for
            // each cube (cause a cube could have multiple squares or multi contour shapes in it)

            cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
            cv::drawContours(drawingt, tmpContours, (int)i, color, 2, cv::LINE_8, hierarchy, 0);
        }

    }

    // cv::namedWindow( "view3");
    // cv::imshow( "view2", drawingt );
    // cv::waitKey(3);

    int sc = smallBoxContours.size();
    // get the moments
    std::vector<cv::Moments> smallMu(smallBoxContours.size());
    for( int i = 0; i<smallBoxContours.size(); i++ )
    { smallMu[i] = cv::moments( smallBoxContours[i], false ); }
    // get the centroid of figures.
    std::vector<cv::Point2f> smallCentroids(smallBoxContours.size());
    for( int i = 0; i<smallBoxContours.size(); i++) {
        if (smallMu[i].m00 != 0) {
            float centroid_x = smallMu[i].m10/smallMu[i].m00;
            float centroid_y = smallMu[i].m01/smallMu[i].m00;
            smallCentroids[i] = cv::Point2f(centroid_x, centroid_y);
        } else {
            sc--;
        }
    }

    cv::Point2f zero(0.0f, 0.0f);
    cv::Point2f sum  = std::accumulate(smallCentroids.begin(), smallCentroids.end(), zero);
    cv::Point2f small_mean_point(sum.x / sc, sum.y / sc);
    result.push_back(small_mean_point);

    int bc = bigBoxContours.size();
    // get the moments
    std::vector<cv::Moments> bigMu(bigBoxContours.size());
    for( int i = 0; i<bigBoxContours.size(); i++ )
    { bigMu[i] = cv::moments( bigBoxContours[i], false ); }
    // get the centroid of figures.
    std::vector<cv::Point2f> bigCentroids(bigBoxContours.size());
    for( int i = 0; i<bigBoxContours.size(); i++) {
        // ignoring the contours that are disjointed and require another way to find their centroids. 
        if (bigMu[i].m00 != 0) {
            float centroid_x = bigMu[i].m10/bigMu[i].m00;
            float centroid_y = bigMu[i].m01/bigMu[i].m00;
            bigCentroids[i] = cv::Point2f(centroid_x, centroid_y);
        } else {
            bc--;
        }
    }

    sum  = std::accumulate(bigCentroids.begin(), bigCentroids.end(), zero);
    cv::Point2f big_mean_point(sum.x / bc, sum.y / bc);
    result.push_back(big_mean_point);


    // draw findal contours
    cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));

    for( int i = 0; i<bigBoxContours.size(); i++ ) {
        cv::Scalar color = cv::Scalar(0,151,100); // B G R values
        cv::drawContours(drawing, bigBoxContours, (int)i, color, 2, 8, hierarchy, 0, cv::Point());
        cv::circle( drawing, bigCentroids[i], 4, color, -1, 8, 0 );
    }
    for( int i = 0; i<smallBoxContours.size(); i++ ) {
        if (smallMu[i].m00 != 0) {
            cv::Scalar color = cv::Scalar(167,151,0); // B G R values
            cv::drawContours(drawing, smallBoxContours, (int)i, color, 2, 8, hierarchy, 0, cv::Point());
            cv::circle( drawing, smallCentroids[i], 4, color, -1, 8, 0 );
        }
    }

    // show final result.
    // cv::namedWindow( "view2");
    // cv::imshow( "view3", drawing );
    // cv::waitKey(3);

    return result;
}

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
        auto logger = rclcpp::get_logger("my_subscriber");
        RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
 
    }

    cv::Mat canny_output = apply_cv_algorithms(cv_ptr->image);

    std::vector<cv::Point2f> centroids = extract_centroids(canny_output);

    box_centroid = centroids[0];
    target_centroid = centroids[1];
    tmpFlag = true;
}

geometry_msgs::msg::Point pixel_to_3d_point(const sensor_msgs::msg::PointCloud2::SharedPtr pCloud, const int u, const int v) {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*pCloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pCloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pCloud, "z");
    geometry_msgs::msg::Point p;

    p.x = *(iter_x+(640*v+u));
    p.y = *(iter_y+(640*v+u));
    p.z = *(iter_z+(640*v+u));

    return p;
}

geometry_msgs::msg::Point transform_between_frames(geometry_msgs::msg::Point p, const std::string from_frame, const std::string to_frame) {
    geometry_msgs::msg::PoseStamped input_pose_stamped;
    input_pose_stamped.pose.position = p;
    input_pose_stamped.header.frame_id = from_frame;
    input_pose_stamped.header.stamp = node->get_clock()->now();

    geometry_msgs::msg::PoseStamped output_pose_stamped = tf_buffer->transform(input_pose_stamped, to_frame);
    return output_pose_stamped.pose.position;
}

void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pCloud) {
    // sensor_msgs::msg::PointCloud2 tmpCloud = pCloud;
    if (tmpFlag) {
        geometry_msgs::msg::Point box_position_camera_frame;
        box_position_camera_frame = pixel_to_3d_point(pCloud, box_centroid.x, box_centroid.y);

        geometry_msgs::msg::Point target_position_camera_frame;
        target_position_camera_frame = pixel_to_3d_point(pCloud, target_centroid.x, target_centroid.y);

        box_position_base_frame = transform_between_frames(box_position_camera_frame, from_frame, to_frame);
        target_position_base_frame = transform_between_frames(target_position_camera_frame, from_frame, to_frame);

        // std::cout << "3d box position base frame: x " << box_position_base_frame.x << " y " << box_position_base_frame.y << " z " << box_position_base_frame.z << std::endl;
        // std::cout << "3d target position base frame: x " << target_position_base_frame.x << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z << std::endl;
    }
}

// service call response
bool get_positions(std::shared_ptr<chessaton_interfaces::srv::BoxPositions::Request>  req,
                    std::shared_ptr<chessaton_interfaces::srv::BoxPositions::Response> res) {
        res->box_position = box_position_base_frame;
        res->target_position = target_position_base_frame;

        // TODO: process of the images should be removed from callback functions of subscriptions
        // so that the program doesn't slow down. for now an ugly solution has implemented which 
        // will kill the node after responding to request satisfying our one time use of the service
        // but if we were going to make it a general process we have to change it to structure priorly mentioned.
        node.reset();
        // Shutdown ROS
        rclcpp::shutdown();

        // return true;
}

int main(int argc, char **argv) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>(
       "opencv_services",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // we use sim_time for simulation in gazebo
    // rclcpp::Parameter sim_time_param("use_sim_time", true);
    // node->set_parameter(sim_time_param);
 
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listner;
    tf_listner.reset(new tf2_ros::TransformListener(*tf_buffer));

    // Used to publish and subscribe to images
    image_transport::ImageTransport it(node);
    image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1, image_callback);
    auto point_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(POINT_CLOUD2_TOPIC, 1, point_cloud_callback);
   
    // ros::ServiceServer service = nh.advertiseService("box_and_target_position",  get_box_and_target_position);
    rclcpp::Service<chessaton_interfaces::srv::BoxPositions>::SharedPtr service = 
        node->create_service<chessaton_interfaces::srv::BoxPositions>("box_positions", &get_positions);

    rclcpp::spin(node);
    // Close down OpenCV
    // cv::destroyWindow("view1");
    // cv::destroyWindow("view2");
    // cv::destroyWindow("view3");
    return 0;
}
