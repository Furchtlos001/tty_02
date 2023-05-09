#include"../../include/ros_detection_tracking/project.h"


namespace DetectandTract{

    void projector::initParams()
    {
        std::string pkg_loc = ros::package::getPath("ros_detection_tracking");
        std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");
        infile >> i_params.camera_topic;
        infile >> i_params.lidar_topic;
        //std::cout<<i_params.camera_topic<<std::endl;
        double_t camtocam[12];
        double_t cameraIn[16];//内参
        double_t RT[16];// 外参
        for (int i = 0; i < 16; i++){
            infile >> camtocam[i];
        }
        cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);//cameratocamera params

        for (int i = 0; i < 12; i++){
            infile >> cameraIn[i];
        }
        cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);//cameraIn params

        for (int i = 0; i < 16; i++){
            infile >> RT[i];
        }
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);//lidar to camera params
        std::cout<<i_params.RT<<std::endl;
    }


    void projector::projection_callback(const sensor_msgs::Image::ConstPtr &img, 
                                    const sensor_msgs::PointCloud2::ConstPtr &pc)
        {
            // 定义一个指向CvImage的指针，用于存储转换后的图像数据
            cv_bridge::CvImagePtr cv_ptr;
            // 将ROS中的图像消息转换为OpenCV中的图像数据类型，存储到cv_ptr中。如果转换失败，则返回
            try {
                cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
            }
            catch (cv_bridge::Exception &e) {
                return;
            }
            cv::Mat raw_img = cv_ptr->image;// 获取转换后的图像数据
            // 将ROS中的点云消息转换为PCL中的点云数据类型
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*pc, *cloud);
            
            // 复制原始图像数据raw_img，得到visImg和overlay，后续会在visImg上绘制点云数据，而overlay是用于绘制点云数据的遮罩
            cv::Mat visImg = raw_img.clone();
            cv::Mat overlay = visImg.clone();
            std::cout<<"get pc and image data"<<std::endl;
            
            cv::Mat X(4,1,cv::DataType<double>::type);
            cv::Mat Y(3,1,cv::DataType<double>::type);
            for(pcl::PointCloud<pcl::PointXYZI>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++){
                // 保留下0<x<maxX,-maxY<y<maxY,minZ<z
                if(it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ ) 
                {
                    continue;
                }
                
                X.at<double>(0,0) = it->x;
                X.at<double>(1,0) = it->y;
                X.at<double>(2,0) = it->z;
                X.at<double>(3,0) = 1;

                Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X;  //tranform the point to the camera coordinate

                cv::Point pt;
                pt.x = Y.at<double>(0,0) / Y.at<double>(0,2);
                pt.y = Y.at<double>(1,0) / Y.at<double>(0,2);

                float val = it->x;
                float maxVal = 20.0;
                // 通过颜色进行距离区分
                int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
                cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);
            }


            // Publish the image projection
            ros::Time time = ros::Time::now();// 获取当前时间作为图像的时间戳。
            cv_ptr->encoding = "bgr8";
            cv_ptr->header.stamp = time;
            cv_ptr->header.frame_id = "/camera_main";
            cv_ptr->image = overlay;// 将处理后的图像赋值给cv_ptr的image字段
            // 将cv_ptr转换成ROS中的sensor_msgs::Image类型，并通过image_publisher发布到指定的topic上
            image_publisher.publish(cv_ptr->toImageMsg());
            std::cout<<"project picture is published!"<<std::endl;
        }

    
    void projector::matrix_to_transfrom(Eigen::MatrixXf &matrix, tf::Transform & trans)
    {
        
    }

    projector::projector() 
    {
        ros::NodeHandle nh("~");// 创建一个ROS节点句柄
        initParams();// 初始化节点参数

        ros::Publisher project_img_pub;// ROS中用于发布消息的类，后面貌似没用到

        // 将相机图像和激光雷达点云的消息同步到一个回调函数 projection_callback 中，进行投影操作
        // Subscriber订阅话题，其构造函数的参数分别是ROS节点句柄、话题名称和缓存队列的大小
        message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, i_params.camera_topic, 5);
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, i_params.lidar_topic, 5);
        // 使用typedef定义了一个同步策略MySyncPolicy，使用message_filters::sync_policies::ApproximateTime类型
        // 该类型的含义是以近似的时间戳进行同步
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        // 使用message_filters::Synchronizer将两个Subscriber注册到同步器中，并指定同步策略和缓存队列的大小，
        // 同时也指定了回调函数projection_callback，并使用boost::bind绑定了projection_callback函数和当前对象this，
        // 并使用_1和_2指定回调函数接收的参数类型为sensor_msgs::Image和sensor_msgs::PointCloud2
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, pcl_sub);
        sync.registerCallback(boost::bind(&projector::projection_callback, this, _1, _2));
        // 这样，当相机和激光雷达的数据到达时，message_filters库会根据其时间戳进行同步，并将同步后的数据传递给回调函数projection_callback进行投影等处理。

        // ImageTransport是ROS中用于传输图像消息的类，创建时需要传入一个ros::NodeHandle对象，它会自动地在ROS网络中创建与该NodeHandle相关联的图像传输通道
        image_transport::ImageTransport imageTransport(nh);
        // advertise()方法可以用于创建一个图像发布者，投影完成后，该节点将结果图像发布到 "/project_pc_image" 话题上，20是指消息队列的大小
        image_publisher = imageTransport.advertise("/project_pc_image", 20);

        // ros::Publisher和image_transport::ImageTransport都是用于发布消息，但是它们的实现方式不同。
        // ros::Publisher发布的消息是ROS消息，可以包含各种类型的数据，但是传输效率相对较低；
        // 而image_transport::ImageTransport发布的消息是图像数据，使用专门的图像传输协议，传输效率更高。

        ros::spin();// 开始ROS节点主循环
    }



}

int main(int argc, char **argv){
    ros::init(argc, argv, "project_pc_to_image");
    DetectandTract::projector projector;
    return 0;
}