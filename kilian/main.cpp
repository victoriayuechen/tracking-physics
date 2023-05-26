#include "pointcloud_tracking.hpp"

int main()
{
    //Set the program options that will be available through the command line
    unsigned short port_self;
    unsigned short port_send;
    std::string ip_send;
    bool kinect_data = true;
    unsigned int particles = 700;
    bool kld = false;
    double variance = 0.00025;
    double downsample_grid = 0.02;
    std::string model3d = "../vid_dyn_move/frame_0.pcd";
    std::string location_result;
    std::string location_video;
    bool use_video = true;
    bool save_video = false;
    bool save_results = false;
//
//    boost::program_options::variables_map variable_map;
//    boost::program_options::options_description desc("Options for the network client occupied with tracking");
//    desc.add_options()
//            ("help,h", "Produce this help message")
//            ("server,s", boost::program_options::value<unsigned short>(& port_self)->default_value(2525),
//             "Set the port of the server, the port that this server sends its data from")
//            ("client,c", boost::program_options::value<unsigned short>(& port_send)->default_value(2323),
//             "Set the port of the client, the port this server sends its data to")
//            ("ip,i", boost::program_options::value<std::string>(& ip_send)->default_value("127.0.0.1"),
//             "Set the IP address the data is sent to")
//            ("kinect,k", boost::program_options::value<bool>(& kinect_data)->default_value(true),
//             "Indicate whether to use data provided by the connected kinect or test data")
//            ("particles,p", boost::program_options::value<unsigned int>(& particles)->default_value(1500),
//             "Set how many particles the Particle Filter should use for its filtering process")
//            ("covariance,v", boost::program_options::value<double>(& variance)->default_value(0.000225),
//             "Set the covariance in the particle sampling process (stddev^2)")
//            ("downsample,d", boost::program_options::value<double>(& downsample_grid)->default_value(0.02),
//             "Set the level of downsampling (higher value = faster but less accurate)")
//            ("model3d,m", boost::program_options::value<std::string>(& model3d)->default_value("../data/Ensenso_Blender_Positioned_OG.pcd"),
//             "Location of the 3D pointcloud model used for object tracking")
//            ("usevid,u", boost::program_options::value<bool>(& use_video)->default_value(false),
//             "Use an existing video instead of live data from the kinect")
//            ("savevid,f", boost::program_options::value<bool>(& save_video)->default_value(false),
//             "Save the current frames received by the kinect as a video")
//            // ("locvid,t", boost::program_options::value<std::string>(& location_video)->default_value("../data/vid_dyn_move"),
//            //     "Video location")
//            ("saveres,r", boost::program_options::value<bool>(& save_results)->default_value(true),
//             "Save model data")
//            ("locres,o", boost::program_options::value<std::string>(& location_result)->default_value("../data/results/dump.txt"),
//             "The location where the data of the model is stored")
//            ;
//    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), variable_map);
//    boost::program_options::notify(variable_map);
//    if (variable_map.count("help")) {
//        std::cout << desc << "\n";
//        return 0;
//    }

    //Initialize the sensor (in this case kinect) and the network
    KinectCamera* kinect_camera;
    kinect_camera = new KinectCamera;
//    network::NetworkClient* tracking_network;
//    tracking_network = new network::NetworkClient;

    //If you want to save the results of the tracking efforts open a file and append
    if(save_results)
    {
//        kinect_camera->position_tracking_time.open(location_result, std::ofstream::out | std::ofstream::app);
//        if(boost::filesystem::file_size(location_result) == 0)
//        {
//            kinect_camera->position_tracking_time << (boost::format("%f %f %f %f %f %f %f\n") % "x" % "y" % "z" % "roll" % "pitch" % "yaw" % "time").str();
//        }
    }

    kinect_camera->SetDownsampleSize(downsample_grid);

    // Begin tracing session
  //  Instrumentor::Get().BeginSession("tracking");

    //If data from the sensor (in this case kinect) is desired, initialize the kinect
    kinect_camera->SetUpKinect(model3d, particles, variance, kld, false, save_results, "../vid_dyn_move");


   // tracking_network->SetUpNetwork(*kinect_camera, port_self, port_send, ip_send, kinect_data);
   std::cout << "set up " << std::endl;
    //If a video is to be used for tracking instead of real-time data, iterate over the video frames
    if(use_video)
    {
        pcl::PointCloud<RefPointType>::Ptr cloud (new pcl::PointCloud<RefPointType>);
        while (kinect_camera->GetFrameNumber() < 50)
        {
            if(true)
            {
              //  PROFILE_SCOPE("NEW FRAME");
                kinect_camera->ss.str(std::string());
                kinect_camera->ss << "../vid_dyn_move" << "/frame_" << kinect_camera->GetFrameNumber() << ".pcd";
                pcl::io::loadPCDFile (kinect_camera->ss.str(), *cloud);
            }
            kinect_camera->CloudCallback(cloud);
            kinect_camera->savePointCloud();
            kinect_camera->IncrementFrameNumber(1);
        }
    }
        //Else wait for the Enter key to be pressed
    else
    {
        std::cin.ignore();
    }

    //If data from the sensor (in this case kinect) is desired, stop the kinect when the process is done
//    if(kinect_data)
//    {
//        kinect_camera->StopKinectListener(use_video);
//    }
//
//  //  tracking_network->StopNetwork();
//
//    //If you want to save the results of the tracking efforts, close the file used for this when all is done
//    if(save_results)
//    {
//        kinect_camera->position_tracking_time.close();
//    }


    return 0;
}

