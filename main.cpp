
#include "librealsense2/rs.hpp"
#include <algorithm>
#include "librealsense2/rsutil.h"
#include "time.h"
#include <omp.h>
#include <librealsense2/hpp/rs_export.hpp> // Include RealSense Cross Platform API
#include <opencv2/core/mat.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
int main(int argc, char * argv[]) try
{
    clock_t start, stop;
    rs2::pointcloud a ;
    rs2::context ctx;
    auto list = ctx.query_devices();
    if (list.size() == 0)
    throw std::runtime_error("No device detected. Is it plugged in?");
    rs2::pipeline pipe;

    pipe.start();
    std::cout << "Connected!" << std::endl;
    cv::Mat cloud;
    float planarPoint3d[3];
    float pixel[2];

    while (true) // Application still alive?
        {
            clock_t start = clock();
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            // Get the depth frame's dimensions
            int width = depth.get_width();
            int height = depth.get_height();
            int numVertices = width*height;
            std::cout << width << " x " << height << " = " << numVertices << std::endl;
            cloud = cv::Mat(numVertices, 3, CV_32FC1);
            auto inrist = rs2::video_stream_profile(depth.get_profile()).get_intrinsics();

            int index = 0;

            //#pragma omp parallel for

            for(int dy=0; dy<height; ++dy)
            {
                for(int dx=0; dx<width; ++dx)
                {
                    pixel[0]=dx;
                    pixel[1]=dy;
                    float pixel_distance_in_meters = depth.get_distance(dx,dy);
                    rs2_deproject_pixel_to_point(planarPoint3d, &inrist, pixel, pixel_distance_in_meters);
                    if(planarPoint3d[2]>0.00001&&planarPoint3d[2]<0.6){
                        //cloud.push_back()
                        memcpy(cloud.ptr<float>(index),planarPoint3d,sizeof (planarPoint3d));
                        //std::cout << *planarPoint3d << " "<< *(planarPoint3d+1) << " " << *(planarPoint3d+2) << std::endl;
                        index++;
                    }
                }
            }
            cloud.resize(index);
            clock_t stop = clock();


            std::cout << "Time to get point cloud: " <<(float)(stop-start)*1000/CLOCKS_PER_SEC << " ms" <<  std::endl;
            //cv::ppf_match_3d::loadPLYSimple("test1.ply", 1).copyTo(cloud);
            //std::cout << cloud;
            start = clock();
            cv::Mat Cloud;
            double viewpoint[3] = {0, 0, 0};
            cv::ppf_match_3d::computeNormalsPC3d(cloud,Cloud,6,false,viewpoint);
            stop = clock();
            std::cout << "Time to compute normal: " <<(float)(stop-start)*1000/CLOCKS_PER_SEC << " ms" <<  std::endl;
            cv::ppf_match_3d::writePLY(cloud, "exported_scene.ply");
            cv::ppf_match_3d::writePLY(Cloud, "exported_scene_normal.ply");
            //a.map_to(color);
            a.calculate(depth).export_to_ply("rs_exported_scene.ply",color);
            //points.export_to_ply("test.ply",color);
            pipe.stop();
            return EXIT_SUCCESS;
        }



}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
