/* \author Geoffrey Biggs */

#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/octree/octree_search.h>

using namespace std::chrono_literals;

#define DEBUG(x) std::cout << "DEBUG: " << x << std::endl;


typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Mytree;


// --------------
// -----Help-----
// --------------
void printUsage (){
    std::cout << "\n\nUsage: "<<" [options]\n\n"
                << "First arg must be a mode (view, sample, compare):\n"
                << "-------------------------------------------\n"
                << "\t-h                        this help\n"
                << "\t-seed <int>               force a random seed\n"
                << "mode :\n"
                << "\t-view\n"
                << "\t\t-file <path>\n"
                << "\t-sample\n"
                << "\t\t-mode <mode>            r, mdna, mdwa with min dist\n"
                << "\t\t-args <float...>        mode specific args\n"
                << "\t\t-o <path>               output save path\n"
                << "\t-compare\n"
                << "\t\t-oc1 <float>            octree param\n"
                << "\t\t-oc2 <float>            octree param\n"
                << "\t\t-file1 <path>           origin point cloud path\n"
                << "\t\t-file2 <path>           compared point cloud path\n"
                << "\t\t-o <path>               output save path\n"
                << "\n\n";
    exit(0);
}



pcl::visualization::PCLVisualizer::Ptr initViewer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0.2, 0.4, 0.2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();
  return (viewer);
}



unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}




void main_view(int argc, char** argv){
    DEBUG("mode : view");
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::string PATH;


    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-file") {
            if(i+1 == argc) exit(-1);
            PATH = argv[i+1];
        } else if (arg == "-foo") {
            if(i+1 == argc) exit(-1);
        }
    }

    DEBUG("\t Loading : " << PATH << " ...");


    // Create a Point Cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(PATH, *cloud_ptr) == -1) {
        PCL_ERROR("Couldn't read the PCD file.\n");
        exit(-1);
    }

    DEBUG("Loaded " << cloud_ptr->width * cloud_ptr->height << " data points");

    viewer = initViewer(cloud_ptr);
    
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }


}

void main_sample(int argc, char** argv){
    DEBUG("mode : sample");
    return;
}

void main_compare(int argc, char** argv){
    DEBUG("mode : compare");
    return;
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv){
    //global var
    unsigned int seed = static_cast<unsigned int>(std::time(nullptr));

    if(argc == 1) printUsage();
    
    //redundant parsing but eh (parse both here and after in submain)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-seed") {
            if(i+1 == argc) return -1;
            seed = atoi(argv[i+1]);
        } else if (arg == "-foo") {
            if(i+1 == argc) return -1;
        }
    }
    std::srand(seed);
    std::cout << "seed initialized : " << seed << std::endl;
        
    
    std::string arg = argv[1];
         if(arg == "-view")      main_view(argc, argv);        
    else if(arg == "-sample")    main_sample(argc, argv);        
    else if(arg == "-compare")   main_compare(argc, argv);        
    else printUsage();
}

