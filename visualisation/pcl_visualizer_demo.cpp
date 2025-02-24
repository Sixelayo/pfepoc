/* \author Geoffrey Biggs */

#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree_search.h>

using namespace std::chrono_literals;

#define DEBUG(x) std::cout << x;

typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Mytree;

/*
be carefull when using this as benchmark. Especially if there are std::cout call in between start and split
*/
namespace timer{
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> split_time;
    std::chrono::duration<double, std::milli> elapsed;

    inline void start(){
        start_time = std::chrono::high_resolution_clock::now();
    }
    inline void splitLOG(){
        split_time = std::chrono::high_resolution_clock::now();
        elapsed = split_time - start_time;
        std::cout << "(" << elapsed.count() << " ms)";
    }
    inline void endLOG(){
        split_time = std::chrono::high_resolution_clock::now();
        elapsed = split_time - start_time;
        std::cout << "(" << elapsed.count() << " ms)";
        start();
    }

}


// --------------
// -----Help-----
// --------------
void printUsage (){
    std::cout << "\n\nUsage: "<<" [options]\n\n"
                << "First arg must be a mode (view, sample, compare).\n"
                << "Please be gentle, the parsing is very rudimental so if things don't works check that you've given valid arguments\n"
                << "Aslo please don't look too closely at the implementaiton the CLI arguments are parsed like 3 times I was to lazy to do think correctly"
                << "-------------------------------------------\n"
                << "\t-h                        this help\n"
                << "\t-seed <int>               force a random seed\n"
                << "mode :\n"
                << "\t-view\n"
                << "\t\t-file <path>\n"
                << "\t-sample\n"
                << "\t\t-mode <mode> <args...>  Depends on mode :\n"
                << "\t\t\trand <float: proportion>                          Randomly keep a given % of points\n"
                << "\t\t\tmd <float: dist>                                  Minimum distance, no acceleration\n"
                << "\t\t\tmdwo <float: dist,float: octree_dist>             Minimum distance using octree\n"
                << "\t\t-file <path>               output save path\n"
                << "\t\t-o <path>               output save path\n"
                << "\t\t-binary                 save output as binary (default : ASCII)\n"
                << "\t\t-prev                   if present, load a viewer with sampled cloud (no preview if asbent)\n"
                << "\t-compare\n"
                << "\t\t-oc1 <float>            octree param\n"
                << "\t\t-oc2 <float>            octree param\n"
                << "\t\t-file1 <path>           origin point cloud path\n"
                << "\t\t-file2 <path>           compared point cloud path\n"
                << "\t\t-o <path>               output save path\n"
                << "\n\n";
    exit(0);
}



pcl::visualization::PCLVisualizer::Ptr initViewer (pcXYZ::ConstPtr cloud)
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

namespace smp{
    enum Mode {
        RANDOM = 0,
        MIN_DIST = 1,
        MIN_DIST_OCTREE = 2
    };
    struct Arg_rand{
        float proportion;
    };
    struct Arg_md{
        float min_dist;
    };
    struct Arg_mdwo{
        float min_dist;
        float octree_dist;
    };
    //Accept points with a random probability
    void sample_random(pcXYZ::Ptr src_cloud, pcXYZ::Ptr new_cloud, const Arg_rand& args){
        timer::start();
        float proba = args.proportion;
        for(pcl::PointXYZ point : src_cloud->points){
            float random_value = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
            if(random_value < proba) new_cloud->push_back(point);
        }

        timer::endLOG();
        DEBUG(" Finished sampling, " << new_cloud->size() << " points kept\n")
    }
    void sample_min_dist(pcXYZ::Ptr src_cloud, pcXYZ new_cloud, const Arg_md args){
        
    }
    void sample_mind_dist_octree(pcXYZ::Ptr src_cloud, pcXYZ new_cloud, const Arg_mdwo args){
        
    }

}


void main_view(int argc, char** argv){
    DEBUG("mode : view\n");
    
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

    DEBUG("\t Loading : " << PATH << " ...\n");

    // Create a Point Cloud object
    pcXYZ::Ptr cloud_ptr(new pcXYZ);

    // Load the PCD file
    timer::start();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(PATH, *cloud_ptr) == -1) {
        PCL_ERROR("Couldn't read the PCD file.\n");
        exit(-1);
    }

    timer::endLOG();
    DEBUG(" Loaded " << cloud_ptr->width * cloud_ptr->height << " data points ");

    pcl::visualization::PCLVisualizer::Ptr viewer;
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
    DEBUG("mode : sample\n");

    std::string PATH_SRC;
    std::string PATH_OUT;

    smp::Mode sampling_method;
    smp::Arg_rand arg_rand;
    smp::Arg_md arg_md;
    smp::Arg_mdwo arg_mdwo;
    bool preview = false;
    bool binary = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-file") {
            if(i+1 == argc) exit(-1);
            PATH_SRC = argv[i+1];
        } else if (arg == "-o") {
            if(i+1 == argc) exit(-1);
            PATH_OUT = argv[i+1];
        }else if (arg == "-prev") {
            preview = true;
        }else if (arg == "-binary") {
            binary = true;
        }

        else if (arg == "-mode") {
            if(i+1 == argc) exit(-1);
            std::string arg2 = argv[i+1];
            if(arg2 == "rand"){
                if(i+2 >= argc) exit(-1);
                sampling_method = smp::RANDOM;
                arg_rand.proportion = atof(argv[i+2]);
            } else if(arg2 == "md"){
                if(i+2 >= argc) exit(-1);
                sampling_method = smp::MIN_DIST;
                arg_md.min_dist = atof(argv[i+2]);
            } else if(arg2 == "mdwo"){
                if(i+3 >= argc) exit(-1);
                sampling_method = smp::MIN_DIST_OCTREE;
                arg_mdwo.min_dist = atof(argv[i+2]);
                arg_mdwo.octree_dist = atof(argv[i+3]);
            }
            else{
                std::cout << "wrong -mode usage";
                exit(-1);
            }
        }
    }

    //create point cloud object
    pcXYZ::Ptr src_cloud_ptr(new pcXYZ);
    pcXYZ::Ptr new_cloud_ptr(new pcXYZ);

    // Load the PCD file
    timer::start();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(PATH_SRC, *src_cloud_ptr) == -1) {
        PCL_ERROR("Couldn't read the PCD file.\n");
        exit(-1);
    }

    timer::endLOG();
    DEBUG(" Loaded " << src_cloud_ptr->width * src_cloud_ptr->height << " data points\n");


    

    //construct out_cloud_ptr
    switch (sampling_method){
        case smp::RANDOM: smp::sample_random(src_cloud_ptr, new_cloud_ptr, arg_rand); break;
        case smp::MIN_DIST:
            break;
        case smp::MIN_DIST_OCTREE:
            break;
    }
    //clear source in memoyr bc not needed anymore TODO maybe ?
    src_cloud_ptr->clear(); 


    
    //save output
    timer::start();
    pcl::io::savePCDFile(PATH_OUT, *new_cloud_ptr, binary);
    timer::endLOG();
    DEBUG(" saved file at " << PATH_OUT << "\n");

    if(preview){
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = initViewer(new_cloud_ptr);
        
        //--------------------
        // -----Main loop-----
        //--------------------
        while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            std::this_thread::sleep_for(100ms);
        }
    }

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

