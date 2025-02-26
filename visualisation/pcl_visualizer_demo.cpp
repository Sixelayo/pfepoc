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
typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;
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
                << "Please be gentle, the parsing is very rudimental so if things don't works check that you've given valid arguments. All arguments are mandatory except []\n"
                << "Aslo please don't look too closely at the implementaiton the CLI arguments are parsed like 3 times I was to lazy to do think correctly"
                << "-------------------------------------------\n"
                << "\t-h                        this help (also work with any non valid arg)\n"
                << "\t-seed <int>               force a random seed\n"
                << "mode :\n"
                << "\t-view\n"
                << "\t\t-file <path>\n"
                << "\t\t[-rgb]                  if the viewing rgb files\n"
                << "\t-sample\n"
                << "\t\t-mode <mode> <args...>  Depends on mode :\n"
                << "\t\t\trand <float: proportion>                          Randomly keep a given % of points\n"
                << "\t\t\tmd <float: dist>                                  Minimum distance, no acceleration\n"
                << "\t\t\tmdwo <float: dist,float: octree_resolution>       Minimum distance using octree\n"
                << "\t\t-file <path>            output save path\n"
                << "\t\t[-save <path>]          output save path\n"
                << "\t\t[-binary]               save output as binary (default : ASCII)\n"
                << "\t\t[-prev]                 if present, load a viewer with sampled cloud (no preview if asbent)\n"
                << "\t-compare\n"
                << "\t\t-file_src <path>        origin point cloud path\n"
                << "\t\t-file_comp <path>       compared point cloud path\n"
                << "\t\t-octree_res <float>     octree resolution\n"
                << "\t\t-threshold <float>      minimum distance requiered for point to be considered different\n"
                << "\t\t[-only_src]             include points present only in source point cloud int output\n"
                << "\t\t[-only_comp]            include points present only in compared point cloud in output\n"
                << "\t\t[-all]                  equivalent to -only_src -only_comp\n"
                << "\t\t[-save <path>]          output save path\n"
                << "\t\t[-binary]               save output as binary (default : ASCII)\n"
                << "\t\t[-prev]                 if present, load a viewer with sampled cloud (no preview if asbent)\n"
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
pcl::visualization::PCLVisualizer::Ptr initViewerRGB (pcXYZRGB::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
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

namespace util{
    void loadXYZ(const std::string& path, pcXYZ::Ptr cloud){
        DEBUG("Loading : " << path << " ...\n");
        // Load the PCD file
        timer::start();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) {
            PCL_ERROR("Couldn't read the PCD file.\n");
            exit(-1);
        }

        timer::endLOG();
        DEBUG(" Loaded " << cloud->width * cloud->height << " data points\n");
    }

    void loadXYZRGB(const std::string& path, pcXYZRGB::Ptr cloud){
        DEBUG("Loading : " << path << " ...\n");
        // Load the PCD file
        timer::start();
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) == -1) {
            PCL_ERROR("Couldn't read the PCD file.\n");
            exit(-1);
        }

        timer::endLOG();
        DEBUG(" Loaded " << cloud->width * cloud->height << " data points\n");
    }


    void saveXYZ(const std::string& path, pcXYZ::Ptr cloud, bool binary){
        DEBUG("Saving " << cloud->width * cloud->height << " data points to " <<path << "\n");

        timer::start();
        pcl::io::savePCDFile(path, *cloud, binary);
        timer::endLOG();
        DEBUG(" Done")
    }

    void saveXYZRGB(const std::string& path, pcXYZRGB::Ptr cloud, bool binary){
        DEBUG("Saving " << cloud->width * cloud->height << " data points to " <<path << "\n");

        timer::start();
        pcl::io::savePCDFile(path, *cloud, binary);
        timer::endLOG();
        DEBUG(" Done")
    }


    Mytree loadOctree(pcXYZ::Ptr cloud, float resolution){
        DEBUG("Loading " << cloud->size() << " data points to octree of resolution " << resolution <<"\n");
        timer::start();
        Mytree octree (resolution);
        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud();
        timer::endLOG();
        DEBUG(" done\n")
        return octree; //RVO and std::move FTW !
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
        float octree_res;
    };
    //Accept points with a random probability
    void sample_random(pcXYZ::Ptr src_cloud, pcXYZ::Ptr new_cloud, const Arg_rand& args){
        timer::start();
        float proba = args.proportion;

        DEBUG("Starting random sampling. Keeping " << proba*100 << "% of points\n");
        for(pcl::PointXYZ point : src_cloud->points){
            float random_value = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
            if(random_value < proba) new_cloud->push_back(point);
        }

        timer::endLOG();
        DEBUG(" Finished sampling, " << new_cloud->size() << " points kept\n")
    }

    //returns distance of a point to a cloud
    float dist_pointToCloud(const pcl::PointXYZ& point, pcXYZ::Ptr cloud){
        float min_dist = 999999; //start with absurdly hight value
        for(pcl::PointXYZ other_point : cloud->points){
             float dist = std::sqrt(std::pow(point.x - other_point.x, 2) +
                                   std::pow(point.y - other_point.y, 2) +
                                   std::pow(point.z - other_point.z, 2));
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }

    void sample_min_dist(pcXYZ::Ptr src_cloud, pcXYZ::Ptr new_cloud, const Arg_md args){
        float threshold = args.min_dist;

        DEBUG("Starting minimum distance sampling. Points are at least " << threshold << "AU appart\n"); //note that AU obviously refers to arbitrary unit and not Astronomical Unit
        timer::start();
        for(pcl::PointXYZ point : src_cloud->points){
            // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
            if (dist_pointToCloud(point, new_cloud) >= threshold) {
                new_cloud->push_back(point);
            }
        }
        timer::endLOG();
        DEBUG(" Finished sampling, " << new_cloud->size() << " points kept\n")
    }

    //true if there's no point within mdsit of point in octree
    bool accept_pointToCloudOCTREE(const pcl::PointXYZ& point, const Mytree& octree, float mdist) {
        //TODO maybe an optimisation : do not create vector on stack each time ? pass as ref in argument and clear them at the end 
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        return octree.radiusSearch(point, mdist, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0;
    }

    void sample_mind_dist_octree(pcXYZ::Ptr src_cloud, pcXYZ::Ptr new_cloud, const Arg_mdwo args){
        float threshold = args.min_dist;
        float resolution = args.octree_res;

        DEBUG("Starting minimum distance sampling with octree. \n\tPoints are at least " << threshold << "AU appart\n\tOctree resolution : "<< resolution <<" UA\n");
        timer::start();

        //step 1 create octree
        Mytree octree(resolution);
        octree.setInputCloud(new_cloud);

        //step 2 loop over points
        for(pcl::PointXYZ point : src_cloud->points){
            // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
            if (accept_pointToCloudOCTREE(point, octree, threshold)) {
                octree.addPointToCloud(point, new_cloud);
            }
        }

        timer::endLOG();
        DEBUG(" Finished sampling, " << new_cloud->size() << " points kept\n")
    }

} //end namespace smp

namespace cmp{
    void compare(float threshold, pcXYZ::Ptr cloud1, Mytree& octree1, pcXYZ::Ptr cloud2, Mytree& octree2, pcXYZRGB::Ptr out, bool only_src, bool only_comp){
        if(only_src){
            DEBUG("comparing cloud 1 points to octree 2\n");
            timer::start();
            for(pcl::PointXYZ point : cloud1->points){
                // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
                if (smp::accept_pointToCloudOCTREE(point, octree2, threshold))
                    out->emplace_back(point.x, point.y, point.z, 255,0,0); //red : poit only in cloud 1
                else
                    out->emplace_back(point.x, point.y, point.z, 0,255,0); //green : point in common
            }
            timer::endLOG();
            DEBUG("points only in source : " << out->size()<<"\n");
        }
        
        if(only_comp){
            DEBUG("comparing cloud 2 points to octree 1\n");
            timer::start();
            for(pcl::PointXYZ point : cloud2->points){
                // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
                if (smp::accept_pointToCloudOCTREE(point, octree1, threshold))
                    out->emplace_back(point.x, point.y, point.z, 0,0,255); //blue : point only in c loud 2
                else
                    out->emplace_back(point.x, point.y, point.z, 0,255,0); //green : point in common
            }
            timer::endLOG();
            DEBUG("points only in compared : " << out->size()<<"\n");
        }
        

    }
} //end namespace cmp


void main_view(int argc, char** argv){
    DEBUG("mode : view\n");
    
    std::string PATH;
    bool rgb = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-file") {
            if(i+1 == argc) exit(-1);
            PATH = argv[i+1];
        } else if (arg == "-rgb") {
            rgb = true;
        }
    }

    //technically irrelevant variable living in scope but will do for now
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcXYZ::Ptr cloud_ptr(new pcXYZ);
    pcXYZRGB::Ptr cloud_ptr_rgb(new pcXYZRGB);
    
    // Create a Point Cloud object and load it
    if(!rgb){
        util::loadXYZ(PATH, cloud_ptr);
        viewer = initViewer(cloud_ptr);
    } else{
        util::loadXYZRGB(PATH, cloud_ptr_rgb);
        viewer = initViewerRGB(cloud_ptr_rgb);
    }
    
    
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
        } else if (arg == "-save") {
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
                arg_mdwo.octree_res = atof(argv[i+3]);
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


    util::loadXYZ(PATH_SRC, src_cloud_ptr);    

    //construct out_cloud_ptr
    switch (sampling_method){
        case smp::RANDOM: smp::sample_random(src_cloud_ptr, new_cloud_ptr, arg_rand); break;
        case smp::MIN_DIST: smp::sample_min_dist(src_cloud_ptr, new_cloud_ptr, arg_md); break;
        case smp::MIN_DIST_OCTREE: smp::sample_mind_dist_octree(src_cloud_ptr, new_cloud_ptr, arg_mdwo); break;
    }
    //clear source in memoyr bc not needed anymore
    src_cloud_ptr->clear(); 

    //save output
    if(!PATH_OUT.empty())
        util::saveXYZ(PATH_OUT, new_cloud_ptr, binary);

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
    DEBUG("mode : compare\n");

    std::string PATH_SRC;
    std::string PATH_COMP;
    std::string PATH_OUT;
    bool preview = false;
    bool binary = false;
    float resolution; //no incidence on result, only compute time
    float threshold; //related to sample args. Can make result vari

    //output filter
    bool only_src = false;
    bool only_comp = false;

    //parse arg
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-file_src") {
            if(i+1 == argc) exit(-1);
            PATH_SRC = argv[i+1];
        }else if (arg == "-file_comp") {
            if(i+1 == argc) exit(-1);
            PATH_COMP = argv[i+1];
        }else if (arg == "-octree_res") {
            if(i+1 == argc) exit(-1);
            resolution = atof(argv[i+1]);
        }else if (arg == "-threshold") {
            if(i+1 == argc) exit(-1);
            threshold = atof(argv[i+1]);
        }else if (arg == "-only_src") {
            only_src = true;
        }else if (arg == "-only_comp") {
            only_comp = true;
        }else if (arg == "-all") {
            only_src = true;
            only_comp = true;
        }else if (arg == "-save") {
            if(i+1 == argc) exit(-1);
            PATH_OUT = argv[i+1];
        }else if (arg == "-prev") {
            preview = true;
        }else if (arg == "-binary") {
            binary = true;
        }
    }

    //create point cloud object
    pcXYZ::Ptr src_cloud_ptr(new pcXYZ);
    pcXYZ::Ptr comp_cloud_ptr(new pcXYZ);
    pcXYZRGB::Ptr new_cloud_ptr(new pcXYZRGB); //todo needs to be XYZRGB, and rewrite viewing


    //if too long consider serializing octree
    util::loadXYZ(PATH_SRC, src_cloud_ptr); 
    Mytree src_octree = util::loadOctree(src_cloud_ptr, resolution);
    
    util::loadXYZ(PATH_COMP, comp_cloud_ptr); 
    Mytree comp_octree = util::loadOctree(comp_cloud_ptr, resolution);

    //algo de comparaison
    cmp::compare(threshold, src_cloud_ptr, src_octree, comp_cloud_ptr, comp_octree, new_cloud_ptr, only_src, only_comp);

    //save output TODO RGB
    if(!PATH_OUT.empty())
        util::saveXYZRGB(PATH_OUT, new_cloud_ptr, binary);
    
    if(preview){
        pcl::visualization::PCLVisualizer::Ptr viewer;
        viewer = initViewerRGB(new_cloud_ptr);
        
        //--------------------
        // -----Main loop-----
        //--------------------
        while (!viewer->wasStopped ()){
            viewer->spinOnce (100);
            std::this_thread::sleep_for(100ms);
        }
    }
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

