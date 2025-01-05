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

//#define PCD_FPATH "O:\\pfe\\example_pcd\\data-master\\terrain\\CSite2_orig-utm.pcd"
std::string PCD_FPATH;
float sample_arg; //either a proba or a min dist




// --------------
// -----Help-----
// --------------
void printUsage (const char* progName){
    std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
                << "Options:\n"
                << "-------------------------------------------\n"
                << "-h           this help\n"
                << "-s           Simple visualisation example\n"
                << "-l <path>    load from a pcd file\n"
                << "-r <float>      keep random proportion btw 0-1\n"
                << "-mdna <float>   minimum distance (no acceleration)\n"
                << "-mdwa <float>   minimum distance with acceleration\n"
                << "\n\n";
}


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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






pcl::visualization::PCLVisualizer::Ptr viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

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



void sample_random(pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_loaded_cloud_ptr, float proba){
    for(pcl::PointXYZ point : loaded_cloud->points){
        //accept with random proba
        float random_value = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
        if(random_value < proba) sampled_loaded_cloud_ptr->points.push_back(point);
    }
}

float dist_pointToCloud(pcl::PointXYZ point, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    float min_dist = 999999; //start with absurdly hight value
    for(pcl::PointXYZ other_point : cloud->points){
         float dist = std::sqrt(std::pow(point.x - other_point.x, 2) +
                               std::pow(point.y - other_point.y, 2) +
                               std::pow(point.z - other_point.z, 2));
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}



//without accelerationd data structure
void sample_mindist(pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_loaded_cloud_ptr, float threshold){
    int cnt =0; //todo torm
    for(pcl::PointXYZ point : loaded_cloud->points){
        cnt++; //todo torm
        if(cnt%10000==0) std::cout << cnt << " ";
        // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
        if (dist_pointToCloud(point, sampled_loaded_cloud_ptr) >= threshold) {
            sampled_loaded_cloud_ptr->points.push_back(point);
        }
    }
}



bool dist_pointToCloudOCTREE(pcl::PointXYZ point, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree, float mdist) {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    return !(octree.radiusSearch(point, mdist, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0);
}

//with acceleration
void sample_mindistOCTREE(pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_loaded_cloud_ptr, float threshold){
    int cnt =0; //todo torm

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(2*threshold); //threshold times 2 because why not ?
    octree.setInputCloud(sampled_loaded_cloud_ptr);

    for(pcl::PointXYZ point : loaded_cloud->points){
        cnt++; //todo torm
        if(cnt%10000==0) std::cout << cnt << " ";
        // Accept if the minimum distance to any point in the sampled cloud is greater than the threshold
        if (dist_pointToCloudOCTREE(point, octree, threshold)) {
            //sampled_loaded_cloud_ptr->points.push_back(point);
            octree.addPointToCloud(point, sampled_loaded_cloud_ptr);
        }
    }
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv){
    //initialisation (seed ...)
    srand(time(NULL));


    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument (argc, argv, "-h") >= 0)
    {
        printUsage (argv[0]);
        return 0;
    }
    bool simple(false), viewports(false), interaction_customization(false),
        loadFile(false), re_sample(false), sample_rand(false), sample_mdna(false), sample_mdwa(false);
    {//parsing ...
        if (pcl::console::find_argument (argc, argv, "-s") >= 0){
            simple = true;
            std::cout << "Simple visualisation example\n";
        }
        else if (pcl::console::find_argument(argc, argv, "-l") >= 0) {
            loadFile = true;
            int index = pcl::console::find_argument(argc, argv, "-l") + 1;
            if (index < argc) {
                PCD_FPATH = argv[index];
                std::cout << "File to load: " << PCD_FPATH << "\n";
                //precise sampling method, -r / -mdna / -mdwa
                if (pcl::console::find_argument(argc, argv, "-r") >= 0) {
                    sample_rand = true; re_sample = true;
                    int index = pcl::console::find_argument(argc, argv, "-r") + 1;
                    if (index < argc) {
                        sample_arg = atof(argv[index]);
                        std::cout << "proportion keep : " << sample_arg << "\n";
                    } else {
                        std::cerr << "Error: No float given provided after -d.\n";
                        exit(-1);
                    }
                }
                else if (pcl::console::find_argument(argc, argv, "-mdna") >= 0) {
                    sample_mdna = true; re_sample = true;
                    int index = pcl::console::find_argument(argc, argv, "-mdna") + 1;
                    if (index < argc) {
                        sample_arg = atof(argv[index]);
                        std::cout << "minimum distance : " << sample_arg << "\n";
                    } else {
                        std::cerr << "Error: No float given provided after -mdna.\n";
                        exit(-1);
                    }
                }
                else if (pcl::console::find_argument(argc, argv, "-mdwa") >= 0) {
                    sample_mdwa = true; re_sample = true;
                    int index = pcl::console::find_argument(argc, argv, "-mdwa") + 1;
                    if (index < argc) {
                        sample_arg = atof(argv[index]);
                        std::cout << "minimum distance : " << sample_arg << "\n";
                    } else {
                        std::cerr << "Error: No float given provided after -mdwa.\n";
                        exit(-1);
                    }
                }
            } else {
                std::cerr << "Error: No file path provided after -l.\n";
                exit(-1);
            }
        }
        else{
            printUsage (argv[0]);
            return 0;
        }
    }

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Generating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    std::uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05){
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
        pcl::PointXYZ basic_point;
        basic_point.x = 0.5 * std::cos (pcl::deg2rad(angle));
        basic_point.y = sinf (pcl::deg2rad(angle));
        basic_point.z = z;
        basic_cloud_ptr->points.push_back(basic_point);

        pcl::PointXYZRGB point;
        point.x = basic_point.x;
        point.y = basic_point.y;
        point.z = basic_point.z;
        point.r = r;
        point.g = g;
        point.b = b;
        point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
        r -= 12;
        g += 12;
        }
        else
        {
        g -= 12;
        b += 12;
        }
    }
    basic_cloud_ptr->width = basic_cloud_ptr->size ();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = point_cloud_ptr->size ();
    point_cloud_ptr->height = 1;


    // Create a Point Cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(loadFile){
        std::cout << "Loading point clouds from file\n\n";
        // Load the PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(PCD_FPATH, *loaded_cloud) == -1) {
            PCL_ERROR("Couldn't read the PCD file.\n");
            return -1;
        }

        std::cout << "Loaded " << loaded_cloud->width * loaded_cloud->height
                << " data points from " << PCD_FPATH << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_loaded_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    //compute new point cloud after sampling ...
    auto start = std::chrono::high_resolution_clock::now();
    if(sample_rand) sample_random(loaded_cloud, sampled_loaded_cloud_ptr, sample_arg);
    if(sample_mdna) sample_mindist(loaded_cloud, sampled_loaded_cloud_ptr, sample_arg);
    if(sample_mdwa) sample_mindistOCTREE(loaded_cloud, sampled_loaded_cloud_ptr, sample_arg);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() << " ms" << std::endl;
    std::cout << "Points in sampled cloud :\t" << sampled_loaded_cloud_ptr->size() << std::endl;
    loaded_cloud->clear();
    
    

    // ----------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.05-----
    // ----------------------------------------------------------------
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (point_cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);

    // ---------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.1-----
    // ---------------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.1);
    ne.compute (*cloud_normals2);

    pcl::visualization::PCLVisualizer::Ptr viewer;
    {//depending on args
        if (simple){
            viewer = simpleVis(basic_cloud_ptr);
        }
        else if (viewports){
            viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
        }
        else if (loadFile){
            if(!re_sample) viewer = simpleVis(loaded_cloud);
            else viewer = simpleVis(sampled_loaded_cloud_ptr);
        }
    }

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
}