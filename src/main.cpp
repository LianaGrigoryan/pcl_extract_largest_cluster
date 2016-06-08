#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "Logger.hpp"


// Typedefs
using PointType = pcl::PointXYZRGBA;
using Cloud = pcl::PointCloud <PointType>;

namespace fs = boost::filesystem;


// User defined literals
// @formatter:off
constexpr size_t operator "" _sz (unsigned long long size) { return size_t{size}; }
constexpr double operator "" _deg (long double deg) { return deg * M_PI / 180.0; }
constexpr double operator "" _deg (unsigned long long deg) { return deg * M_PI / 180.0; }
constexpr double operator "" _cm (long double cm) { return cm / 100.0; }
constexpr double operator "" _cm (unsigned long long cm) { return cm / 100.0; }
constexpr double operator "" _mm (long double mm) { return mm / 1000.0; }
constexpr double operator "" _mm (unsigned long long mm) { return mm / 1000.0; }
// @formatter:on


// Constants
constexpr auto MIN_VALID_ARGS = 3U;
constexpr auto MAX_VALID_ARGS = 9U;
constexpr auto NUM_PCD_FILES_EXPECTED = 2U;
constexpr auto NUM_PCD_DIRS_EXPECTED = 2U;
constexpr auto INPUT_DIR_ARG_POS = 1U;
constexpr auto OUTPUT_DIR_ARG_POS = 2U;
constexpr auto DEFAULT_MIN_CLUSTER_SIZE = 100U;
constexpr auto DEFAULT_MAX_CLUSTER_SIZE = 25000U;
constexpr auto DEFAULT_TOLERANCE = 2_cm;
constexpr auto DEFAULT_MAX_NUM_CLUSTERS = 20U;


auto printHelp (int argc, char ** argv) -> void {
  using pcl::console::print_error;
  using pcl::console::print_info;

  // TODO: Update this help
  print_error ("Syntax is: %s (<path-to-pcd-files> <path-to-store-output>) <options> | "
                   "(<pcd-file> <output-pcd-file>) <options> | -h | --help\n", argv[0]);
  print_info ("%s -h | --help : shows this help\n", argv[0]);
  print_info ("-min X : use a minimum of X points per cluster (default: 100)\n");
  print_info ("-max X : use a maximum of X points per cluster (default: 25000)\n");
  print_info ("-tol X : the spatial distance (in meters) between clusters (default: 0.002.\n");
}


auto getPcdFilesInPath (fs::path const & pcd_dir)
-> std::deque <fs::path> {
  auto result_set = std::deque <fs::path>{};
  for (auto const & entry : boost::make_iterator_range (fs::directory_iterator{pcd_dir})) {
    if (fs::is_regular_file (entry.status ())) {
      if (entry.path ().extension () == ".pcd") {
        result_set.emplace_back (entry);
      }
    }
  }
  return result_set;
}


auto expandTilde (std::string path_string) -> fs::path {
  if (path_string.at (0) == '~')
    path_string.replace (0, 1, getenv ("HOME"));
  return fs::path{path_string};
}


auto extractIndices (Cloud::Ptr cloud,
                     pcl::PointIndicesPtr indices,
                     bool keep_organised = false,
                     bool negative = false)
-> Cloud::Ptr {
  auto extract = pcl::ExtractIndices <pcl::PointXYZRGBA>{};
  extract.setInputCloud (cloud);
  extract.setIndices (indices);
  if (cloud->isOrganized ())
    extract.setKeepOrganized (keep_organised);
  extract.setNegative (negative);
  auto extracted_cloud = boost::make_shared <Cloud> ();
  extract.filter (*extracted_cloud);
  return extracted_cloud;
}


auto getClusters (Cloud::Ptr input_cloud,
                  unsigned min_cluster_size = DEFAULT_MIN_CLUSTER_SIZE,
                  unsigned max_cluster_size = DEFAULT_MAX_CLUSTER_SIZE,
                  double cluster_tolerance = DEFAULT_TOLERANCE,
                  unsigned max_num_clusters = DEFAULT_MAX_NUM_CLUSTERS,
                  bool keep_organised = false)
-> std::vector <Cloud::Ptr> {
  auto tree = boost::make_shared <pcl::search::KdTree <PointType>> ();
  tree->setInputCloud (input_cloud);

  auto clusters = std::vector <pcl::PointIndices> {};
  auto clusterer = pcl::EuclideanClusterExtraction <PointType> {};

  clusterer.setClusterTolerance (cluster_tolerance);
  clusterer.setMinClusterSize (min_cluster_size);
  clusterer.setMaxClusterSize (max_cluster_size);
  clusterer.setSearchMethod (tree);
  clusterer.setInputCloud (input_cloud);
  clusterer.extract (clusters);

  auto larger_cluster_lambda = [] (pcl::PointIndices a, pcl::PointIndices b) {
    return a.indices.size () >= b.indices.size ();
  };
  std::sort (clusters.begin (), clusters.end (), larger_cluster_lambda);

  auto output_cluster_clouds = std::vector <Cloud::Ptr> {};
  auto extracted_clusters = 0U;
  for (auto const & cluster : clusters) {
    auto indices_ptr = boost::make_shared <pcl::PointIndices> (cluster);
    auto output_cluster_cloud = extractIndices (input_cloud, indices_ptr, keep_organised);
    output_cluster_clouds.push_back (output_cluster_cloud);
    if (++extracted_clusters >= DEFAULT_MAX_NUM_CLUSTERS)
      break;
  }

  return output_cluster_clouds;
}


auto processLargestClusterExtraction (fs::path input_cloud_path,
                                      fs::path output_cloud_path,
                                      unsigned min_cluster_size = DEFAULT_MIN_CLUSTER_SIZE,
                                      unsigned max_cluster_size = DEFAULT_MAX_CLUSTER_SIZE,
                                      double cluster_tolerance = DEFAULT_TOLERANCE)
-> bool {
  // Load input pcd
  auto input_cloud = boost::make_shared <Cloud> ();
  if (pcl::io::loadPCDFile <pcl::PointXYZRGBA> (input_cloud_path.c_str (), *input_cloud) == -1) {
    pcl::console::print_error ("Failed to load: %s\n", input_cloud_path);
    return false;
  }

  auto output_clouds = getClusters (input_cloud,
                                    min_cluster_size,
                                    max_cluster_size,
                                    cluster_tolerance,
                                    1U);

  if (output_clouds.size () < 1_sz)
    return false;

  auto & largest_cluster_cloud = output_clouds.at (0);

  if (pcl::io::savePCDFileBinaryCompressed <pcl::PointXYZRGBA> (output_cloud_path.string (),
                                                                *largest_cluster_cloud) == -1) {
    Logger::log (Logger::ERROR, "Failed to save: %s\n", output_cloud_path);
    return false;
  }

  //  Logger::log(Logger::INFO, "Output file %s written.\n", output_cloud_path.c_str());

  return true;
}


auto getFront (std::deque <fs::path> & paths_set_deque, std::mutex & deque_mutex)
-> fs::path {
  auto next_cloud_path = fs::path{};
  std::lock_guard <std::mutex> lock {deque_mutex};
  if (!paths_set_deque.empty ()) {
    next_cloud_path = paths_set_deque.front ();
    paths_set_deque.pop_front ();
    if (paths_set_deque.size () % 10 == 0) {
      Logger::log (Logger::INFO, "%u items remaining in deque.\n", paths_set_deque.size ());
    }
  }
  return next_cloud_path;
}


auto euclideanClusteringRunner (std::deque <fs::path> & paths_deque,
                                fs::path output_dir,
                                std::mutex & deque_mutex,
                                unsigned min_cluster_size = DEFAULT_MIN_CLUSTER_SIZE,
                                unsigned max_cluster_size = DEFAULT_MAX_CLUSTER_SIZE,
                                double cluster_tolerance = DEFAULT_TOLERANCE)
-> bool {
  auto input_pcd_file = getFront (paths_deque, deque_mutex);

  while (!paths_deque.empty ()) {
    auto output_pcd_file = output_dir / input_pcd_file.filename ();
    processLargestClusterExtraction (input_pcd_file,
                                     output_pcd_file,
                                     min_cluster_size,
                                     max_cluster_size,
                                     cluster_tolerance);
    input_pcd_file = getFront (paths_deque, deque_mutex);
  }
  Logger::log (Logger::INFO, "Thread exiting.\n");

  return true;
}


auto threadedClusterSegmentation (std::deque <fs::path> input_files,
                                  fs::path output_dir,
                                  unsigned min_cluster_size = DEFAULT_MIN_CLUSTER_SIZE,
                                  unsigned max_cluster_size = DEFAULT_MAX_CLUSTER_SIZE,
                                  double cluster_tolerance = DEFAULT_TOLERANCE)
-> bool {
  auto runners = std::vector <std::shared_ptr <std::thread>> {};
  auto num_threads_to_use = std::thread::hardware_concurrency ();
  std::mutex deque_mutex;

  for (auto i = 0_sz; i < num_threads_to_use; ++i) {
    auto runner = std::make_shared <std::thread> (euclideanClusteringRunner,
                                                  std::ref (input_files),
                                                  output_dir,
                                                  std::ref (deque_mutex),
                                                  min_cluster_size,
                                                  max_cluster_size,
                                                  cluster_tolerance);
    runners.push_back (runner);
  }

  for (auto const & runner : runners)
    if (runner->joinable ())
      runner->join ();

  return true;
}


auto main (int argc, char * argv[])
-> int {
  pcl::console::print_highlight ("Tool to extract the largest cluster found in a point cloud.\n");

  auto help_flag_1 = pcl::console::find_switch (argc, argv, "-h");
  auto help_flag_2 = pcl::console::find_switch (argc, argv, "--help");

  if (help_flag_1 || help_flag_2) {
    printHelp (argc, argv);
    return -1;
  }

  if (argc > MAX_VALID_ARGS || argc < MIN_VALID_ARGS) {
    pcl::console::print_error ("Invalid number of arguments.\n");
    printHelp (argc, argv);
    return -1;
  }

  auto consumed_args = 1;

  auto min_cluster_size = DEFAULT_MIN_CLUSTER_SIZE;
  auto max_cluster_size = DEFAULT_MAX_CLUSTER_SIZE;
  auto tolerance = DEFAULT_TOLERANCE;

  if (pcl::console::parse_argument (argc, argv, "-min", min_cluster_size) != -1)
    consumed_args += 2;

  if (pcl::console::parse_argument (argc, argv, "-max", max_cluster_size) != -1)
    consumed_args += 2;

  if (pcl::console::parse_argument (argc, argv, "-tol", tolerance) != -1)
    consumed_args += 2;

  auto ss = std::stringstream {};
  ss << "Applying euclidean cluster extraction with the following settings: " << std::endl <<
      "\tMinimum cluster size = " << min_cluster_size << std::endl <<
      "\tMaximum cluster size = " << max_cluster_size << std::endl <<
      "\tCluster tolerance = " << tolerance << std::endl << std::endl;

  // Check if we are working with pcd files
  auto pcd_arg_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (pcd_arg_indices.size () == NUM_PCD_FILES_EXPECTED) {
    auto input_pcd_file = expandTilde (std::string {argv[pcd_arg_indices.at (0)]});
    auto output_pcd_file = expandTilde (std::string {argv[pcd_arg_indices.at (1)]});
    consumed_args += NUM_PCD_FILES_EXPECTED;
    if (consumed_args != argc) {
      printHelp (argc, argv);
      return -1;
    }

    // Load input pcd
    auto input_cloud = boost::make_shared <Cloud> ();
    if (pcl::io::loadPCDFile <PointType> (input_pcd_file.c_str (), *input_cloud) == -1) {
      pcl::console::print_error ("Failed to load: %s\n", input_pcd_file);
      printHelp (argc, argv);
      return -1;
    }

    // Print any settings
    pcl::console::print_info (ss.str ().c_str ());

    auto cluster_clouds = getClusters (input_cloud, min_cluster_size, max_cluster_size, tolerance);
    auto & largest_cluster_cloud = cluster_clouds.at (0);

    if (pcl::io::savePCDFileBinaryCompressed <pcl::PointXYZRGBA> (output_pcd_file.string (),
                                                                  *largest_cluster_cloud) == -1) {
      Logger::log (Logger::ERROR, "Failed to save: %s\n", output_pcd_file);
      return -1;
    }

    // TODO: Visualize clusters (?)
  } else {
    // We are working with folders
    // Validate that we have correct number of args
    if (argc - consumed_args != NUM_PCD_DIRS_EXPECTED) {
      pcl::console::print_error ("Invalid number of arguments.\n");
      printHelp (argc, argv);
      return -1;
    }

    auto input_dir = expandTilde (std::string{argv[INPUT_DIR_ARG_POS]});
    if (!fs::exists (input_dir) || !fs::is_directory (input_dir)) {
      pcl::console::print_error ("A valid input directory was not specified.\n");
      printHelp (argc, argv);
      return -1;
    }

    auto output_dir = expandTilde (std::string{argv[OUTPUT_DIR_ARG_POS]});
    if (!fs::exists (output_dir) || !fs::is_directory (output_dir)) {
      try {
        fs::create_directory (output_dir);
      } catch (fs::filesystem_error e) {
        pcl::console::print_error ("Unable to create output directory. "
                                       "Please ensure that the correct permissions are "
                                       "set for the target folder and that the path is valid.\n");
        printHelp (argc, argv);
        return -1;
      }
    }

    // We have the input and output dir now
    auto input_files = getPcdFilesInPath (input_dir);

    // Load an input pcd
    auto input_cloud = boost::make_shared <Cloud> ();
    auto mid_point = input_files.size () / 2;
    auto sample_cloud_file = input_files.at (mid_point);
    if (pcl::io::loadPCDFile <pcl::PointXYZRGBA> (sample_cloud_file.c_str (), *input_cloud) == -1) {
      pcl::console::print_error ("Failed to load: %s\n", sample_cloud_file.c_str ());
      printHelp (argc, argv);
      return -1;
    }

    // Print any settings
    pcl::console::print_info (ss.str ().c_str ());

    threadedClusterSegmentation (input_files,
                                 output_dir,
                                 min_cluster_size,
                                 max_cluster_size,
                                 tolerance);
  }
  return 0;
}