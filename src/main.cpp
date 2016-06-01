#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <iostream>

using namespace std;


auto printHelp(int argc, char **argv) -> void {
    using pcl::console::print_error;
    using pcl::console::print_info;

    print_error("Syntax is: %s [-z] [-d 0.015] (<path-to-pcd-files> <path-to-store-output>) | "
                    "(<pcd-file> <output-pcd-file> <plane-coords-yml>)| -h | --help\n", argv[0]);
    print_info("%s -h | --help : shows this help\n", argv[0]);
    print_info("-d xx : Distance xx in metres from surface below which all points "
                   "will be discarded.\n");
    print_info("-z : Get all points below the planar surface.\n");
}

int main() {
    cout << "Hello, World!" << endl;
    return 0;
}