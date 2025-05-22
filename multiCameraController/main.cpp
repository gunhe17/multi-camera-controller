#include "config/config.hpp"
#include "capture/capture.hpp"
#include <iostream>


/**
 *  Main
 */

int main(int argc, char* argv[]) {
    // Initialize
    Config config = parse_args(argc, argv);
    Capture capture(config);

    // run
    capture.start();
    
    return 0;
}