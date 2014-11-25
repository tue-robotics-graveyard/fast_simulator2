#include "fast_simulator2/simulator.h"

#include <tue/config/configuration.h>

#include <iostream>

#include <fast_simulator2/id_map.h>
#include <fast_simulator2/object.h>
#include <sstream>

#include <tue/profiling/timer.h>

int main(int argc, char **argv)
{

    sim::Simulator simulator;

    tue::Configuration config;

    std::string config_filename;
    if (argc == 2)
        config_filename = argv[1];
    else
        config_filename = "/home/sdries/ros/hydro/dev/src/fast_simulator2/test/configs/test1.yaml";

    // Load the YAML config file
    config.loadFromYAMLFile(config_filename);
    simulator.configure(config);

    if (config.hasError())
    {
        std::cout << config.error() << std::endl;
        return 1;
    }


    while(true)
    {
        // Check if reconfiguration is needed
        if (config.sync())
        {
            if (config.hasError())
                std::cout << config.error() << std::endl;
            else
            {
                simulator.configure(config);
                if (config.hasError())
                    std::cout << config.error() << std::endl;
            }
        }

        // Step the simulator
        std::vector<sim::ObjectConstPtr> changed_objects;
        simulator.step(0.01, changed_objects);

        usleep(10000);
    }

    return 0;
}
