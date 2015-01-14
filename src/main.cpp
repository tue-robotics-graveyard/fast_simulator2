#include "fast_simulator2/simulator.h"

#include <tue/config/configuration.h>

#include <iostream>

//#include <fast_simulator2/id_map.h>
//#include <fast_simulator2/object.h>
#include <sstream>

#include <tue/profiling/timer.h>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "[Fast Simulator 2] Please provide configuration file." << std::endl;
        return 1;
    }

    std::string config_filename = argv[1];

    sim::Simulator simulator;

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    // Get plugin paths
    const char* sim_plugin_path = ::getenv("SIM_PLUGIN_PATH");
    if (sim_plugin_path == 0)
    {
        std::cout << "Error: Environment variable SIM_PLUGIN_PATH not set." << std::endl;
        return 1;
    }

    std::stringstream ss(sim_plugin_path);
    std::string item;
    while (std::getline(ss, item, ':'))
        simulator.addPluginPath(item);

    // - - - - - - - - - - - - - -

    // Load the YAML config file
    tue::Configuration config;
    config.loadFromYAMLFile(config_filename);
    simulator.configure(config);

    if (config.hasError())
    {
        std::cout << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
        simulator.step(0.01);

        usleep(10000);
    }

    return 0;
}
