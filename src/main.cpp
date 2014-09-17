#include "simulator.h"

#include <tue/config/configuration.h>

#include <iostream>

int main(int argc, char **argv)
{
    sim::Simulator simulator;

    tue::Configuration config;

    if (argc == 2)
    {
        // Load the YAML config file
        config.loadFromYAMLFile(argv[1]);
        simulator.configure(config);

        if (config.hasError())
        {
            std::cout << config.error() << std::endl;
            return 1;
        }
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

        usleep(10000);
    }

    return 0;
}
