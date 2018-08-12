#pragma once

#include <ros/ros.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <common_utilities/CommonDefinitions.h>

using namespace std;

class MotorConfig{
public:
    MotorConfig();
    /**
     * Reads a yaml motor config file
     * @param filepath to config
     * @return success
     */
    bool readConfig(const string &filepath);
    /**
     * Writes a yaml motor config file
     * @param filepath
     * @return success
     */
    bool writeConfig(const string &filepath);
    /**
     * Checks if a file exists
     * @param filepath
     * @return exists
     */
    inline bool fileExists(const string &filepath);
    /**
     * Transforms displacement to force using loaded coefficients
     * @param displacement
     * @param fpga for this fpga
     * @param motor motor id (as listed in read config)
     * @return force
     */
    double displacement2force(double displacement, int fpga, int motor);
    /**
     * Transforms force to displacement using loaded coefficients
     * @param displacement
     * @param fpga for this fpga
     * @param motor motor id (as listed in read config)
     * @return force
     */
    double force2displacement(double force, int fpga, int motor);

    vector<vector<vector<float>>> coeffs_displacement2force, coeffs_force2displacement;
};