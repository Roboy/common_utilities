#pragma once

#include <ros/ros.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;

class MotorConfig{
public:
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

    map<int, vector<float>> coeffs_displacement2force, coeffs_force2displacement;
};