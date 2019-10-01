#pragma once

#include <ros/ros.h>
#include <map>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "CommonDefinitions.h"

using namespace std;

class Motor{
public:
    Motor(int icebus, int icebus_id, int motor_id_global, string muscleType,
          vector<float> &coeffs_force2displacement,
          vector<float> &coeffs_displacement2force):
          icebus(icebus), icebus_id(icebus_id), motor_id_global(motor_id_global),muscleType(muscleType),
          coeffs_force2displacement(coeffs_force2displacement),
          coeffs_displacement2force(coeffs_displacement2force){
        stringstream str;
        str << "force -> displacement" << "\t ";
        for(int i=0;i<coeffs_force2displacement.size();i++){
            str << coeffs_force2displacement[i] << "\t";
        }
        str << "\n displacement -> force" << "\t ";
        for(int i=0;i<coeffs_displacement2force.size();i++){
            str << coeffs_displacement2force[i] << "\t";
        }
        ROS_INFO("Motor with global id %d on icebus %d with icebus_id %d initialized with polynomial parameters:"
                 "%s",motor_id_global, icebus, icebus_id, str.str().c_str());
    };
    int icebus, icebus_id, motor_id_global;
    vector<float> coeffs_force2displacement;
    vector<float> coeffs_displacement2force;
    string muscleType;
};

typedef boost::shared_ptr<Motor> MotorPtr;

class BodyPart{
public:

    vector<MotorPtr> motor_ids_global;
    string name;
};

typedef boost::shared_ptr<BodyPart> BodyPartPtr;

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
    /**
     * Transforms displacement to force using loaded coefficients
     * @param displacement
     * @param fpga for this fpga
     * @param motor motor id (as listed in read config)
     * @return force
     */
    double displacement2force(double displacement, int motor_id_global);
    /**
     * Transforms force to displacement using loaded coefficients
     * @param displacement
     * @param fpga for this fpga
     * @param motor motor id (as listed in read config)
     * @return force
     */
    double force2displacement(double force, int motor_id_global);
    int number_of_icebuses = 0, total_number_of_motors = 0;
    map<int, MotorPtr> motor;
    map<int, vector<MotorPtr>> icebus;
    map<int, BodyPartPtr> body_part;
};

typedef boost::shared_ptr<MotorConfig> MotorConfigPtr;