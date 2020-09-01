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
    Motor(int bus, int bus_id, int baudrate, int update_frequency, int motor_id,
          int motor_id_global, string muscleType,
          float encoder0_conversion_factor,
          float encoder1_conversion_factor,
          vector<float> &coeffs_force2displacement,
          vector<float> &coeffs_displacement2force):
          bus(bus), bus_id(bus_id), baudrate(baudrate), update_frequency(update_frequency),motor_id(motor_id),
          motor_id_global(motor_id_global),muscleType(muscleType),encoder0_conversion_factor(encoder0_conversion_factor),
          encoder1_conversion_factor(encoder1_conversion_factor),
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
        ROS_INFO("%d \t |  %d \t |   %d \t | %d",motor_id_global, bus, motor_id, bus_id);
    };
    int bus, bus_id, baudrate, update_frequency, motor_id, motor_id_global;
    float encoder0_conversion_factor, encoder1_conversion_factor;
    uint8_t control_mode = ENCODER0_POSITION;
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
    int number_of_icebuses = 0, number_of_myobuses = 0, total_number_of_motors = 0;
    map<int, MotorPtr> motor;
    map<int, vector<MotorPtr>> icebus, armbus, myobus;
    map<int, BodyPartPtr> body_part;
private:
    void yaml_error();
};

typedef boost::shared_ptr<MotorConfig> MotorConfigPtr;

static const string example_motor_config = {
"icebus:\n"
"  number_of_motors: [10,6]\n"
"  update_frequency: [100,100]\n"
"  motor_ids:\n"
"    - [0,1,2,3,4,5,6,7,8,9]\n"
"    - [0,1,2,3,4,5]\n"
"  baudrate:\n"
"    - [460800,460800,460800,460800,460800,460800,19200,19200,19200,19200]\n"
"    - [2000000,2000000,2000000,2000000,2000000,2000000]\n"
"  motor_ids_global:\n"
"    - [0,1,2,3,4,5,6,7,8,9]\n"
"    - [10,11,12,13,14,15]\n"
"  bus_ids:\n"
"    - [128,129,130,131,132,133,134,135,136,137]\n"
"    - [128,129,130,131,132,133]\n"
"  muscle_type:\n"
"    - [\"m3\",\"m3\",\"m3\",\"m3\",\"m3\",\"m3\",\"openBionics\",\"openBionics\",\"openBionics\",\"openBionics\"]\n"
"    - [\"myoBrick\",\"myoBrick\",\"myoBrick\",\"myoBrick\",\"myoBrick\",\"myoBrick\"]\n"
"  encoder0_conversion_factor:\n"
"    - [1,1,1,1,1,1,1,1,1,1]\n"
"    - [1,1,1,1,1,1]\n"
"  encoder1_conversion_factor:\n"
"    - [1,1,1,1,1,1,1,1,1,1]\n"
"    - [1,1,1,1,1,1]\n"
"  coeffs_force2displacement: #not used yet\n"
"    - [0,0,1]\n"
"  coeffs_displacement2force: #not used yet\n"
"    - [0,0,1]\n\n"
"myobus:\n"
"  number_of_motors: [0]\n"
"  update_frequency: [100]\n"
"  motor_ids:\n"
"    - [0,1,2,3,4,5,6,7,8,9]\n"
"  motor_ids_global:\n"
"    - [16,17,18,19,20,21,22,23,24,25]\n"
"  bus_ids:\n"
"    - [0,1,2,3,4,5,6,7,8,9]\n"
"  muscle_type:\n"
"    - [\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\",\"myoMuscle\"]\n"
"  encoder0_conversion_factor:\n"
"    - [1,1,1,1,1,1,1,1,1,1]\n"
"  encoder1_conversion_factor:\n"
"    - [1,1,1,1,1,1,1,1,1,1]\n"
"  coeffs_force2displacement: #not used yet\n"
"    - [0,0,1]\n"
"  coeffs_displacement2force: #not used yet\n"
"    - [0,0,1]\n\n"
"body_part:\n"
"  name: [\"shoulder\"]\n"
"  motor_ids_global:\n"
"    - [0]\n\n"
"elbow:\n"
"  order: [0,1,2,3]\n"
"  sign: [1,1,1,-1]\n"
"  offset: [43.2,82.5,215.9,218.8]\n"};
