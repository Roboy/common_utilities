#include "common_utilities/MotorConfig.hpp"

bool MotorConfig::readConfig(const string &filepath){
    if(!fileExists(filepath)) {
        ROS_FATAL_STREAM(filepath << " does not exist, check your path");
        return false;
    }
    YAML::Node config = YAML::LoadFile(filepath);
    number_of_icebuses = config["number_of_icebuses"].as<int>();
    if(number_of_icebuses==0)
        ROS_FATAL("no icebus defined, check your motor config file");

    for(int icebus=0;icebus<number_of_icebuses;icebus++){
        char str[20];
        sprintf(str,"icebus_%d",icebus);
        int number_of_motors = config[str]["number_of_motors"].as<int>();
        vector<int> motor_ids = config[str]["motor_ids"].as<vector<int>>();
        vector<int> motor_ids_global = config[str]["motor_ids_global"].as<vector<int>>();
        vector<vector<float>> coeffs_force2displacement = config[str]["coeffs_force2displacement"].as<vector<vector<float>>>();
        vector<vector<float>> coeffs_displacement2force = config[str]["coeffs_displacement2force"].as<vector<vector<float>>>();
        if(motor_ids.size()!=number_of_motors){
            ROS_ERROR("motor_ids of icebus %d does not match number_of_motors, check your motor config file",icebus);
            continue;
        }
        if(motor_ids_global.size()!=number_of_motors){
            ROS_ERROR("motor_ids_global of icebus %d does not match number_of_motors, check your motor config file",icebus);
            continue;
        }
        if(coeffs_force2displacement.size()!=number_of_motors){
            ROS_ERROR("coeffs_force2displacement of icebus %d does not match number_of_motors, check your motor config file",icebus);
            continue;
        }
        if(coeffs_displacement2force.size()!=number_of_motors){
            ROS_ERROR("coeffs_displacement2force of icebus %d does not match number_of_motors, check your motor config file",icebus);
            continue;
        }
        for(int m=0;m<number_of_motors;m++){
            motor[motor_ids_global[m]].reset(new Motor(icebus,motor_ids[m],motor_ids_global[m],coeffs_force2displacement[m],
                                            coeffs_displacement2force[m]));
        }
        total_number_of_motors += number_of_motors;
    }

    return true;
}

bool MotorConfig::writeConfig(const string &filepath){
//    std::ofstream fout(filepath);
//    if (!fout.is_open()) {
//        ROS_WARN_STREAM("Could not write config " << filepath);
//        return false;
//    }
//    YAML::Node config;
//    for(int fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
//        char fpgastr[200];
//        sprintf(fpgastr,"fpga%d",fpga);
//        int motor = 0;
//        for (auto const &coefficients : coeffs_displacement2force[fpga]) {
//            stringstream str;
//            str << "[";
//            for (uint i = 0; i <= coefficients.size(); i++) {
//                if (i < coefficients.size())
//                    str << "0,";
//                else
//                    str << "0]";
//            }
//            YAML::Node node = YAML::Load(str.str());
//            // first number is the sensor id
//            node[0] = motor;
//            for (int i = 0; i < coefficients.size(); i++) {
//                node[i + 1] = coefficients[i];
//            }
//            config[fpgastr]["motor_polynomial_parameters_displacement2force"].push_back(node);
//            motor++;
////        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_displacement2force[coefficients.first][0]<< "\t" << coeffs_displacement2force[coefficients.first][1]
////                                           << "\t" << coeffs_displacement2force[coefficients.first][2]<< "\t" << coeffs_displacement2force[coefficients.first][3]
////                                           << "\t" << coeffs_displacement2force[coefficients.first][4]);
//        }
//        motor = 0;
//        for (auto const &coefficients : coeffs_force2displacement[fpga]) {
//            stringstream str;
//            str << "[";
//            for (uint i = 0; i <= coefficients.size(); i++) {
//                if (i < coefficients.size())
//                    str << "0,";
//                else
//                    str << "0]";
//            }
//            YAML::Node node = YAML::Load(str.str());
//            // first number is the sensor id
//            node[0] = motor;
//            for (int i = 0; i < coefficients.size(); i++) {
//                node[i + 1] = coefficients[i];
//            }
//            config[fpgastr]["motor_polynomial_parameters_force2displacement"].push_back(node);
//            motor++;
////        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_force2displacement[coefficients.first][0]<< "\t" << coeffs_force2displacement[coefficients.first][1]
////                                           << "\t" << coeffs_force2displacement[coefficients.first][2]<< "\t" << coeffs_force2displacement[coefficients.first][3]
////                                           << "\t" << coeffs_force2displacement[coefficients.first][4]);
//        }
//    }
//    fout << config;
    return true;
}

bool MotorConfig::fileExists(const string &filepath){
    struct stat buffer;
    return (stat (filepath.c_str(), &buffer) == 0);
}

double MotorConfig::displacement2force(double displacement, int motor_id_global){
    double force = 0;
    for (uint i = 0; i < motor[motor_id_global]->coeffs_displacement2force.size(); i++) {
        force += motor[motor_id_global]->coeffs_displacement2force[i] * pow(displacement, (double) i);
    }
    return force;
}

double MotorConfig::force2displacement(double force, int motor_id_global){
    double displacement = 0;
    for (uint i = 0; i < motor[motor_id_global]->coeffs_force2displacement.size(); i++) {
        displacement += motor[motor_id_global]->coeffs_force2displacement[i] * pow(force, (double) i);
    }
    return displacement;
}