#include "common_utilities/MotorConfig.hpp"

MotorConfig::MotorConfig(){
    coeffs_displacement2force.resize(NUMBER_OF_FPGAS);
    coeffs_force2displacement.resize(NUMBER_OF_FPGAS);
    for(int fpga=0;fpga<NUMBER_OF_FPGAS;fpga++){
        coeffs_displacement2force[fpga].resize(NUMBER_OF_MOTORS_PER_FPGA);
        coeffs_force2displacement[fpga].resize(NUMBER_OF_MOTORS_PER_FPGA);
    }
}

bool MotorConfig::readConfig(const string &filepath){
    if(!fileExists(filepath)) {
        ROS_ERROR_STREAM(filepath << " does not exist, check your path");
        return false;
    }
    YAML::Node config = YAML::LoadFile(filepath);
    for(int fpga=0;fpga<NUMBER_OF_FPGAS;fpga++) {
        char str[200];
        sprintf(str, "fpga%d", fpga);
        vector<vector<float>> motor_polynomial_parameter =
                config[str]["motor_polynomial_parameters_displacement2force"].as<vector<vector<float >>>();
        for (int i = 0; i < motor_polynomial_parameter.size(); i++) {
            vector<float> polynomial_parameters;
            int motor_id = 0;
            bool first_value = true;
            for (auto &val:motor_polynomial_parameter[i]) {
                if (first_value) {
                    motor_id = val;
                    first_value = false;
                } else {
                    polynomial_parameters.push_back(val);
                }
            }
            coeffs_displacement2force[fpga][motor_id] = polynomial_parameters;
//        ROS_INFO_STREAM(motor_id << "\t" << coeffs_displacement2force[motor_id][0]<< "\t" << coeffs_displacement2force[motor_id][1]
//                                 << "\t" << coeffs_displacement2force[motor_id][2]<< "\t" << coeffs_displacement2force[motor_id][3]
//                                 << "\t" << coeffs_displacement2force[motor_id][4]);
        }
        motor_polynomial_parameter =
                config[str]["motor_polynomial_parameters_force2displacement"].as<vector<vector<float >>>();
        for (int i = 0; i < motor_polynomial_parameter.size(); i++) {
            vector<float> polynomial_parameters;
            int motor_id = 0;
            bool first_value = true;
            for (auto &val:motor_polynomial_parameter[i]) {
                if (first_value) {
                    motor_id = val;
                    first_value = false;
                } else {
                    polynomial_parameters.push_back(val);
                }
            }
            coeffs_force2displacement[fpga][motor_id] = polynomial_parameters;
//        ROS_INFO_STREAM(motor_id << "\t" << coeffs_force2displacement[motor_id][0]<< "\t" << coeffs_force2displacement[motor_id][1]
//                                 << "\t" << coeffs_force2displacement[motor_id][2]<< "\t" << coeffs_force2displacement[motor_id][3]
//                                 << "\t" << coeffs_force2displacement[motor_id][4]);
        }
    }
    return true;
}

bool MotorConfig::writeConfig(const string &filepath){
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }
    YAML::Node config;
    for(int fpga = 0; fpga<NUMBER_OF_FPGAS; fpga++) {
        char fpgastr[200];
        sprintf(fpgastr,"fpga%d",fpga);
        int motor = 0;
        for (auto const &coefficients : coeffs_displacement2force[fpga]) {
            stringstream str;
            str << "[";
            for (uint i = 0; i <= coefficients.size(); i++) {
                if (i < coefficients.size())
                    str << "0,";
                else
                    str << "0]";
            }
            YAML::Node node = YAML::Load(str.str());
            // first number is the sensor id
            node[0] = motor;
            for (int i = 0; i < coefficients.size(); i++) {
                node[i + 1] = coefficients[i];
            }
            config[fpgastr]["motor_polynomial_parameters_displacement2force"].push_back(node);
            motor++;
//        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_displacement2force[coefficients.first][0]<< "\t" << coeffs_displacement2force[coefficients.first][1]
//                                           << "\t" << coeffs_displacement2force[coefficients.first][2]<< "\t" << coeffs_displacement2force[coefficients.first][3]
//                                           << "\t" << coeffs_displacement2force[coefficients.first][4]);
        }
        motor = 0;
        for (auto const &coefficients : coeffs_force2displacement[fpga]) {
            stringstream str;
            str << "[";
            for (uint i = 0; i <= coefficients.size(); i++) {
                if (i < coefficients.size())
                    str << "0,";
                else
                    str << "0]";
            }
            YAML::Node node = YAML::Load(str.str());
            // first number is the sensor id
            node[0] = motor;
            for (int i = 0; i < coefficients.size(); i++) {
                node[i + 1] = coefficients[i];
            }
            config[fpgastr]["motor_polynomial_parameters_force2displacement"].push_back(node);
            motor++;
//        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_force2displacement[coefficients.first][0]<< "\t" << coeffs_force2displacement[coefficients.first][1]
//                                           << "\t" << coeffs_force2displacement[coefficients.first][2]<< "\t" << coeffs_force2displacement[coefficients.first][3]
//                                           << "\t" << coeffs_force2displacement[coefficients.first][4]);
        }
    }
    fout << config;
    return true;
}

bool MotorConfig::fileExists(const string &filepath){
    struct stat buffer;
    return (stat (filepath.c_str(), &buffer) == 0);
}

double MotorConfig::displacement2force(double displacement, int fpga, int motor){
    double force = 0;
    for (uint i = 0; i < coeffs_displacement2force[fpga][motor].size(); i++) {
        force += coeffs_displacement2force[fpga][motor][i] * pow(displacement, (double) i);
    }
    return force;
}

double MotorConfig::force2displacement(double force, int fpga, int motor){
    double displacement = 0;
    for (uint i = 0; i < coeffs_force2displacement[fpga][motor].size(); i++) {
        displacement += coeffs_force2displacement[fpga][motor][i] * pow(force, (double) i);
    }
    return displacement;
}