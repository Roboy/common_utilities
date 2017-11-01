#include "common_utilities/MotorConfig.hpp"

bool MotorConfig::readConfig(const string &filepath){
    if(!fileExists(filepath))
        return false;
    YAML::Node config = YAML::LoadFile(filepath);
    vector<vector<float>> motor_polynomial_parameter =
            config["motor_polynomial_parameters_displacement2force"].as<vector<vector<float >>>();
    for (int i = 0; i < motor_polynomial_parameter.size(); i++) {
        vector<float> polynomial_parameters;
        int motor_id = 0;
        bool first_value = true;
        for(auto &val:motor_polynomial_parameter[i]){
            if(first_value) {
                motor_id = val;
                first_value = false;
            }else {
                polynomial_parameters.push_back(val);
            }
        }
        coeffs_displacement2force[motor_id] = polynomial_parameters;
//        ROS_INFO_STREAM(motor_id << "\t" << coeffs_displacement2force[motor_id][0]<< "\t" << coeffs_displacement2force[motor_id][1]
//                                 << "\t" << coeffs_displacement2force[motor_id][2]<< "\t" << coeffs_displacement2force[motor_id][3]
//                                 << "\t" << coeffs_displacement2force[motor_id][4]);
    }
    motor_polynomial_parameter =
            config["motor_polynomial_parameters_force2displacement"].as<vector<vector<float >>>();
    for (int i = 0; i < motor_polynomial_parameter.size(); i++) {
        vector<float> polynomial_parameters;
        int motor_id = 0;
        bool first_value = true;
        for(auto &val:motor_polynomial_parameter[i]){
            if(first_value) {
                motor_id = val;
                first_value = false;
            }else {
                polynomial_parameters.push_back(val);
            }
        }
        coeffs_force2displacement[motor_id] = polynomial_parameters;
//        ROS_INFO_STREAM(motor_id << "\t" << coeffs_force2displacement[motor_id][0]<< "\t" << coeffs_force2displacement[motor_id][1]
//                                 << "\t" << coeffs_force2displacement[motor_id][2]<< "\t" << coeffs_force2displacement[motor_id][3]
//                                 << "\t" << coeffs_force2displacement[motor_id][4]);
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
    for (auto const &coefficients : coeffs_displacement2force) {
        stringstream str;
        str << "[";
        for(uint i=0;i<=coefficients.second.size();i++){
            if( i <coefficients.second.size())
                str << "0,";
            else
                str << "0]";
        }
        YAML::Node node = YAML::Load(str.str());
        // first number is the sensor id
        node[0] = coefficients.first;
        for (int i = 0; i < coefficients.second.size(); i++) {
            node[i+1] = coefficients.second[i];
        }
        config["motor_polynomial_parameters_displacement2force"].push_back(node);
//        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_displacement2force[coefficients.first][0]<< "\t" << coeffs_displacement2force[coefficients.first][1]
//                                           << "\t" << coeffs_displacement2force[coefficients.first][2]<< "\t" << coeffs_displacement2force[coefficients.first][3]
//                                           << "\t" << coeffs_displacement2force[coefficients.first][4]);
    }

    for (auto const &coefficients : coeffs_force2displacement) {
        stringstream str;
        str << "[";
        for(uint i=0;i<=coefficients.second.size();i++){
            if( i <coefficients.second.size())
                str << "0,";
            else
                str << "0]";
        }
        YAML::Node node = YAML::Load(str.str());
        // first number is the sensor id
        node[0] = coefficients.first;
        for (int i = 0; i < coefficients.second.size(); i++) {
            node[i+1] = coefficients.second[i];
        }
        config["motor_polynomial_parameters_force2displacement"].push_back(node);
//        ROS_DEBUG_STREAM(coefficients.first << "\t" << coeffs_force2displacement[coefficients.first][0]<< "\t" << coeffs_force2displacement[coefficients.first][1]
//                                           << "\t" << coeffs_force2displacement[coefficients.first][2]<< "\t" << coeffs_force2displacement[coefficients.first][3]
//                                           << "\t" << coeffs_force2displacement[coefficients.first][4]);
    }

    fout << config;
    return true;
}

bool MotorConfig::fileExists(const string &filepath){
    struct stat buffer;
    return (stat (filepath.c_str(), &buffer) == 0);
}