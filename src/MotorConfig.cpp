#include "common_utilities/MotorConfig.hpp"

bool MotorConfig::readConfig(const string &filepath){
    if(!fileExists(filepath)) {
        ROS_FATAL_STREAM(filepath << " does not exist, check your path");
        return false;
    }
    YAML::Node config = YAML::LoadFile(filepath);
    vector<int> number_of_motors = config["icebus"]["number_of_motors"].as<vector<int>>();
    number_of_icebuses = number_of_motors.size();
    vector<vector<int>> bus_ids = config["icebus"]["bus_ids"].as<vector<vector<int>>>();
    vector<vector<int>> baudrate = config["icebus"]["baudrate"].as<vector<vector<int>>>();
    vector<int> update_frequency = config["icebus"]["update_frequency"].as<vector<int>>();
    vector<vector<int>> motor_ids = config["icebus"]["motor_ids"].as<vector<vector<int>>>();
    vector<vector<int>> motor_ids_global = config["icebus"]["motor_ids_global"].as<vector<vector<int>>>();
    vector<vector<string>> muscleType = config["icebus"]["muscle_type"].as<vector<vector<string>>>();
    vector<vector<float>> coeffs_force2displacement = config["icebus"]["coeffs_force2displacement"].as<vector<vector<float>>>();
    vector<vector<float>> coeffs_displacement2force = config["icebus"]["coeffs_displacement2force"].as<vector<vector<float>>>();
    if(number_of_icebuses==0){
      ROS_WARN("no motors defined, check motor_config yaml file");
      return false;
    }else{
      ROS_INFO("configuring %d icebuses",number_of_icebuses);
      for(int i=0;i<number_of_icebuses;i++){
          ROS_INFO("configuring icebus %d with %d motors",i,number_of_motors[i]);
          if(motor_ids[i].size()>number_of_motors[i]){
              ROS_ERROR("motor_ids of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              motor_ids[i].resize(number_of_motors[i]);
          }
          if(bus_ids[i].size()>number_of_motors[i]){
              ROS_ERROR("bus_ids of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              bus_ids[i].resize(number_of_motors[i]);
          }
          if(baudrate[i].size()>number_of_motors[i]){
              ROS_ERROR("baudrate of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              baudrate[i].resize(number_of_motors[i]);
          }
          if(motor_ids_global.size()>number_of_motors[i]){
              ROS_ERROR("motor_ids_global of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              motor_ids_global[i].resize(number_of_motors[i]);
          }
          if(coeffs_force2displacement.size()!=number_of_motors[i]){
              ROS_WARN("coeffs_force2displacement not implemented");
              // ROS_ERROR("coeffs_force2displacement of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              coeffs_force2displacement.resize(number_of_motors[i]);
          }
          if(coeffs_displacement2force.size()!=number_of_motors[i]){
              ROS_WARN("coeffs_displacement2force not implemented");
              // ROS_ERROR("coeffs_displacement2force of icebus %d does not match number_of_motors, check your motor config file, adjusting to number_of_motors parameter and continue",i);
              coeffs_displacement2force.resize(number_of_motors[i]);
          }
          for(int m=0;m<number_of_motors[i];m++){
              MotorPtr motor_ = MotorPtr(
                  new Motor(i,bus_ids[i][m],baudrate[i][m],update_frequency[i],
                      motor_ids[i][m],motor_ids_global[i][m],muscleType[i][m],
                      coeffs_force2displacement[m], coeffs_displacement2force[m]));
              motor[motor_ids_global[i][m]] = motor_;
              icebus[i].push_back(motor_);
          }
          total_number_of_motors += number_of_motors[i];
      }
    }

    vector<string> body_parts = config["body_part"]["name"].as<vector<string>>();
    vector<vector<int>> body_part_motor_ids_global = config["body_part"]["motor_ids_global"].as<vector<vector<int>>>();
    for(int i=0;i<body_parts.size();i++){
        body_part[i].reset(new BodyPart());
        body_part[i]->name = body_parts[i];
        for(int j=0;j<body_part_motor_ids_global[i].size();j++){
            if(motor.find(body_part_motor_ids_global[i][j]) != motor.end())
              body_part[i]->motor_ids_global.push_back(motor[body_part_motor_ids_global[i][j]]);
            else
              ROS_ERROR("the motor %d assigned to body_part %s does not exist, check your motor config file", body_part_motor_ids_global[i][j], body_part[i]->name.c_str());
        }
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
