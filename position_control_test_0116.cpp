#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "brics_actuator/JointPositions.h"

using namespace std;

// S-curve parameters
struct SCurveParams {
    double max_velocity;       // 最大速度
    double max_acceleration;   // 最大加速度
    double start_position;     // 起始位置
    double target_position;    // 目標位置
};

// Joint data (commanded and actual positions)
struct JointData {
    vector<double> command_positions;
    vector<double> actual_positions;
};

struct SaveData {
    vector<double> command;
    vector<double> actual;
};
//
// 計算線性軌跡的位置序列
vector<double> computeLinearPositionsWithDuration(const SCurveParams &params, double duration, double time_step) {
    vector<double> positions;
    size_t steps = static_cast<size_t>(duration / time_step);  // 總步數

    for (size_t i = 0; i <= steps; ++i) {
        double t = i * time_step;  // 當前時間
        double ratio = t / duration;  // 當前進度比例
        double pos = params.start_position + ratio * (params.target_position - params.start_position);
        positions.push_back(pos);
    }

    return positions;
}

// 計算 S 曲線軌跡（基於指定的 duration 和 time_step）
// 注意：這裡的函數已經包含方向處理
vector<double> computeSCurvePositionsWithDuration(
    const SCurveParams &params, double duration, double time_step) {
    
    vector<double> positions;
    double delta_position = params.target_position - params.start_position;  // 總移動距離
    double direction = (delta_position >= 0) ? 1.0 : -1.0;
    double abs_delta = fabs(delta_position);
    
    double max_velocity = params.max_velocity;
    double max_acceleration = params.max_acceleration;
    
    // 計算理想加速段時間
    double t_acc = max_velocity / max_acceleration;
    double s_acc = 0.5 * max_acceleration * t_acc * t_acc;
    double t_total_acc_dec = 2 * t_acc;
    
    double t_const = 0.0;
    //if (t_total_acc_dec < duration)
        t_const = duration - t_total_acc_dec;
    
    // 如果沒有足夠時間給恆速段（即三角形軌跡）
    if (t_const <= 0 || abs_delta < 2 * s_acc) {
        t_acc = duration / 2.0;
        s_acc = abs_delta / 2.0;
        t_const = 0.0;
        max_velocity = max_acceleration * t_acc;
    } else {
        double v_const = (abs_delta - 2 * s_acc) / t_const;
        if (v_const < max_velocity) {
            max_velocity = v_const;
            t_acc = max_velocity / max_acceleration;
            t_const = duration - 2 * t_acc;
            s_acc = 0.5 * max_acceleration * t_acc * t_acc;
        }
    }
    
    double t = 0.0;
    while (t <= duration) {
        double pos = 0.0;
        if (t < t_acc) {
            // 加速段
            pos = params.start_position + direction * 0.5 * max_acceleration * t * t;
        } else if (t>=t_acc && t <= t_acc + t_const) {
            // 恆速段
            //pos = params.start_position + direction * (s_acc + max_velocity * (t - t_acc));
            pos = positions.back() + direction * max_velocity * time_step;
        } else if (t > t_acc + t_const) {
            // 減速段
            double t_dec = (2*t_acc + t_const) - t;
            pos = params.target_position - direction * 0.5 * max_acceleration * t_dec * t_dec;
        }
        positions.push_back(pos);
        t += time_step;
    }
    
    return positions;
}



/******************************************************************************************** */
int main(int argc, char **argv) {
    ros::init(argc, argv, "s_curve_position_control_with_feedback");
    ros::NodeHandle n;

    // 發佈器與訂閱器
    ros::Publisher armPositionsPublisher = 
        n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 10);

    vector<JointData> joint_data(5); // 用於記錄每個關節的命令值和實際值
    vector<SaveData>  save_data(5); 

ros::Subscriber jointStateSubscriber = n.subscribe<sensor_msgs::JointState>(
    "/joint_states", 1, // 隊列長度設為 1
    [&](const sensor_msgs::JointState::ConstPtr &msg) {
        for (size_t i = 0; i < msg->position.size() && i < joint_data.size(); ++i) {
            joint_data[i].actual_positions.resize(1);
            joint_data[i].actual_positions[0] = msg->position[i];
        }
    }
);


    ros::Rate rate(50); // 20 Hz
    double time_step = 0.05; // 50 ms
    vector<double> durations = {4.0, 4.0, 4.0, 4.0, 4.0}; // 每個關節的運行時間（秒）

    // 初始化 S 曲線參數
    // vector<SCurveParams> joint_params = {
    //     //{1.0, 0.5, 0.011, 5.84},  // Joint 1
    //     {1.0, 0.5, 0.011, 4.8},  // Joint 1
    //     //{1.0, 0.5, 0.0100692, 2.61799}, // Joint 2
    //     {1.0, 0.5, 0.0109, 2.0}, // Joint 2
    //     //{1.0, 0.5, -0.015, -5.02655},  // Joint 3
    //     {-1.0, -0.5, -0.016, -2.0},  // Joint 3
    //     //{1.0, 0.5, 0.0221239, 3.4292}, // Joint 4
    //     {1.0, 0.5, 0.023, 2.0}, // Joint 4
    //     //{1.0, 0.5, 0.110619, 5.64159}   // Joint 5
    //     {1.0, 0.5, 0.11062, 4.0}   // Joint 5
    // };

        vector<SCurveParams> joint_params = {
        //{1.0, 0.5, 0.011, 5.84},  // Joint 1
        {1.5, 1.5, 0.011, 4.511},  // Joint 1
        //{1.0, 0.5, 0.0100692, 2.61799}, // Joint 2
        {0.5, 1.0, 0.011, 1.761}, // Joint 2
        //{1.0, 0.5, -0.015, -5.02655},  // Joint 3
        {1.5, 1.5, -0.016, -4.516},  // Joint 3
        //{1.0, 0.5, 0.0221239, 3.4292}, // Joint 4
        {0.5, 1.0, 0.023, 1.773}, // Joint 4
        //{1.0, 0.5, 0.110619, 5.64159}   // Joint 5
        {1.5, 1.5, 0.111, 4.611}   // Joint 5
    };
    
    // 計算每一軸的 S 曲線位置序列
    vector<vector<double>> joint_trajectories(joint_params.size());
    //vector<double> durations(joint_params.size());
    for (size_t i = 0; i < joint_params.size(); ++i) {
        joint_trajectories[i] = computeSCurvePositionsWithDuration(joint_params[i], durations[i], time_step);
    }
    // 計算每一軸的線性軌跡位置序列
    // for (size_t i = 0; i < joint_params.size(); ++i) {
    //     joint_trajectories[i] = computeLinearPositionsWithDuration(joint_params[i], durations[i], time_step);
    // }

    size_t max_steps = 0;
    for (const auto &trajectory : joint_trajectories) {
        max_steps = max(max_steps, trajectory.size());
    }


// 等待 joint_states 初始化
ROS_INFO("Waiting for joint_states messages...");
while (ros::ok() && joint_data[0].actual_positions.empty()) {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
}
ROS_INFO("Joint states initialized.");


    // 發送軌跡點並記錄實際位置
// 發送命令與反饋同步
for (size_t step = 0; step < max_steps && ros::ok(); ++step) {
    brics_actuator::JointPositions command;
    vector<brics_actuator::JointValue> armJointPositions(joint_params.size());
    std::stringstream jointName;

    for (size_t i = 0; i < joint_params.size(); ++i) {
        double command_value = (step < joint_trajectories[i].size()) 
            ? joint_trajectories[i][step] 
            : joint_params[i].target_position;

        jointName.str("");
        jointName << "arm_joint_" << (i + 1);

        armJointPositions[i].joint_uri = jointName.str();
        armJointPositions[i].value = command_value;
        armJointPositions[i].unit = "rad";

        joint_data[i].command_positions.push_back(command_value);
        save_data[i].command.push_back(command_value);
    }

    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);

    // 等待反饋同步
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 0.1) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // 打印命令與反饋對比
    for (size_t i = 0; i < joint_data.size(); ++i) {
        double actual_value = !joint_data[i].actual_positions.empty() 
            ? joint_data[i].actual_positions.back()
            : 0.0;
        save_data[i].actual.push_back(actual_value);
        ROS_INFO("Joint %lu: Command = %.3f, Actual = %.3f", i + 1, 
                 joint_data[i].command_positions.back(), actual_value);
    }

    rate.sleep();
}

// 打印實際位置與命令位置的對比並寫入 CSV 檔案
ROS_INFO("Saving joint positions to CSV...");
std::string file_path = "./joint_positions.csv"; // 確保路徑存在
std::ofstream csv_file(file_path);

if (csv_file.is_open()) {
    csv_file << "Joint,Step,Command,Actual\n";
    for (size_t i = 0; i < save_data.size(); ++i) {
        size_t num_steps = save_data[i].command.size();
        for (size_t j = 0; j < num_steps; ++j) {
            double command_value = save_data[i].command[j];
            double actual_value = (j < save_data[i].actual.size()) 
                ? save_data[i].actual[j] 
                : 0.0;

            csv_file << (i + 1) << "," << j << "," << command_value << "," << actual_value << "\n";
        }
    }
    csv_file.close();
    ROS_INFO("Joint positions successfully saved to %s", file_path.c_str());
} else {
    ROS_ERROR("Failed to open file for writing: %s", file_path.c_str());
}


    ROS_INFO("S-curve trajectory with feedback complete.");
    return 0;
}
