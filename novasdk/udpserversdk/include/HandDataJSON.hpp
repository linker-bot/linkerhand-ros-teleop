#pragma once

#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

// 前置声明和结构体定义
struct FingerJoints {
    double cmc_roll = 0.0;
    double cmc_yaw = 0.0;
    double cmc_pitch = 0.0;
    double mcp = 0.0;
    double pip = 0.0;
    double dip = 0.0;
};

struct HandJoints {
    FingerJoints thumb;
    FingerJoints index;
    FingerJoints middle;
    FingerJoints ring;
    FingerJoints pinky;
};

struct FingerForces {
    double normalforce;
    double approachforce;
    double tangentialforce;
};

struct HandForces {
    FingerForces thumb;
    FingerForces index;
    FingerForces middle;
    FingerForces ring;
    FingerForces pinky;
};

struct SensorData {
    std::string timestamp;
    std::string datatype;
    HandForces right;
    HandForces left;
};

void to_json(json& j, const FingerJoints& f);
void from_json(const json& j, FingerForces& f);
void to_json(json& j, const HandJoints& h);
void from_json(const json& j, HandForces& h);

class HandDataJSON {
private:
    std::string get_current_timestamp() const;
    HandJoints right_hand_joint;
    HandJoints left_hand_joint;
    HandForces right_hand_force;
    HandForces left_hand_force;
    std::string data_type = "datasend";

public:
    HandDataJSON() = default;
    explicit HandDataJSON(const std::string& json_str);

    std::string to_json_string(bool pretty = false) const;
    SensorData from_json(const json& j);

    HandJoints& right() { return right_hand_joint; }
    HandJoints& left() { return left_hand_joint; }
};