#include "include/HandDataJSON.hpp"

using json = nlohmann::json;

std::string HandDataJSON::get_current_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt;
    localtime_s(&bt, &timer);  // 安全版本

    std::ostringstream oss;
    oss << std::put_time(&bt, "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

    return oss.str();
}
// FingerJoints 序列化
void to_json(json& j, const FingerJoints& f) {
    j = {
        {"cmc_roll", f.cmc_roll},
        {"cmc_yaw", f.cmc_yaw},
        {"cmc_pitch", f.cmc_pitch},
        {"mcp", f.mcp},
        {"pip", f.pip},
        {"dip", f.dip}
    };
}

// Hand 序列化
void to_json(json& j, const HandJoints& h) {
    j = {
        {"thumb", h.thumb},
        {"index", h.index},
        {"middle", h.middle},
        {"ring", h.ring},
        {"pinky", h.pinky}  // 处理原始拼写
    };
}

void from_json(const json& j, FingerForces& f) {
    j.at("normalforce").get_to(f.normalforce);
    j.at("approachforce").get_to(f.approachforce);
    j.at("tangentialforce").get_to(f.tangentialforce);
}


void from_json(const json& j, HandForces& h) {
    j.at("thumb").get_to(h.thumb);
    j.at("index").get_to(h.index);
    j.at("middle").get_to(h.middle);
    j.at("ring").get_to(h.ring);
    j.at("pinky").get_to(h.pinky);
}

HandDataJSON::HandDataJSON(const std::string& json_str) {
    json j = json::parse(json_str);
    from_json(j);
}

std::string HandDataJSON::to_json_string(bool pretty) const {
    json j = {
        {"timsstamp", get_current_timestamp()},
        {"datatype", data_type},
        {"euler", {
            {"right", right_hand_joint},
            {"left", left_hand_joint}
        }}
    };
    return pretty ? j.dump(4) : j.dump();
}

SensorData HandDataJSON::from_json(const json& j) {
    //try {

        SensorData data;
        if (j.contains("datatype")) {
            j.at("datatype").get_to(data.datatype);
        }
        if (j.contains("right")) {
            j.at("right").get_to(data.right);
        }
        if (j.contains("left")) {
            j.at("left").get_to(data.left);
        }
    //    j.at("timsstamp").get_to(data.timestamp); // 注意JSON中的拼写错误
    //    j.at("datatype").get_to(data.datatype);
    //    j.at("right").get_to(data.right);
    //    j.at("left").get_to(data.left);

        return data;
    //}
    //catch (const json::exception& e) {
    //    throw std::runtime_error("JSON解析错误: " + std::string(e.what()));
    //}
}




