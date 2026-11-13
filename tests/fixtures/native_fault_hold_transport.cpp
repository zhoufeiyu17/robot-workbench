// In-memory CAN substitute for isolated native fault injection.
#include "hardware/arx_can.h"
#include <atomic>
#include <map>

namespace {
std::mutex mutex;
std::map<const ArxCan *, std::array<OD_Motor_Msg, 10>> states;
std::atomic<uint64_t> sends{0};
float initial_joint_four = 0.2f;
}

std::string workbench_guarded_can_interface(const std::string &value) { return value; }
bool workbench_ec_guard_ready() { return true; }
std::string workbench_ec_guard_vendor() { return "isolated-memory-transport"; }
extern "C" uint64_t workbench_test_sends() { return sends.load(); }
extern "C" void workbench_test_initial_joint_four(float value) { initial_joint_four = value; }
extern "C" void workbench_test_set_gripper(float value) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &row : states) row.second[8].angle_actual_rad = value;
}

ArxCan::ArxCan(std::string) {
    std::lock_guard<std::mutex> lock(mutex);
    auto &rows = states[this];
    for (auto &row : rows) row.angle_actual_rad = 0.2f;
    rows[5].angle_actual_rad = initial_joint_four;
}
ArxCan::~ArxCan() { std::lock_guard<std::mutex> lock(mutex); states.erase(this); }
void ArxCan::send_EC_motor_cmd(uint16_t, float, float, float, float, float) { ++sends; }
void ArxCan::send_DM_motor_cmd(uint16_t, float, float, float, float, float) { ++sends; }
void ArxCan::can_cmd_init(uint16_t, uint8_t) {}
void ArxCan::query_EC_motor_pos(uint16_t) {}
void ArxCan::query_EC_motor_vel(uint16_t) {}
void ArxCan::query_EC_motor_current(uint16_t) {}
void ArxCan::set_motor(uint16_t, uint8_t) {}
void ArxCan::enable_DM_motor(uint16_t) {}
void ArxCan::reset_zero_readout(uint16_t) {}
void ArxCan::clear(uint16_t) {}
const std::array<OD_Motor_Msg, 10> ArxCan::get_motor_msg() {
    std::lock_guard<std::mutex> lock(mutex);
    return states.at(this);
}
