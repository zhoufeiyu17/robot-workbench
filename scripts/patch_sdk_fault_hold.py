#!/usr/bin/env python3
"""Apply the workbench fault-hold extension to a separate SDK source candidate."""
import argparse
from pathlib import Path


HOLD_METHODS = '''
void Arx5ControllerBase::bind_fault_hold(int fd, int side)
{
    std::lock_guard<std::recursive_mutex> gate(lifecycle_mutex_);
    fault_hold_.bind(fd, side);
}

void Arx5ControllerBase::cache_support_pose_(const JointState &measured, const VecDoF &gravity)
{
    if (measured.pos.allFinite() && measured.vel.allFinite() && gravity.allFinite() &&
        std::isfinite(measured.gripper_pos) && measured.gripper_pos >= -0.005 &&
        measured.gripper_pos <= robot_config_.gripper_width + 0.005) {
        JointState pose = measured;
        for (int i = 0; i < robot_config_.joint_dof; ++i)
            pose.pos[i] = std::max(robot_config_.joint_pos_min[i],
                std::min(robot_config_.joint_pos_max[i], pose.pos[i]));
        pose.vel.setZero();
        pose.torque = gravity;
        pose.gripper_pos = std::max(0., std::min(robot_config_.gripper_width, pose.gripper_pos));
        pose.gripper_vel = pose.gripper_torque = 0.;
        for (int i = 0; i < robot_config_.joint_dof; ++i)
            pose.torque[i] = std::max(-command_torque_limit_(i),
                std::min(command_torque_limit_(i), pose.torque[i]));
        cached_support_valid_ = true;
        guard_hold_pose_ = pose;
        guard_hold_gain_.kp = controller_config_.default_kp * 0.3;
        for (int i = 0; i < robot_config_.joint_dof; ++i)
            guard_hold_gain_.kd[i] = std::min(controller_config_.default_kd[i] + 1., 4.5);
        guard_hold_gain_.gripper_kp = gain_.gripper_kp;
        guard_hold_gain_.gripper_kd = gain_.gripper_kd;
    }
}

void Arx5ControllerBase::latch_guard_hold_()
{
    if (emergency_latched_ || !fault_hold_.fault()) return;
    emergency_reason_ = "external_guard_fault";
    emergency_joint_ = -1;
    emergency_value_ = 0;
    emergency_time_ns_ = workbench::FaultHold::now();
    emergency_feedback_valid_ = false;
    valid_guard_hold_ = cached_support_valid_;
    if (native_home_.status == "running") native_home_.status = "failed";
    emergency_latched_ = true;
}
'''

HOLD_OUTPUT = '''
    if (valid_guard_hold_) {
        // Frozen finite measured pose and gravity. Never integrate stale feedback.
        std::lock_guard<std::recursive_mutex> references(reference_mutex_);
        std::lock_guard<std::mutex> commands(cmd_mutex_);
        output_joint_cmd_ = guard_hold_pose_;
        output_joint_cmd_.timestamp = get_timestamp();
        gain_ = guard_hold_gain_;
        interpolator_.init_fixed(output_joint_cmd_);
        publish_control_observation_(guard_hold_pose_);
        return;
    }
'''


def patch(source):
    source = Path(source)
    changes = {
        'include/app/controller_base.h': [
            ('#include "utils.h"', '#include "utils.h"\n#include "workbench_fault_hold.h"'),
            ('    EmergencyReport get_emergency_state();',
             '    EmergencyReport get_emergency_state();\n    void bind_fault_hold(int fd, int side);'),
            ('    void update_emergency_cmd_();', '''    void update_emergency_cmd_();
    workbench::FaultHold fault_hold_;
    bool valid_guard_hold_ = false, cached_support_valid_ = false;
    JointState guard_hold_pose_{robot_config_.joint_dof};
    Gain guard_hold_gain_{robot_config_.joint_dof};
    void latch_guard_hold_();
    void cache_support_pose_(const JointState &measured, const VecDoF &gravity);'''),
            ('    uint64_t latched_monotonic_ns;', '    uint64_t latched_monotonic_ns;\n    std::string support_mode;'),
        ],
        'src/app/controller_base.cpp': [
            ('    if (quiescing_) throw std::runtime_error("SDK quiesced: command rejected");\n    if (emergency_latched_)',
             '    if (quiescing_) throw std::runtime_error("SDK quiesced: command rejected");\n    latch_guard_hold_();\n    if (emergency_latched_)'),
            ('void Arx5ControllerBase::set_to_damping()\n{', HOLD_METHODS + '\nvoid Arx5ControllerBase::set_to_damping()\n{'),
            ('    return {WORKBENCH_SDK_BUILD_ID, instance_id_, emergency_reason_, emergency_latched_.load(),',
             '    latch_guard_hold_();\n    return {WORKBENCH_SDK_BUILD_ID, instance_id_, emergency_reason_, emergency_latched_.load(),'),
            ('            emergency_value_, emergency_time_ns_};',
             '            emergency_value_, emergency_time_ns_, valid_guard_hold_ ? "cached_position_hold" : emergency_feedback_valid_ ? "damping" : "unavailable"};'),
            ('        emergency_feedback_valid_ = valid;',
             '        emergency_feedback_valid_ = valid;\n        valid_guard_hold_ = !valid && cached_support_valid_;\n        fault_hold_.trip(4);'),
            ('    if (native_home_.status == "running") native_home_.status = "cancelled";',
             '    latch_guard_hold_();\n    if (native_home_.status == "running") native_home_.status = "cancelled";'),
            ('void Arx5ControllerBase::update_emergency_cmd_()\n{',
             'void Arx5ControllerBase::update_emergency_cmd_()\n{' + HOLD_OUTPUT),
            ('    if (emergency_latched_) update_emergency_cmd_();',
             '    latch_guard_hold_();\n    if (emergency_latched_) update_emergency_cmd_();'),
            ('''    if (controller_config_.gravity_compensation)
    {
        output_joint_cmd_.torque += gravity_torque_(joint_state_.pos);
    }''', '''    VecDoF support_gravity = VecDoF::Zero(robot_config_.joint_dof);
    if (controller_config_.gravity_compensation)
    {
        support_gravity = gravity_torque_(joint_state_.pos);
        output_joint_cmd_.torque += support_gravity;
    }
    cache_support_pose_(joint_state_, support_gravity);'''),
            ('    if (emergency_latched_) emergency_damping_committed_ = true;',
             '    if (emergency_latched_) emergency_damping_committed_ = true;\n    if (valid_guard_hold_) fault_hold_.committed();'),
        ],
        'src/app/joint_controller.cpp': [
            ('    if (emergency_latched_) throw NativeEmergencyLatched(emergency_reason_);',
             '    if (fault_hold_.fault()) throw NativeEmergencyLatched("external_guard_fault");\n    if (emergency_latched_) throw NativeEmergencyLatched(emergency_reason_);'),
        ],
        'python/arx5_pybind.cpp': [
            ('    m.attr("background_home_supported") = true;',
             '    m.attr("background_home_supported") = true;\n    m.attr("fault_hold_schema") = "workbench.native-fault-hold.v1";'),
            ('        .def_readonly("latched_monotonic_ns", &EmergencyReport::latched_monotonic_ns);',
             '        .def_readonly("latched_monotonic_ns", &EmergencyReport::latched_monotonic_ns)\n        .def_readonly("support_mode", &EmergencyReport::support_mode);'),
            ('        .def("get_control_observation", &Arx5JointController::get_control_observation)',
             '        .def("bind_fault_hold", &Arx5JointController::bind_fault_hold, py::call_guard<py::gil_scoped_release>())\n        .def("get_control_observation", &Arx5JointController::get_control_observation)'),
        ],
    }
    results = {}
    for name, replacements in changes.items():
        text = (source / name).read_text()
        if 'fault_hold_' in text or 'fault_hold_schema' in text:
            raise ValueError('SDK source already changed; use an unmodified candidate: ' + name)
        for old, new in replacements:
            count = text.count(old)
            expected = 2 if name.endswith('joint_controller.cpp') else 1
            if count != expected:
                raise ValueError(f'SDK source boundary differs: {name}: {old!r}: {count}')
            text = text.replace(old, new)
        results[name] = text
    for name, text in results.items():
        (source / name).write_text(text)
    header = Path(__file__).resolve().parents[1] / 'native/workbench_fault_hold.h'
    (source / 'include/workbench_fault_hold.h').write_bytes(header.read_bytes())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    patch(parser.parse_args().source)
