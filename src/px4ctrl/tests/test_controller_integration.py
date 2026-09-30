#!/usr/bin/env python3
"""Compile and test the real controller adapter without ROS or Torch.

All generated headers/binaries live in a temporary directory. The C++ fixture
is an exact nominal plant (configuration C) with a zero residual budget, which
makes every pairing, gating and transaction decision deterministic. The shim
replaces transport/message types, not controller or core algorithms.
"""

import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import textwrap


ROS_SHIM = r"""
#pragma once
#include <iostream>
#include <stdexcept>
#include <string>
namespace ros {
inline double fixture_now = 100.0;
class Time {
    double seconds_ = 0.0;
public:
    Time() = default;
    explicit Time(double value) : seconds_(value) {}
    double toSec() const { return seconds_; }
    static Time now() { return Time(fixture_now); }
};
class NodeHandle {
public:
    template<class K, class V> bool getParam(const K&, V&) const { return false; }
};
}
#define ROS_ERROR_STREAM(value) do { std::cerr << value << '\n'; } while (false)
#define ROS_BREAK() throw std::runtime_error("unexpected ROS_BREAK in adapter fixture")
"""

INPUT_SHIM = r"""
#pragma once
#define __INPUT_H
#include <Eigen/Geometry>
#include <cmath>
#include "PX4CtrlParam.h"
#include "quadrotor_msgs/Px4ctrlDebug.h"
struct FixtureMessage { quadrotor_msgs::FixtureHeader header; };
struct Odom_Data_t {
    Eigen::Vector3d p = Eigen::Vector3d::Zero(), v = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    FixtureMessage msg;
    ros::Time rcv_stamp;
    bool received = true;
};
struct Imu_Data_t {
    Eigen::Vector3d a = Eigen::Vector3d(0.0, 0.0, 9.81);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    FixtureMessage msg;
    ros::Time rcv_stamp;
    bool received = true;
};
namespace uav_utils {
inline double get_yaw_from_quaternion(const Eigen::Quaterniond& q) {
    return std::atan2(2.0 * (q.w()*q.z() + q.x()*q.y()),
                      1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z()));
}
}
"""

TEST_CPP = r"""
#include "controller.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

// The production ROS parameter loader is deliberately not replaced by guessed
// defaults. This constructor only lets the fixture explicitly set every field
// consumed by the REAL Controller::configure().
Parameter_t::Parameter_t() {}

namespace {
using Clock = std::chrono::steady_clock;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, const std::string& message, double tolerance=1e-8) {
    require(std::isfinite(actual) && std::abs(actual-expected) <= tolerance,
            message + ": got " + std::to_string(actual));
}
struct Fixture {
    Parameter_t parameters;
    std::unique_ptr<Controller> controller;
    Odom_Data_t odom;
    Imu_Data_t imu;
    Desired_State_t reference;
    Controller_Output_t output;
    explicit Fixture(double start=100.0) {
        ros::fixture_now = start;
        auto& p = parameters;
        p.rc_reverse = {}; p.takeoff_land = {};
        p.msg_timeout = {0.1, 0.1, 0.1, 0.1, 0.1};
        p.thr_map = {};
        p.method = "C";
        p.mass = 1.233; p.gra = 9.81; p.max_angle = 0.7;
        p.ctrl_freq_max = 100.0; p.low_voltage = 14.0;
        p.thr_map.hover_percentage = 0.5;
        p.thr_map.accurate_thrust_model = false;
        p.thr_map.K1 = 2.1; p.thr_map.K2 = 1.0; p.thr_map.K3 = 0.0;
        p.analysis_lower.setConstant(-10.0); p.analysis_upper.setConstant(10.0);
        p.solver_cutoff = 0.0085; p.control_deadline = 0.01;
        p.sensor_max_skew = 0.002; p.input_delay = 0.0; p.command_max_age = 0.03;
        for (int axis=0; axis<3; ++axis) {
            auto& b = p.bounds[axis];
            // Exact nominal test plant: zero residual budget on every axis.
            b.rkhs_norm = b.rkhs_norm_nominal = 0.0;
            b.disturbance = b.measurement = b.state_error = 0.0;
            b.synchronization = b.command_modification = b.hold_error = 0.0;
            b.lipschitz_f = b.lipschitz_g = b.prior_abs_f = 0.0;
            b.prior_min_g = b.prior_max_g = 1.0;
            b.reference_acceleration = 0.2;
            b.max_envelope = b.min_envelope = b.fixed_envelope = 0.0;
            p.gp[axis].rkhs_bound = 0.0;
            p.mpc[axis].max_envelope = p.mpc[axis].min_envelope = 0.0;
            p.mpc[axis].correction_domain = Eigen::Vector2d(-2.5, 2.5);
            p.physical_input_limits[axis] = Eigen::Vector2d(-3.0, 3.0);
        }
        p.physical_input_limits[2] = Eigen::Vector2d(-2.0, 5.0);
        p.mpc[2].Q_diag = Eigen::Vector2d(15.0, 2.0); p.mpc[2].R = 0.5;
        p.mpc[2].Q_anc_diag = Eigen::Vector2d(30.0, 5.0);
        p.mpc[2].state_limits = Eigen::Vector2d(1.5, 1.0);
        p.mpc[2].correction_domain = Eigen::Vector2d(-1.5, 4.5);
        controller.reset(new Controller(p));
        require(controller->ready(), "fixture configuration: " + controller->lastStatus());
        source(start, start);
    }
    void source(double now, double stamp, double imu_stamp=-1.0) {
        ros::fixture_now = now;
        odom.msg.header.stamp = ros::Time(stamp);
        imu.msg.header.stamp = ros::Time(imu_stamp < 0.0 ? stamp : imu_stamp);
        odom.rcv_stamp = imu.rcv_stamp = ros::Time(now);
    }
    void cycle(bool active=true) {
        controller->beginCycle(Clock::now());
        controller->update(reference, odom, imu, output, 16.0, active);
    }
    void validCycle() {
        cycle();
        require(output.valid, "controller output: " + controller->lastStatus());
        require(controller->publicationAllowed(), "unexpected test-machine deadline overrun");
    }
    void publish() { controller->commitPublished(output, ros::Time::now()); }
    void window(unsigned expected) {
        for (int axis=0; axis<3; ++axis)
            require(controller->debug.gp_window_size[axis] == expected,
                    "GP window: expected " + std::to_string(expected) + ", got " +
                    std::to_string(controller->debug.gp_window_size[axis]));
    }
};

void hover_and_publication_transaction() {
    Fixture f;
    f.cycle(false);
    require(f.output.valid && !f.output.adaptive_command, "inactive prestream marked adaptive");
    near(f.output.thrust, 0.0, "inactive prestream thrust");
    f.publish();
    f.source(100.01, 100.01);
    f.validCycle();
    near(f.output.thrust, 0.5, "calibrated hover thrust");
    near(f.output.q.angularDistance(Eigen::Quaterniond::Identity()), 0.0, "upright hover");
    near(f.controller->debug.fb_a_z, 0.0, "specific force gravity conversion");
    f.window(0);
    require(!f.controller->debug.gp_sample_inserted, "first cycle learned unexecuted control");
    f.controller->discardPending();
    f.source(100.02, 100.02);
    f.validCycle();
    f.window(0); // Neither inactive nor discarded output became command history.
    f.publish();
    f.source(100.03, 100.03);
    f.validCycle();
    f.window(1);
    require(f.controller->debug.gp_sample_inserted, "published command did not permit sampling");
    f.controller->discardPending();
    f.source(100.04, 100.04);
    f.validCycle();
    f.window(1); // Candidate at 100.03 was discarded, not committed.
    f.publish();
    f.source(100.05, 100.05);
    f.validCycle();
    f.window(2);
    f.publish();
    f.source(100.055, 100.05);
    f.validCycle();
    f.window(2);
    require(!f.controller->debug.gp_sample_inserted, "duplicate source timestamp inserted twice");
    f.publish();
}

void actual_executed_input_and_sensor_alignment() {
    Fixture f(200.0);
    f.reference.a.x() = 0.1;
    f.validCycle();
    near(f.output.executed_input.x(), 0.1, "fixture's first executed acceleration");
    f.window(0);
    f.publish();
    f.source(200.01, 200.01);
    f.odom.p.x() = 0.5 * 0.1 * 0.01 * 0.01;
    f.odom.v.x() = 0.1 * 0.01;
    f.reference.p = f.odom.p; f.reference.v = f.odom.v;
    f.reference.a.setZero();
    f.imu.a.x() = 0.1;
    f.validCycle();
    f.window(1);
    // The new action is zero, while the observed acceleration is 0.1. With a
    // zero-error test certificate, pairing with the new rather than actually
    // published previous action would cause immediate residual rejection.
    near(f.output.executed_input.x(), 0.0, "current action differs from paired action");
    f.publish();
    f.source(200.02, 200.02, 200.01);
    f.imu.a.x() = 0.0;
    f.odom.p.x() += f.odom.v.x() * 0.01;
    f.reference.p = f.odom.p;
    f.validCycle();
    f.window(1);
    require(!f.controller->debug.gp_sample_inserted, "unsynchronized source data entered GP");
}

void stale_nonfinite_and_old_history() {
    Fixture f(300.0);
    f.validCycle(); f.publish();
    f.source(300.01, 300.01);
    f.validCycle(); f.window(1); f.publish();
    f.source(300.20, 300.01);
    f.cycle();
    require(!f.output.valid, "stale republished source was accepted");
    f.controller->discardPending();
    f.source(300.21, 300.21);
    f.validCycle(); f.window(1);
    require(!f.controller->debug.gp_sample_inserted, "old executed command paired beyond maximum age");
    f.publish();
    f.source(300.22, 300.22);
    f.imu.a.x() = std::numeric_limits<double>::quiet_NaN();
    f.cycle();
    require(!f.output.valid, "NaN acceleration accepted");
    f.controller->discardPending();
    f.imu.a.x() = 0.0;
    f.odom.p.x() = std::numeric_limits<double>::infinity();
    f.cycle();
    require(!f.output.valid, "infinite source state accepted");
}

void source_precedes_publication_and_actuator_delay() {
    Fixture f(350.0);
    f.source(350.0, 349.99);
    f.validCycle(); f.window(0); f.publish();
    // Receipt is newer than publication but SOURCE time is older. It cannot
    // be labeled with a command that had not yet been issued at measurement.
    f.source(350.01, 349.995);
    f.validCycle(); f.window(0);
    require(!f.controller->debug.gp_sample_inserted, "receipt time paired a future published action");
    f.controller->discardPending();
    f.parameters.input_delay = 0.015;
    f.source(350.012, 350.012);
    f.validCycle(); f.window(0);
    require(!f.controller->debug.gp_sample_inserted, "action sampled before configured actuator delay");
    f.controller->discardPending();
    f.source(350.02, 350.02);
    f.validCycle(); f.window(1);
    require(f.controller->debug.gp_sample_inserted, "delayed executed action never became eligible");
}

void future_reset_epoch_and_old_data() {
    Fixture f(400.0);
    f.validCycle(); f.publish();
    f.source(400.01, 400.01);
    f.validCycle(); f.window(1); f.publish();
    require(!f.controller->scheduleReset(400.0), "past reset epoch accepted");
    require(f.controller->scheduleReset(400.04), "future reset refused");
    f.source(400.02, 400.02);
    f.validCycle(); f.window(2); f.publish();
    f.source(400.04, 400.04);
    f.validCycle(); f.window(0); f.publish();
    f.source(400.041, 400.039);
    f.validCycle(); f.window(0);
    require(!f.controller->debug.gp_sample_inserted, "pre-epoch delayed data repopulated reset window");
    f.publish();
    f.source(400.051, 400.051);
    f.validCycle(); f.window(1); f.publish();
    f.cycle(false);
    require(f.output.valid && !f.output.adaptive_command, "mode exit prestream failed");
    f.source(400.061, 400.061);
    f.validCycle(); f.window(0);
}

void reference_switch_backup_and_deadline() {
    Fixture f(500.0);
    f.validCycle(); f.publish();
    f.source(500.01, 500.01);
    f.reference.p.x() = 0.5; // Reference switch outside the retained tube.
    f.validCycle();
    require(!f.output.used_backup, "reference switch was not re-optimized: " + f.controller->lastStatus());
    near(f.controller->debug.des_p_x, 0.5, "new reference was not adopted");
    f.window(1);
    f.controller->discardPending();
    f.reference.p.setZero();
    // A positive cutoff already exhausted by bookkeeping deterministically
    // exercises the solver-cutoff path: the retained plan with its compatible
    // reference continuation supplies the command.
    f.parameters.solver_cutoff = 1e-12;
    f.source(500.02, 500.02);
    f.validCycle();
    require(f.output.used_backup, "solver cutoff did not use the feasible published-plan continuation");
    near(f.controller->debug.des_p_x, 0.0, "backup did not retain its compatible reference");
    f.window(0); // The sample proposal is rolled back with the late candidate.
    f.publish();
    f.parameters.solver_cutoff = 0.0085;
    f.source(500.03, 500.03);
    f.controller->beginCycle(Clock::now() - std::chrono::milliseconds(20));
    f.controller->update(f.reference, f.odom, f.imu, f.output, 16.0, true);
    require(!f.output.valid && !f.controller->publicationAllowed(), "late command passed publication gate");
    f.controller->discardPending();
    f.source(500.04, 500.04);
    f.validCycle(); f.window(1); // Deadline-discarded candidate did not commit.
}
}

int main() {
    try {
        hover_and_publication_transaction();
        actual_executed_input_and_sensor_alignment();
        stale_nonfinite_and_old_history();
        source_precedes_publication_and_actuator_delay();
        future_reset_epoch_and_old_data();
        reference_switch_backup_and_deadline();
        std::cout << "Controller integration tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Controller integration test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
"""


def debug_header(message_path: Path) -> str:
    types = {"float64": "double", "float32": "float", "uint32": "std::uint32_t",
             "bool": "bool", "string": "std::string"}
    fields = []
    for raw in message_path.read_text().splitlines():
        raw = raw.split("#", 1)[0].strip()
        if not raw:
            continue
        kind, name = raw.split()
        if kind == "Header":
            fields.append("FixtureHeader " + name + "{};")
        elif "[" in kind:
            base, length = kind.rstrip("]").split("[")
            fields.append(f"std::array<{types[base]}, {int(length)}> {name}{{}};")
        else:
            fields.append(f"{types[kind]} {name}{{}};")
    return ("#pragma once\n#include <array>\n#include <cstdint>\n#include <string>\n"
            "#include <ros/ros.h>\nnamespace quadrotor_msgs {\n"
            "struct FixtureHeader { ros::Time stamp; };\nstruct Px4ctrlDebug {\n" +
            "\n".join(fields) + "\n};\n}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--eigen", default=os.environ.get("EIGEN3_INCLUDE_DIR"), required=False)
    parser.add_argument("--qpoases-include", default=os.environ.get("QPOASES_INCLUDE_DIR"))
    parser.add_argument("--qpoases-library", default=os.environ.get("QPOASES_LIBRARY"))
    parser.add_argument("--core-library", help="Optional already-built uadl core static library")
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"))
    parser.add_argument("--cxx-flag", action="append", default=[])
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    for name in ("eigen", "qpoases_include", "qpoases_library"):
        if not getattr(args, name):
            parser.error("missing --" + name.replace("_", "-"))
    package = Path(__file__).resolve().parents[1]
    workspace = package.parents[1]
    source = package / "src"
    with tempfile.TemporaryDirectory(prefix="uadl_controller_integration_") as directory:
        build = Path(directory)
        (build / "ros").mkdir()
        (build / "quadrotor_msgs").mkdir()
        (build / "ros/ros.h").write_text(textwrap.dedent(ROS_SHIM))
        (build / "quadrotor_msgs/Px4ctrlDebug.h").write_text(
            debug_header(workspace / "src/quadrotor_msgs/msg/Px4ctrlDebug.msg"))
        shim = build / "input_shim.h"
        shim.write_text(textwrap.dedent(INPUT_SHIM))
        test = build / "test_controller.cpp"
        test.write_text(textwrap.dedent(TEST_CPP))
        binary = build / "test_controller"
        command = [args.cxx, "-std=c++17", "-O1", "-I" + args.eigen,
                   "-I" + args.qpoases_include, "-I" + str(build), "-I" + str(source),
                   "-include", str(shim), str(source / "controller.cpp"), str(test)]
        if args.core_library:
            command.append(args.core_library)
        else:
            command.extend(str(source / name) for name in
                           ("online_gp.cpp", "tube_mpc.cpp", "control_geometry.cpp"))
        command.extend([args.qpoases_library, "-o", str(binary)])
        command.extend(args.cxx_flag)
        if args.sanitize:
            command.extend(["-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer"])
        subprocess.run(command, check=True, timeout=180)
        subprocess.run([str(binary)], check=True, timeout=60)


if __name__ == "__main__":
    main()
