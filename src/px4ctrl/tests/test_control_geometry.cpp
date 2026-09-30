#include "control_geometry.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    uadl::ThrustConfig config;
    std::array<Eigen::Vector2d,3> limits{{{-3,3},{-3,3},{-2,5}}};
    const Eigen::Vector3d gravity(0,0,config.gravity);
    auto stationary = uadl::specificForceToNetAcceleration(gravity, Eigen::Quaterniond::Identity(), config.gravity);
    assert(stationary.norm() < 1e-12);
    const Eigen::Quaterniond tilted(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()));
    stationary = uadl::specificForceToNetAcceleration(tilted.inverse()*gravity, tilted, config.gravity);
    assert(stationary.norm() < 1e-12);
    auto hover = uadl::mapAcceleration(Eigen::Vector3d::Zero(), 1.2, 16, config, limits);
    assert(hover.valid && std::abs(hover.thrust-config.hover_fraction)<1e-12);
    assert(hover.executed_input.norm()<1e-12);
    assert((hover.attitude*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm()<1e-12);
    for (int bits=0;bits<8;++bits) {
        Eigen::Vector3d input;
        for(int i=0;i<3;++i) input(i)=limits[i]((bits>>i)&1);
        auto out=uadl::mapAcceleration(input,-0.7,16,config,limits);
        assert(out.valid && (out.executed_input-input).norm()<1e-10);
        assert(out.thrust>=0 && out.thrust<=1);
        auto reconstructed=(config.gravity/config.hover_fraction)*out.thrust*(out.attitude*Eigen::Vector3d::UnitZ())-gravity;
        assert((reconstructed-out.executed_input).norm()<1e-10);
    }
    auto saturated=uadl::mapAcceleration(Eigen::Vector3d(100,-100,100),0,16,config,limits);
    assert(saturated.valid && (saturated.executed_input-Eigen::Vector3d(3,-3,5)).norm()<1e-10);
    config.max_tilt=0.01;
    auto tilt_limited=uadl::mapAcceleration(Eigen::Vector3d(3,0,0),0,16,config,limits);
    assert(tilt_limited.valid && tilt_limited.executed_input.x()<0.11);
    config.max_tilt=1.396;
    config.voltage_model=true;
    config.k3=0.3;
    auto voltage=uadl::mapAcceleration(Eigen::Vector3d(1,2,1),0.5,16,config,limits);
    assert(voltage.valid && (voltage.executed_input-Eigen::Vector3d(1,2,1)).norm()<1e-10);
    assert(!uadl::mapAcceleration(Eigen::Vector3d::Zero(),0,0,config,limits).valid);
    assert(!uadl::mapAcceleration(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),0,16,config,limits).valid);
    Eigen::Quaterniond zero(0,0,0,0);
    assert(!uadl::specificForceToNetAcceleration(gravity,zero,9.81).allFinite());
    std::cout << "control geometry tests passed\n";
}
