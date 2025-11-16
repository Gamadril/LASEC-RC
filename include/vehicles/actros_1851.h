#pragma once

#include "../sounds/actros_1851/air_brake.h"
#include "../sounds/actros_1851/blinker.h"
#include "../sounds/actros_1851/coupling.h"
#include "../sounds/actros_1851/decoupling.h"
#include "../sounds/actros_1851/engine_fan.h"
#include "../sounds/actros_1851/engine_idle.h"
#include "../sounds/actros_1851/engine_rev.h"
#include "../sounds/actros_1851/engine_start.h"
#include "../sounds/actros_1851/engine_stop.h"
#include "../sounds/actros_1851/engine_turbo.h"
#include "../sounds/actros_1851/gear_shift.h"
#include "../sounds/actros_1851/horn.h"
#include "../sounds/actros_1851/jake_brake.h"
#include "../sounds/actros_1851/parking_brake.h"
#include "../sounds/actros_1851/reverse.h"
#include "../vehicle.h"

const Vehicle ACTROS_1851 = {
    .name = {'A', 'c', 't', 'r', 'o', 's', ' ', '1', '8', '5', '1', '\0'},
    .startVolume = 70,
    .idleVolume = 80,
    .engineIdleVolume = 50,
    .fullThrottleVolume = 140,
    .revSoundEnabled = true,
    .revVolume = 130,
    .engineRevVolume = 50,
    .revSwitchPoint = 100,
    .idleEndPoint = 400,
    .idleVolumeProportion = 90,
    .jakeBrakeEnabled = true,
    .jakeBrakeVolume = 200,
    .jakeBrakeIdleVolume = 0,
    .jakeBrakeMinRpm = 200,
    .dieselKnockVolume = 130,
    .dieselKnockIdleVolume = 10,
    .dieselKnockInterval = 6,
    .dieselKnockStartPoint = 110,
    .dieselKnockAdaptiveVolume = 50,
    .dieselKnockPulses = 0,
    .dieselKnockDependsOnRPM = true,
    .dieselKnockCylinders = {0, 0, 0, 0},
    .turboEnabled = true,
    .turboVolume = 30,
    .turboIdleVolume = 0,
    .chargerEnabled = false,
    .chargerVolume = 0,
    .chargerIdleVolume = 10,
    .chargerStartPoint = 10,
    .wastegateEnabled = false,
    .wastegateVolume = 50,
    .wastegateIdleVolume = 1,
    .fanEnabled = false,
    .fanVolume = 0,
    .fanIdleVolume = 0,
    .fanStartPoint = 0,
    .hornVolume = 100,
    .brakeVolume = 150,
    .parkingBrakeVolume = 150,
    .shiftingVolume = 100,
    .reversingVolume = 70,
    .indicatorVolume = 100,
    .indicatorOnThreshold = 300,
    .couplingSoundEnabled = true,
    .couplingVolume = 100,
    .xenonLightsEnabled = false,
    .separateHighBeamEnabled = true,
    .escRampTimeFirstGear = 20,
    .escRampTimeSecondGear = 50,
    .escRampTimeThirdGear = 75,
    .escBrakeSteps = 30,
    .escAccelerationSteps = 3,
    .shiftingAutoThrottle = true,
    .clutchEngagingPoint = 10,
    .maxRpmPercentage = 330,
    .acc = 2,
    .dec = 1,
};
