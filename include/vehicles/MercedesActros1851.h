#pragma once

#include "../vehicle.h"
#include "sounds/Actros1863JakeBrake.h"
// #include "sounds/Actros1863idle2.h"
#include "sounds/Actros1863knock3.h"
#include "sounds/Actros1863rev.h"
#include "sounds/Actros1863start.h"
#include "sounds/ClunkingGearShifting.h"
#include "sounds/GenericFan.h"
#include "sounds/ManTgeHorn.h"
#include "sounds/ParkingBrake.h"
#include "sounds/TruckAirBrakes2.h"
#include "sounds/TruckReversingBeep.h"
#include "sounds/TurboWhistle.h"
#include "sounds/UnimogU1000TurboWastegate.h"
#include "sounds/coupling.h"
#include "sounds/door.h"
#include "sounds/mb_idle1.h"
#include "sounds/mb_indicator.h"
#include "sounds/supercharger.h"
#include "sounds/uncoupling.h"

const Vehicle ACTROS_1851 = {
    .name = {'A', 'c', 't', 'r', 'o', 's', ' ', '1', '8', '5', '1', '\0'},
    .startVolume = 130,
    .idleVolume = 70,
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
    .dieselKnockCylinders = {0},
    .turboEnabled = true,
    .turboVolume = 30,
    .turboIdleVolume = 0,
    .chargerEnabled = false,
    .chargerVolume = 0,
    .chargerIdleVolume = 10,
    .chargerStartPoint = 10,
    .wastegateEnabled = true,
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
    .couplingSoundEnabled = false,
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
