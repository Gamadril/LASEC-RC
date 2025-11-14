#pragma once

#define MIN_RPM 0
#define MAX_RPM 500

struct __attribute__((packed)) Vehicle {
  // Vehicle name
  char name[20];
  // engine start volume in %
  uint16_t startVolume;
  // Adjust the idle volume (usually = 100%, more also working, depending on sound, 50 - 60% if
  // additional diesel knock sound is used)
  uint16_t idleVolume;
  // the engine volume will be throttle dependent (usually = 40%, never more than 100%!)
  uint16_t engineIdleVolume;
  // full throttle volume in % (for rev sound as well)
  uint16_t fullThrottleVolume;
  // Optional motor revving sound, recorded at medium rpm. Note, that it needs to represent the same number of ignition cycles as the
  // idle sound. For example 4 or 8 for a V8 engine. It also needs to have about the same length. In order to adjust the length
  // or "revSampleCount", change the "Rate" setting in Audacity until it is about the same.
  bool revSoundEnabled;
  // Adjust the idle volume (usually = 100%, more also working, depending on sound)
  uint16_t revVolume;
  // the engine volume will be throttle dependent (usually = 40%, never more than 100%!)
  uint16_t engineRevVolume;
  // The rev sound is played instead of the idle sound above this point
  uint16_t revSwitchPoint;
  // above this point, we have 100% rev and 0% idle sound volume (usually 500, min. 50 more than
  // revSwitchPoint)
  uint16_t idleEndPoint;
  // The idle sound volume proportion (rest is rev proportion) below "revSwitchPoint" (about 90 -
  // 100%, never more than 100)
  uint16_t idleVolumeProportion;
  // set to true if you want to use the jake brake sound
  bool jakeBrakeEnabled;
  // Adjust the max. volume (usually = 150%)
  uint16_t jakeBrakeVolume;
  // Adjust the min. volume (usually = 80%)
  uint16_t jakeBrakeIdleVolume;
  // Adjust the min. RPM for the jake brake (around 100)
  uint16_t jakeBrakeMinRpm;
  // Adjust the Diesel knock volume (usually = 200 - 600%)
  uint16_t dieselKnockVolume;
  // Diesel knock volume while idling (usually = 20%)
  uint16_t dieselKnockIdleVolume;
  // Idle sample length divided by this number (1 - 20, depending on sound files)
  uint16_t dieselKnockInterval;
  // Volume will raise above this point ( usually 0, for "open pipe" exhaust about 250)
  uint16_t dieselKnockStartPoint;
  // Adjust the Diesel knock volume for the non-first knocks per engine cycle (usually = 50%)
  uint16_t dieselKnockAdaptiveVolume;
  // Number of knock pulses defined
  uint8_t dieselKnockPulses;
  // Diesel knock also depends on RPM
  bool dieselKnockDependsOnRPM;
  // engine cylinders which should knock
  uint8_t dieselKnockCylinders[4];
  // set to true to enable turbo sound
  bool turboEnabled;
  // Adjust the turbo volume (usually = 70%)
  uint16_t turboVolume;
  // the turbo volume will be engine rpm dependent (usually = 10%)
  uint16_t turboIdleVolume;
  // set tot true to enable additional supercharger sound
  bool chargerEnabled;
  // Adjust the supercharger volume (usually = 70%)
  uint16_t chargerVolume;
  // the supercharger volume will be engine rpm dependent (usually = 10%)
  uint16_t chargerIdleVolume;
  // Volume will raise above this point ( usually 10)
  uint8_t chargerStartPoint;
  // additional turbo wastegate / blowoff valve sound
  bool wastegateEnabled;
  // Adjust the wastegate volume (usually = 70%, up to 250%)
  uint16_t wastegateVolume;
  // Wastegate sound is played, after rapid throttle drop with engaged clutch
  uint16_t wastegateIdleVolume;
  // additional cooling fan sound
  bool fanEnabled;
  // Adjust the fan volume (250% for Tatra 813, else 0%)
  uint16_t fanVolume;
  // the fan volume will be engine rpm dependent (usually = 10%)
  uint16_t fanIdleVolume;
  // Volume will raise above this point (250 for Tatra 813)
  uint16_t fanStartPoint;
  // Adjust the horn volume (usually = 100%)
  uint16_t hornVolume;
  // Adjust the brake volume (usually = 200%)
  uint16_t brakeVolume;
  // Adjust the brake volume (usually = 200%)
  uint16_t parkingBrakeVolume;
  // Adjust the shifting volume (usually = 200%)
  uint16_t shiftingVolume;
  // Adjust the reversing sound volume (usually = 70%)
  uint16_t reversingVolume;
  // Adjust the indicator sound volume (usually = 100%)
  uint16_t indicatorVolume;
  // The indicator will be switched on above +/- this value, if wheels are turned
  uint16_t indicatorOnThreshold;
  // set to true to enable trailer couplig & uncoupling sound
  bool couplingSoundEnabled;
  // Adjust the volume (usually = 100%)
  uint16_t couplingVolume;
  // set to true to show a xenon bulb ignition flash
  bool xenonLightsEnabled;
  // set to true if separate full beam light is connected
  bool separateHighBeamEnabled;
  // determines, how fast the acceleration and deceleration happens (about 15 - 25, 20 for King Hauler)
  uint8_t escRampTimeFirstGear;
  // 50 for King Hauler (this value is always in use for automatic transmission, about 80)
  uint8_t escRampTimeSecondGear;
  // 75 for King Hauler
  uint8_t escRampTimeThirdGear;
  // determines, how fast the ESC is able to brake down (20 - 30, 30 for King Hauler)
  uint8_t escBrakeSteps;
  // determines, how fast the ESC is able to accelerate (2 - 3, 3 for King Hauler)
  uint8_t escAccelerationSteps;
  // For Tamiya 3 speed tansmission, throttle is altered for synchronizing, if "true"
  bool shiftingAutoThrottle;
  // CEP. The "clutch" is engaging above this point = engine rpm sound in synch with ESC power
  uint16_t clutchEngagingPoint;
  // Engine max. RPM in % of idle RPM. About 200% for big Diesels, 400% for fast running motors.
  uint16_t maxRpmPercentage;
  // Engine mass simulation
  // Acceleration step (2) 1 = slow for locomotive engine, 9 = fast for trophy truck
  uint8_t acc;
  // Deceleration step (1) 1 = slow for locomotive engine, 5 = fast for trophy truck
  uint8_t dec;
};
