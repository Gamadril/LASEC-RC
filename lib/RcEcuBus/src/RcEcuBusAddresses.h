#pragma once

// Board Address Definitions for RC Vehicle Lighting System
// Addresses must be unique 7-bit I2C addresses (0x08 to 0x77)

// Front Light Controller Boards
#define FRONT_LEFT_LIGHTS_ADDRESS     0x20    // Front left lights (headlights, turn signals)
#define FRONT_RIGHT_LIGHTS_ADDRESS    0x21    // Front right lights (headlights, turn signals)
#define FRONT_CENTER_LIGHTS_ADDRESS   0x22    // Front center lights (fog lights, etc.)

// Rear Light Controller Boards
#define REAR_LEFT_LIGHTS_ADDRESS      0x23    // Rear left lights (tail lights, turn signals, brake)
#define REAR_RIGHT_LIGHTS_ADDRESS     0x24    // Rear right lights (tail lights, turn signals, brake)
#define REAR_CENTER_LIGHTS_ADDRESS    0x25    // Rear center lights (reverse lights, etc.)

// Side Light Controller Boards
#define LEFT_SIDE_LIGHTS_ADDRESS      0x26    // Left side marker lights
#define RIGHT_SIDE_LIGHTS_ADDRESS     0x27    // Right side marker lights

// Roof/Top Light Controller Boards
#define ROOF_LIGHTS_ADDRESS           0x28    // Roof lights, beacons, etc.

// Interior Light Controller Boards
#define INTERIOR_LIGHTS_ADDRESS       0x29    // Interior/cabin lights

// Utility Addresses
#define MAIN_CONTROLLER_ADDRESS       0x01    // Main ESP32 controller (if needed)

// Address ranges for different board types
#define FRONT_BOARD_START_ADDRESS     0x20
#define FRONT_BOARD_END_ADDRESS       0x22
#define REAR_BOARD_START_ADDRESS      0x23
#define REAR_BOARD_END_ADDRESS        0x25
#define SIDE_BOARD_START_ADDRESS      0x26
#define SIDE_BOARD_END_ADDRESS        0x27

// Helper macros for address validation
#define IS_VALID_BOARD_ADDRESS(addr) (addr >= 0x08 && addr <= 0x77)
#define IS_FRONT_BOARD_ADDRESS(addr) (addr >= FRONT_BOARD_START_ADDRESS && addr <= FRONT_BOARD_END_ADDRESS)
#define IS_REAR_BOARD_ADDRESS(addr)  (addr >= REAR_BOARD_START_ADDRESS && addr <= REAR_BOARD_END_ADDRESS)
