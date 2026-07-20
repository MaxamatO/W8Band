# W8Band Documentation

## Description

W8Band is a device for powerlifters, coaches and people, that want to perfect their bar path.
It consists of a user friendly mobile application with dedicated W8Band physical device.
It connects to your phone via Bluetooth Low Energy, and lets you track your path trajectory after completed repetiton.

## How to use

lorem ipsum

### 1. Connect your application

lorem ipsum

## State Machine

### **CalibrationState**

StateMachine's State responsible for handling proper device calibration.
Triggerred by Client by sending "Calibrate" command.<br>
CalibrationState is split into **CalibrationPhases**:

- WaitingForStillness - Device caluclates variance of both acceleration and gyroscope deciding if device is still.
- Accumulating - Device calculates acceleration mean in all 3 axes, and saves it into DataContext as bias. If during that phase, Device stops being still, we go back into WaitingForStillness phase, reseting **BiasAccumulator**

enum class CalibrationPhase
{
    WaitingForStillness,
    Accumulating,
    Done,
    Failed
};
