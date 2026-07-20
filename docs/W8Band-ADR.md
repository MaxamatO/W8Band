# W8Band Architecture Decision Record

do inzynierki:
Przeniesc obliczenia do MCU, przygotowac pod to pipeline.
Nastepnie przeniesc to z freeRTOS na Zephyr, zeby wykorzystac optymalizacje oferowana przez RTOS

## Overall Architecture ideas - in progress

1. Gyro bias:
    Average of readings over time for each axis - dont really need it? SFLP gives quaternions with bias taken into account

2. Accell bias:
    Average of readings - already done, saved to datacontext

3. obrot do ukladu globalnego akceleracji:

    3.1 odczyt quat - done

    3.2 odczyt Accell - done

    3.3 rotacja accell do ukladu globalnego za pomoca quat - TBD (for now we have prepared structures for rotation)
        Rotation will be done isnide CalibrationDone phase - normalise quaternion, rotate accel local body to world body

    3.4 odjecie a_global - g_global

4. Buffering state

    4.1 Save data from IMU using cyclic buffer, to not miss any data when entering RecordingState on WAKE UP interrupt from IMU, since it happens IN MOTION, not before

5. Armed state

    5.1 Drop all data from cyclic buffer into end data and enter Recording State on WU_INT

6. Recording State

    6.1 Record all data untill device is STILL again - reuse already working WaitingForStillness? By chcecking that, we know when to end recording
        We can also add a TIMEOUT in order to cut off recording after it takes too long - in case of a bug? Or user waiting too long in their execrice?

    6.2 When we can determien that we are stopped again, we TRANSITION INTO ProcessingState

7. Processing State

    7.1 We will need to preprocess data, take bias into account, determine candidate points for ZUPT, split processing into phases - CONCENTRIC, TURNAROUND, ECCENTRIC, REST, so we can calculate trajectory easier without that much drift

    7.2 What algorithms to use for proper trajectory calculation? Double integration for place obviously, but before that should we use other filtering algorithms?  Kalmann Filter - EKF? Some low-pass filter for noise? What more?

    7.3 Maybe download data locally/flash it and process it with python algorithms in the same way we would for C++? Record test cases, dump data from them, then adjust algorithm in python for real trajectory with video as a reference. After its decent, move it into c++ - easier debugging?

8. Sending data

    8.1 We want to send only calculated data with BLE, debugging will be shit

9. Before that, we need to also prepare our BLE manager to be able to connect with application - that needs to be done

10. Create an application

11. Create custom PCB

12. Move into Zephyr - optimise battery usage, put Device into sleep, wake it up on BLE connected?
