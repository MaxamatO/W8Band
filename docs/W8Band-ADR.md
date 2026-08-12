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
        Rotation will be done isnide CalibrationDone phase - normalise quaternion, rotate accel local body to world body - done

    3.4 odjecie a_global - g_global - done

4. Buffering state

    4.1 Save data from IMU using cyclic buffer, to not miss any data when entering RecordingState on WAKE UP interrupt from IMU, since it happens IN MOTION, not before - done

    @Additionally we want to detect liftoff using Wake Up interrupt from LSM, but to detect the start of the motion, we shall use already implemented WaitingForStillness, by calculating Variance of acceleration.

    @Idea below has been dropped, because we can combine this state into #bufferring state and split it into 2 phases. And drop pre buffer into #end data onEnter RecordingState

5. Armed state     **FOR REMOVAL**

    5.1 Drop all data from cyclic buffer into end data and enter Recording State on WU_INT - TBD NEXT

6. Recording State - completed

    6.1 Record all data untill device is STILL again - reuse already working WaitingForStillness? By chcecking that, we know when to end recording
        We can also add a TIMEOUT in order to cut off recording after it takes too long - in case of a bug? Or user waiting too long in their execrice?

    6.2 When we can determien that we are stopped again, we TRANSITION INTO ProcessingState

    6.3 Ad1. We determine direction and consecutive stillness by applying these steps:

    1. Obtain current data packet
    2. Get Linear Acceleration of the device
    3. Rotate body to world - in order to integrate provcided by SFLP       Gravity Vector
    4. Integrate real time by using Trapezoid integration in order to obtain velocity
    5. Split Recording state into phases - UP, DONW, NEAR-ZERO VELOCITY
    6. Determine the direction using Schmitt Hysteresis with thresholds **TO BE CALCULATED**
    7. In order to confidently say we have ended our motion, we have to be in NearZero (~0m/s) velocity for DWELL_TIME amount, HAVE BEEN SEEN going up previously, so we only check for end of motion at the eccentric phase.
    8. After completing motion - either by RECORDING_TIMEOUT, or by going end-of-motion, we transition to ProcessingState

7. Processing State

    7.1 We will need to preprocess data, take bias into account, determine candidate points for ZUPT, split processing into phases - CONCENTRIC, TURNAROUND, ECCENTRIC, REST, so we can calculate trajectory easier without that much drift

    7.2 What algorithms to use for proper trajectory calculation? Double integration for place obviously, but before that should we use other filtering algorithms?  Kalmann Filter - EKF? Some low-pass filter for noise? What more?

    7.3 Maybe download data locally/flash it and process it with python algorithms in the same way we would for C++? Record test cases, dump data from them, then adjust algorithm in python for real trajectory with video as a reference. After its decent, move it into c++ - easier debugging?

8. Sending data

    8.1 We want to send only calculated data with BLE, debugging will be shit

9. Before that, we need to also prepare our BLE manager to be able to connect with application - that needs to be done

    9.1 Connecting

    9.2 Recieving data and proper handling related to data recieved

    9.3 Need to handle multiple options of sending BLE data.

10. Create an application

11. Create custom PCB

12. Move into Zephyr - optimise battery usage, put Device into sleep, wake it up on BLE connected?

13. Sources and references:

    13.1 Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems, 2nd edition.
    1. Chapter 2 - Coordinate Frames, Kinematics, and the Earth
    2. Chapter 4 - Interial Sensors
    3. Chapter 5 - Intertial Navigation
    4. Chapter 15 - INS Alignment, Zero Updates and Motion Constraints

    13.2 MP Odpowiadając na pytanie, przyczyn może być kilka.

    Zewnętrzny EKF – proszę do testów spróbować z niego zrezygnować.  LSM6DSV16X posiada sprzętowy Sensor Fusion. Kwaterniony z czujnika należy traktować jako gotową i stabilną informację o orientacji. Dodatkowy EKF w Pythonie może wprowadzać niepotrzebny szum i komplikację.
    Usuwanie grawitacji – nie wiem, czy teraz robi to Pan poprawnie, ale przed odjęciem grawitacji należy najpierw obudować przyspieszenie czujnika za pomocą kwaternionu do układu globalnego, a dopiero potem należy odjąć grawitację. Jeśli kwaternion ma minimalne opóźnienie lub błąd, grawitacja zamiast być odejmowana od osi pionowej, częściowo odejmuje się od osi poziomych i tym samym generuje spore przyspieszenie.
    Zastąpienie RTS - proszę spróbować algorytmu Linear Detrending
    Proszę spróbować wykorzystać fazy spoczynku oraz TURNAROUND jako punktów referencyjnych (v=0). Za ich pomocą podzielić jedno powtórzenie na niezależne odcinki czasu.

PROBLEMY:
    Transfer z BUFFERRING LIFT OFF DETECTED, do RecordingState dzieje sie zbyt szybko. Pierw powinno zostac ustawione Barbell is still, waiting for motion, dopiero przy wiekszym ruchu, powinien byc przeskok do RecordingState.
