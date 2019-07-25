# CopterSim
A high-fidelity simulation model developed in Simulink that compatible with different types of multicopters. The model can be used to develop control algorithms in Simulink. The simulation model includes sensor data outputs that can be used to generate code to performe hardware-in-the-loop simulations for autopilot systems like Pixhawk/PX4 or Ardupilot. The fault-injection function allows testing the safety and reliability of the control algorithms.

## Contact Info.<br>
Visit our Lab pages to contact us:<br>
http://rfly.buaa.edu.cn/index.html<br>
<br>
A video to present the hardware-in-the-loop simulation project based on this simulation model.<br>
[![https://youtu.be/GIb7JcGcXig](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/videocover.png)](https://youtu.be/GIb7JcGcXig)

<br>

## How to use the files.<br>
1. Open "Multicopter_vPC.slx" file with Matlab 2017b and later. Noteworthy, the Aerospace Blockset is required for MATLAB.<br>
2. Click the "Run" button to run the Simulink model.<br>
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/SimulinkRunandCompile.png)<br>
3. Click the "Compile" button to compile the model to C Code (Visual C++ 2015 or later is required).<br>
4. Generate Code for LabVIEW for hardware-in-the-loop simulations. Configure the Simulink setting page according to the figure below.<br>
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/SimulinkSetting.png)<br>
5. Generate code for embedded system. Change the above "System target file" option to "ert.tlc".<br>


<br>

## File structure.<br>
imgs: images for this Readme.md Tutorial.<br>
Init.m: Initialization script automatically called before running the model File.<br>
MavLinkStruct.mat: the bus Structs for the output and input signals<br>
Multicopter_vPC.slx : the main Simulink model file.<br>
SupportedVehicleTypes.docx :  supported vehicle types.<br>
<br>
<br>

## Input and output Ports.<br>
inPWMs: input signal, ESC/motor control signal from the control system, eight-dimensional float vector, ranges from 0 to 1. <br>
Terrain: input signal, the current terrain height, one-dimensional float value, positive for the down direction, unit (m) <br>
MavHILSensor: output signal, bus struct, contains sensor data required by the Autopilot system like PX4/Ardupilot <br>
MavHILGPS: output signal, bus struct, contains GPS data required by the Autopilot system like PX4/Ardupilot <br>
MavVehileStateInfo:  output signal, bus struct, contains true state of the vehicle for the vehicle software simulation in Simulink <br>
the detailed definition for the above output structs are presented below. <br>
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/SimulinkOutputDefinitions.png)<br>

## Change vehicle types.<br>
The models cover all multicopter airframe for PX4 autopilot?http://dev.px4.io/en/airframes/airframe_reference.html <br>
Modify the parameter "ModelParam_uavType" in Init.m file to change the vehicle types.
The supported vehicle types include:
ModelParam_uavType = 1: Tricopter Y+<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image1.png)<br> 
<br> 
ModelParam_uavType = 2: Tricopter Y-<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image2.png)<br>  
<br> 
ModelParam_uavType = 3:  Quadrotor X <br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image3.png)<br> 
<br> 
ModelParam_uavType = 4:  Quadrotor +<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image4.png)<br>  
<br> 
ModelParam_uavType = 5:  Hexarotor x<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image5.png)<br>  
<br> 
ModelParam_uavType = 6:  Hexarotor +<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image6.png)<br>  
<br> 
ModelParam_uavType = 7: Hexarotor Coaxial<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image7.png)<br> 
<br> 
ModelParam_uavType = 8:  Octorotor x<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image8.png)<br> 
<br> 
ModelParam_uavType = 9:  Octorotor +<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image9.png)<br> 
<br> 
ModelParam_uavType = 10: Octorotor Coaxial<br> 
![image](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/image10.png)<br> 
<br> 


## An online toolbox to quickly obtain the model parameters<br> 
https://flyeval.com<br> 
[![https://flyeval.com](https://raw.githubusercontent.com/XunhuaDai/CopterSim/master/imgs/flyeval.png)](https://flyeval.com)


## Modify model parameters and inject fault during flight.<br>
Change the corresponding parameters in Init.m file

load MavLinkStruct;  % load the bus structs HILGPS MavLinkGPS MavLinkSensor MavVehileInfo<br>
<br>
%Initial condition<br>  %set vehicle initial state.
ModelInit_PosE = [0,0,0]; % Vehicle postion xyz in the NED earth frame (m)<br>
ModelInit_VelB = [0,0,0]; % Vehicle speed xyz in the NED earth frame (m/s)<br>
ModelInit_AngEuler = [0,0,0]; % Vehicle Euler angle xyz (roll,pitch,yaw) (rad)<br>
ModelInit_RateB = [0,0,0]; % Vehicle angular speed xyz (roll,pitch,yaw) in the body frame (rad/s)<br>
ModelInit_RPM = 0; %Initial motor speed (rad/s)<br>
<br>
%UAV model parameter<br>
ModelParam_uavMass = 1.4; %Mass of UAV(kg)<br>
ModelParam_uavJxx = 0.0241;  % moment of inertia in body x axis<br>
ModelParam_uavJyy = 0.0239;   % moment of inertia in body y axis<br>
ModelParam_uavJzz = 0.0386;  % moment of inertia in body z axis<br>
%Moment of inertia matrix<br>
ModelParam_uavJ= [ModelParam_uavJxx,0,0;...<br>
    0,ModelParam_uavJyy,0;...<br>
    0,0,ModelParam_uavJzz];<br>
ModelParam_uavType = int8(3); %X-type quadrotor£¬refer to "SupportedVehicleTypes.docx" for specific definitions<br>
ModelParam_uavMotNumbs = int8(4);  %Number of motors<br>
ModelParam_uavR = 0.225;   %Body radius(m)<br>
<br>
ModelParam_motorCr = 1148; %Motor throttle-speed curve slope(rad/s)<br>
ModelParam_motorWb =-141.4;  %Motor speed-throttle curve constant term(rad/s)<br>
ModelParam_motorT = 0.02;  %Motor inertia time constant(s)<br>
ModelParam_motorJm = 0.0001287;    %Moment of inertia of motor rotor + propeller(kg.m^2)<br>
%M=Cm*w^2<br>
ModelParam_rotorCm = 1.779e-07;    %Rotor torque coefficient(kg.m^2)<br>
%T=Ct**w^2<br>
ModelParam_rotorCt = 1.105e-05;    %Rotor thrust coefficient(kg.m^2)<br>
ModelParam_motorMinThr = 0.05;     %Motor throttle dead zone(kg.m^2)<br>
<br>
ModelParam_uavCd = 0.055;   %Damping coefficient(N/(m/s)^2)<br>
ModelParam_uavCCm = [0.0035 0.0039 0.0034]; %Damping moment coefficient vector(N/(m/s)^2)<br>
ModelParam_uavDearo = 0.12;  %Vertical position difference of Aerodynamic center and gravity center(m)<br>
<br>
ModelParam_GlobalNoiseGainSwitch =0;   %Noise level gain<br>
<br>
%Environment Parameter<br>
ModelParam_envGravityAcc = 9.8;  %Gravity acceleration(m/s^2). not used.<br>
ModelParam_envLongitude = 116.259368300000;  %longitude (degree)<br>
ModelParam_envLatitude = 40.1540302;          %Latitude (degree)<br>
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];  %Latitude and longitude<br>
ModelParam_envAltitude = -41.5260009765625;     %Reference height, down is positive<br>
ModelParam_BusSampleRate = 0.001;            %Model sampling rate<br>
<br>
ModelParam_timeSampBaro = 0.01;  % Barometer data sample time<br>
ModelParam_timeSampTurbWind = 0.01; % Atmospheric turbulence data sample time<br>
%%%ModelParam_BattModelEnable=int8(0);<br>
ModelParam_BattHoverMinutes=18; %time of endurance for the battery simulation <br>
ModelParam_BattHoverThr=0.609; % Vehilce hovering time<br>
<br>
%GPS Parameter<br>
ModelParam_GPSEphFinal=0.3; % GPS horizontal accuracy<br>
ModelParam_GPSEpvFinal=0.4;  % GPS vertical accuracy<br>
ModelParam_GPSFix3DFix=3;  % GPS fixed index<br>
ModelParam_GPSSatsVisible=10;  % GPS number of satellites<br>
<br>
%Noise Parameter<br>
ModelParam_noisePowerAccel = [0.001,0.001,0.003];% accelerometer noise power xyz in Body frame<br>
ModelParam_noiseSampleTimeAccel = 0.001; % accelerometer noise sample time<br>
ModelParam_noisePowerOffGainAccel = 0.04; %accelerometer noise factor without motor vibration<br>
ModelParam_noisePowerOffGainAccelZ = 0.03; %accelerometer Z noise factor without motor vibration<br>
ModelParam_noisePowerOnGainAccel = 0.8; %accelerometer noise factor under motor vibration<br>
ModelParam_noisePowerOnGainAccelZ = 4.5; %accelerometer Z noise factor under motor vibration<br>
ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001]; %gyroscope  noise power xyz in Body frame<br>
ModelParam_noiseSampleTimeGyro = 0.001; % gyroscope noise sample time<br>
ModelParam_noisePowerOffGainGyro = 0.02;  %accelerometer noise factor without motor vibration<br>
ModelParam_noisePowerOffGainGyroZ = 0.025; %accelerometer noise Z factor without motor vibration<br>
ModelParam_noisePowerOnGainGyro = 2;  %accelerometer noise factor under motor vibration<br>
ModelParam_noisePowerOnGainGyroZ = 1; %accelerometer Z noise factor under motor vibration<br>
<br>
ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];<br>
ModelParam_noiseSampleTimeMag = 0.01; %magnetometer sample time<br>
ModelParam_noisePowerOffGainMag = 0.02; %magnetometer noise gain without motor magnetic field effect<br>
ModelParam_noisePowerOffGainMagZ = 0.035;<br>
ModelParam_noisePowerOnGainMag = 0.025;  %magnetometer noise gain under motor magnetic field effect<br>
ModelParam_noisePowerOnGainMagZ = 0.05;<br>
ModelParam_noisePowerIMU=0;%IMU noisePower<br>
<br>
ModelParam_noiseUpperGPS=0.5;  %GPS noise upper limit (unit:m)<br>
ModelParam_noiseGPSSampTime=0.2;%GPS Sample time (5Hz)<br>
<br>
ModelParam_noiseUpperBaro=0; %barometer noise upper limit (unit: m)<br>
ModelParam_noiseBaroSampTime=0.5;%barometer noise sample time<br>
ModelParam_noiseBaroCoupleWithSpeed=0;% barometer disturbance factor caused by moving forward<br>
<br>
ModelParam_noiseUpperWindBodyRatio=0;% wind distrubance amplitude scale factor<br>
ModelParam_noiseWindSampTime=0.001;<br>
<br>
<br>
ModelParam_envAirDensity = 1.225;    %ideal air density (not used)<br>
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar<br>
ModelParam_noiseTs = 0.001;<br>
<br>
%Failt Injection Test<br>
ModelFailEnable = boolean(0); %is enabling failt injection test<br>
<br>
%Battery fault simulation info.<br>
ModelFailBatt_isEnable = boolean(0);  %is injecting battery fault?<br>
ModelFailBatt_isUseCustomHovTime = boolean(0); % is use time of endurance simulation?<br>
ModelFailBatt_CustomHovTime=15;%the time of endurance (unit min)<br>
ModelFailBatt_isPowOff = boolean(0);% is power off failure injected.<br>
ModelFailBatt_isLowVoltage = boolean(0); % is low voltage failure injected<br>
ModelFailBatt_remainVoltageRatio = 0.5; %remain voltage for low voltage failure injiection<br>
ModelFailBatt_islowCapacity = boolean(0); % is low capacility failure injected<br>
ModelFailBatt_remainCapacityRatio=0.2; %remain capacility for low capacility failure injection<br>
<br>
<br>
%Propeller Model Failed<br>
ModelFailProp_isEnable = boolean(1);% is injecting propeller failed<br>
ModelFailProp_PropEffRatioVec = ones(1,8);%health state of the eight propeller (0:totally failed,0.x: propller thrust ratio, 1:OK);<br>
<br>
%Payload failure injection<br>
ModelFailLoad_isEnable = boolean(0);%is Enabling payload failure<br>
ModelFailLoad_loadMassRatio = 0;  %payload weight ratio for the vehicle weight (0 to 1)<br>
ModelFailLoad_isLoadFall = boolean(0); %is payload droped<br>
ModelFailLoad_isLoadShift = boolean(0); %is payload offset<br>
ModelFailLoad_LoadShiftXRatio = 0;      %x-direction offset ration 0 to 1<br>
ModelFailLoad_LoadShiftYRatio = 0;      %y-direction offset ration 0 to 1<br>
ModelFailLoad_LoadShiftZRatio = 0;      %z-direction offset ration 0 to 1<br>
ModelFailLoad_isLoadLeak = boolean(0);  %is payload slowly leaked<br>
ModelFailLoad_LoadLeakRatioRate = 0;    %leak speed %/s<br>
<br>
<br>
%wind disturbance Failure<br>
ModelFailWind_isEnable = boolean(0);%is enabling wind disturbance<br>
ModelFailWind_isConstWind = boolean(0);%is constant wind disturbance<br>
ModelFailWind_ConstWindX = 0;%constant wind X-direction speed (m/s)<br>
ModelFailWind_ConstWindY = 0;%constant wind Y-direction speed(m/s)<br>
ModelFailWind_ConstWindZ = 0;%constant wind Z-direction speed(m/s)<br>
ModelFailWind_isGustWind = boolean(0);%is enabling Gust wind<br>
ModelFailWind_GustWindStrength =0;%gust wind stregth (m/s)<br>
ModelFailWind_GustWindFreq =0;%gust wind frequency (times/per minute)<br>
ModelFailWind_isTurbWind = boolean(0);% is enabling atmospheric turbulence<br>
ModelFailWind_TurbWindStrength =0;%turbulence strength<br>
ModelFailWind_isSheerWind = boolean(0);%is enabling sheer wind disturabance<br>
ModelFailWind_SheerWindStrength =0;%sheer wind strength<br>
<br>
ModelFailWind_TurbWindDirec=0;%turbulence wind yaw direction<br>
ModelFailWind_SheerWindDirec=0;%sheer wind yaw direction<br>
<br>


