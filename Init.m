load MavLinkStruct;
%load path;

%Initial condition
ModelInit_PosE = [0,0,0]; % Vehicle postion xyz in the NED earth frame (m)
ModelInit_VelB = [0,0,0]; % Vehicle speed xyz in the NED earth frame (m/s)
ModelInit_AngEuler = [0,0,0]; % Vehicle Euler angle xyz (roll,pitch,yaw) (rad)
ModelInit_RateB = [0,0,0]; % Vehicle angular speed xyz (roll,pitch,yaw) in the body frame (rad/s)
ModelInit_RPM = 0; %Initial motor speed (rad/s)

%UAV model parameter
ModelParam_uavMass = 1.4; %Mass of UAV(kg)
ModelParam_uavJxx = 0.0241;  % moment of inertia in body x axis
ModelParam_uavJyy = 0.0239;   % moment of inertia in body y axis
ModelParam_uavJzz = 0.0386;  % moment of inertia in body z axis
%Moment of inertia matrix
ModelParam_uavJ= [ModelParam_uavJxx,0,0;...
    0,ModelParam_uavJyy,0;...
    0,0,ModelParam_uavJzz];
ModelParam_uavType = int8(3); %X-type quadrotor£¬refer to "SupportedVehicleTypes.docx" for specific definitions
ModelParam_uavMotNumbs = int8(4);  %Number of motors
ModelParam_uavR = 0.225;   %Body radius(m)

ModelParam_motorCr = 1148; %Motor throttle-speed curve slope(rad/s)
ModelParam_motorWb =-141.4;  %Motor speed-throttle curve constant term(rad/s)
ModelParam_motorT = 0.02;  %Motor inertia time constant(s)
ModelParam_motorJm = 0.0001287;    %Moment of inertia of motor rotor + propeller(kg.m^2)
%M=Cm*w^2
ModelParam_rotorCm = 1.779e-07;    %Rotor torque coefficient(kg.m^2)
%T=Ct**w^2
ModelParam_rotorCt = 1.105e-05;    %Rotor thrust coefficient(kg.m^2)
ModelParam_motorMinThr = 0.05;     %Motor throttle dead zone(kg.m^2)

ModelParam_uavCd = 0.055;   %Damping coefficient(N/(m/s)^2)
ModelParam_uavCCm = [0.0035 0.0039 0.0034]; %Damping moment coefficient vector(N/(m/s)^2)
ModelParam_uavDearo = 0.12;  %Vertical position difference of Aerodynamic center and gravity center(m)

ModelParam_GlobalNoiseGainSwitch =0;   %Noise level gain

%Environment Parameter
ModelParam_envGravityAcc = 9.8;  %Gravity acceleration(m/s^2). not used.
ModelParam_envLongitude = 116.259368300000;  %longitude (degree)
ModelParam_envLatitude = 40.1540302;          %Latitude (degree)
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];  %Latitude and longitude
ModelParam_envAltitude = -41.5260009765625;     %Reference height, down is positive
ModelParam_BusSampleRate = 0.001;            %Model sampling rate



ModelParam_timeSampBaro = 0.01;  % Barometer data sample time
ModelParam_timeSampTurbWind = 0.01; % Atmospheric turbulence data sample time
%%%ModelParam_BattModelEnable=int8(0);
ModelParam_BattHoverMinutes=18; %time of endurance for the battery simulation 
ModelParam_BattHoverThr=0.609; % Vehilce hovering time

%GPS Parameter
ModelParam_GPSEphFinal=0.3; % GPS horizontal accuracy
ModelParam_GPSEpvFinal=0.4;  % GPS vertical accuracy
ModelParam_GPSFix3DFix=3;  % GPS fixed index
ModelParam_GPSSatsVisible=10;  % GPS number of satellites


%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];% accelerometer noise power xyz in Body frame
ModelParam_noiseSampleTimeAccel = 0.001; % accelerometer noise sample time
ModelParam_noisePowerOffGainAccel = 0.04; %accelerometer noise factor without motor vibration
ModelParam_noisePowerOffGainAccelZ = 0.03; %accelerometer Z noise factor without motor vibration
ModelParam_noisePowerOnGainAccel = 0.8; %accelerometer noise factor under motor vibration
ModelParam_noisePowerOnGainAccelZ = 4.5; %accelerometer Z noise factor under motor vibration
ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001]; %gyroscope  noise power xyz in Body frame
ModelParam_noiseSampleTimeGyro = 0.001; % gyroscope noise sample time
ModelParam_noisePowerOffGainGyro = 0.02;  %accelerometer noise factor without motor vibration
ModelParam_noisePowerOffGainGyroZ = 0.025; %accelerometer noise Z factor without motor vibration
ModelParam_noisePowerOnGainGyro = 2;  %accelerometer noise factor under motor vibration
ModelParam_noisePowerOnGainGyroZ = 1; %accelerometer Z noise factor under motor vibration



ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];
ModelParam_noiseSampleTimeMag = 0.01; %magnetometer sample time
ModelParam_noisePowerOffGainMag = 0.02; %magnetometer noise gain without motor magnetic field effect
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;  %magnetometer noise gain under motor magnetic field effect
ModelParam_noisePowerOnGainMagZ = 0.05;
ModelParam_noisePowerIMU=0;%IMU noisePower

ModelParam_noiseUpperGPS=0.5;  %GPS noise upper limit (unit:m)
ModelParam_noiseGPSSampTime=0.2;%GPS Sample time (5Hz)

ModelParam_noiseUpperBaro=0; %barometer noise upper limit (unit: m)
ModelParam_noiseBaroSampTime=0.5;%barometer noise sample time
ModelParam_noiseBaroCoupleWithSpeed=0;% barometer disturbance factor caused by moving forward

ModelParam_noiseUpperWindBodyRatio=0;% wind distrubance amplitude scale factor
ModelParam_noiseWindSampTime=0.001;


ModelParam_envAirDensity = 1.225;    %ideal air density (not used)
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;



%Failt Injection Test
ModelFailEnable = boolean(0); %is enabling failt injection test

%Battery fault simulation info.
ModelFailBatt_isEnable = boolean(0);  %is injecting battery fault?
ModelFailBatt_isUseCustomHovTime = boolean(0); % is use time of endurance simulation?
ModelFailBatt_CustomHovTime=15;%the time of endurance (unit min)
ModelFailBatt_isPowOff = boolean(0);% is power off failure injected.
ModelFailBatt_isLowVoltage = boolean(0); % is low voltage failure injected
ModelFailBatt_remainVoltageRatio = 0.5; %remain voltage for low voltage failure injiection
ModelFailBatt_islowCapacity = boolean(0); % is low capacility failure injected
ModelFailBatt_remainCapacityRatio=0.2; %remain capacility for low capacility failure injection


%Propeller Model Failed
ModelFailProp_isEnable = boolean(1);% is injecting propeller failed
ModelFailProp_PropEffRatioVec = ones(1,8);%health state of the eight propeller (0:totally failed,0.x: propller thrust ratio, 1:OK);



%Payload failure injection
ModelFailLoad_isEnable = boolean(0);%is Enabling payload failure
ModelFailLoad_loadMassRatio = 0;  %payload weight ratio for the vehicle weight (0~1)
ModelFailLoad_isLoadFall = boolean(0); %is payload droped
ModelFailLoad_isLoadShift = boolean(0); %is payload offset
ModelFailLoad_LoadShiftXRatio = 0;      %x-direction offset ration 0~1
ModelFailLoad_LoadShiftYRatio = 0;      %y-direction offset ration 0~1
ModelFailLoad_LoadShiftZRatio = 0;      %z-direction offset ration 0~1
ModelFailLoad_isLoadLeak = boolean(0);  %is payload slowly leaked
ModelFailLoad_LoadLeakRatioRate = 0;    %leak speed %/s


%wind disturbance Failure
ModelFailWind_isEnable = boolean(0);%is enabling wind disturbance
ModelFailWind_isConstWind = boolean(0);%is constant wind disturbance
ModelFailWind_ConstWindX = 0;%constant wind X-direction speed (m/s)
ModelFailWind_ConstWindY = 0;%constant wind Y-direction speed(m/s)
ModelFailWind_ConstWindZ = 0;%constant wind Z-direction speed(m/s)
ModelFailWind_isGustWind = boolean(0);%is enabling Gust wind
ModelFailWind_GustWindStrength =0;%gust wind stregth (m/s)
ModelFailWind_GustWindFreq =0;%gust wind frequency (times/per minute)
ModelFailWind_isTurbWind = boolean(0);% is enabling atmospheric turbulence
ModelFailWind_TurbWindStrength =0;%turbulence strength
ModelFailWind_isSheerWind = boolean(0);%is enabling sheer wind disturabance
ModelFailWind_SheerWindStrength =0;%sheer wind strength

ModelFailWind_TurbWindDirec=0;%turbulence wind yaw direction
ModelFailWind_SheerWindDirec=0;%sheer wind yaw direction

