
%  需要先运行这个初始化


%  第二个版本的机翼和电机/螺旋桨
SampleTime = 0.004;

Wind = 1;
SensorType = 0;
d2r=pi/180;  % degrees to radians conversion factor
r2d=180/pi;  % radians to degrees conversion factor

%% %% 重力常数  质量  惯量
Ixx=0.2;
Iyy=0.3;
Izz=0.5;
Ixz=0.003;
momentInertia = [Ixx 0 Ixz;0 Iyy 0;Ixz 0 Izz];


Mass=4.0;
GravityAcc=9.806;

tiltIni=0;

%% weather model 风模型参数
env.windDirHor = 90;
env.windBase = 5;
env.windDirTurb = 180;

%% aerodynamics forces model
% 升力
uavParam.aero.rho=1.225;


% 螺旋桨
RX_front = 0.34;
RY_front = 0.28;
RX_rear = 0.34;
RY_rear = 0.28;

% RX_front = 0.346;
% RY_front = 0.279;
% RX_rear = 0.346;
% RY_rear = 0.278;

uavParam.geom.RotorArm1=[ RX_front    RY_front    0];
uavParam.geom.RotorArm2=[ RX_front    -RY_front   0];
uavParam.geom.RotorArm3=[-RX_rear    -RY_rear     0];
uavParam.geom.RotorArm4=[-RX_rear     RY_rear     0];             
% uavParam.geom.PropDiameter=0.254;   % apc 13 x 5.5 直径
% uavParam.motor.RPMMAX=10000;
uavParam.motor.tilt_trim=0;
uavParam.aero.dragCoeffMov=0.027;

%% propulsion   螺旋桨参数
%Minimum Allowed PWM for motors.
minPWM=0.1;

%% rotors 螺旋桨参数  v4010 eolo cn13*5


%% rotors 螺旋桨参数   cn15*5
kT = 6e-05;
kQ = 1e-06;
% rotorsSpeedDown = 146;
% rotorsSpeedUp = 688;

%% rotors 螺旋桨参数  mn3515 p14*4.8   146  687
% kT = 4e-05;
% kQ = 7e-07;
rotorsSpeedDown = 50;
rotorsSpeedUp = 687;


% apc 1047  x2216
% kT = 1.76345e-05;
% kQ = 2.79022e-07;

eT = 0;
eQ = 0;

rho = 1.2;

servoAngleMax = 0.55;

%% rotors 螺旋桨参数
% ct = 4/(pi^3)*0.1142;
% cq = 8/(pi^3)*0.007048;
% R = uavParam.geom.PropDiameter/2;
% rho = 1.2;
% kT = ct*rho*3.14*R^4;
% kQ = cq*rho*3.14*R^5;


%% Ground Model
% Set ground contact force model parameter
contact = struct('spring', 1.28931184836e5, ...
    'vd', 0.02, ...
    'slidingFriction', 0.8, ...
    'rollingFriction', 0.2,  ...
    'gLimit', 100);

%% filter 滤波器模型参数
ForwardVelocityCutoff = 3;
SensorAAFiltNum = 4.386e+06;
SensorAAFiltDen = [1 2.96e+03 4.386e+06];
ReferenceFilterNum = 0.04877;
ReferenceFilterDen = [1 -0.9512];
