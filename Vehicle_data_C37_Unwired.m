% -------------------------------------------------------------------
%  Vehicle Data for Team Unwired C37 - Formula Bharat 2026
%  ICE Vehicle - NIT Calicut
%  Generated for FB2026 MathWorks Modeling Award
%  Based on DR_CV13.pdf Design Report (SUPRA SAE 2025)
% -------------------------------------------------------------------
%
% This file defines vehicle parameters in the format compatible with
% Formula-Student-Vehicle-Simscape project structure.
%
% Key differences from default Achilles_DWPushrod:
% - ICE powertrain (Royal Enfield Classic 350 with 20mm restrictor)
% - Different mass distribution (35:65 F:R)
% - G-max 170/50 R13 tires
% - Custom suspension hardpoints from CAD
%
% Usage:
%   Run this script to create the Vehicle structure, then use with
%   Simscape model or export individual parameters.
%
% Author: Team Unwired, NIT Calicut
% Date: December 2025

%% Initialize Vehicle Structure
Vehicle = struct;
Vehicle.config = 'C37_Unwired_ICE';

%% ========================= CHASSIS BODY =========================
Vehicle.Chassis = struct;
Vehicle.Chassis.Body = struct;
Vehicle.Chassis.Body.class = struct;
Vehicle.Chassis.Body.class.Value = 'Body_Axle2';

% Wheelbase: 1560 mm (from design report)
% Axle 1 (front) at origin, Axle 2 (rear) at -1.56 m
Vehicle.Chassis.Body.sAxle1 = struct;
Vehicle.Chassis.Body.sAxle1.Units = 'm';
Vehicle.Chassis.Body.sAxle1.Comments = 'Front axle position (origin)';
Vehicle.Chassis.Body.sAxle1.Value = [0 0 0];

Vehicle.Chassis.Body.sAxle2 = struct;
Vehicle.Chassis.Body.sAxle2.Units = 'm';
Vehicle.Chassis.Body.sAxle2.Comments = 'Rear axle position (wheelbase = 1560mm)';
Vehicle.Chassis.Body.sAxle2.Value = [-1.560 0 0];

% Center of Gravity
% Weight distribution: 35% front, 65% rear
% CG longitudinal position from front axle = 0.65 * 1.56 = 1.014 m behind front
% CG height: 280 mm (estimated for FSAE with driver)
Vehicle.Chassis.Body.sCG = struct;
Vehicle.Chassis.Body.sCG.Units = 'm';
Vehicle.Chassis.Body.sCG.Comments = 'CG position: 35:65 F:R weight distribution, 280mm height';
Vehicle.Chassis.Body.sCG.Value = [-1.014 0 0.280];

% Power source location (engine/transmission)
% Engine mounted low and towards rear
Vehicle.Chassis.Body.sPower = struct;
Vehicle.Chassis.Body.sPower.Units = 'm';
Vehicle.Chassis.Body.sPower.Comments = 'Engine/transmission location (rear-mid)';
Vehicle.Chassis.Body.sPower.Value = [-1.2 0 0.25];

% Body width (track width + margin)
% Front track: 1200 mm, Rear track: 1180 mm
Vehicle.Chassis.Body.Width = struct;
Vehicle.Chassis.Body.Width.Units = 'm';
Vehicle.Chassis.Body.Width.Comments = 'Overall body width';
Vehicle.Chassis.Body.Width.Value = 1.35;

% ===================== MASS PROPERTIES =====================
% Total vehicle mass: 300 kg (including 70 kg driver)
% Sprung mass ≈ 270 kg (excluding wheels/suspension)
Vehicle.Chassis.Body.m = struct;
Vehicle.Chassis.Body.m.Units = 'kg';
Vehicle.Chassis.Body.m.Comments = 'Sprung mass (total 300kg - unsprung mass)';
Vehicle.Chassis.Body.m.Value = 260;  % Sprung mass

% Moments of inertia (estimated for FSAE vehicle)
% These should be refined from CAD model
Vehicle.Chassis.Body.J = struct;
Vehicle.Chassis.Body.J.Units = 'kg*m^2';
Vehicle.Chassis.Body.J.Comments = 'Moments of Inertia [Ixx Iyy Izz] - NEED REFINEMENT FROM CAD';
Vehicle.Chassis.Body.J.Value = [35 90 100];  % Estimated for 300kg FSAE

Vehicle.Chassis.Body.Type = 'Body';
Vehicle.Chassis.Body.Instance = 'C37_Unwired';

% Body geometry (for visualization - placeholder)
Vehicle.Chassis.Body.BodyGeometry = struct;
Vehicle.Chassis.Body.BodyGeometry.class = struct;
Vehicle.Chassis.Body.BodyGeometry.class.Value = 'Box';
Vehicle.Chassis.Body.BodyGeometry.sOffset = struct;
Vehicle.Chassis.Body.BodyGeometry.sOffset.Units = 'm';
Vehicle.Chassis.Body.BodyGeometry.sOffset.Value = [-0.78 0 0];
Vehicle.Chassis.Body.BodyGeometry.Color = struct;
Vehicle.Chassis.Body.BodyGeometry.Color.Units = '[RGB]';
Vehicle.Chassis.Body.BodyGeometry.Color.Comments = 'Team Unwired colors';
Vehicle.Chassis.Body.BodyGeometry.Color.Value = [0.8 0.1 0.1];  % Red
Vehicle.Chassis.Body.BodyGeometry.Opacity = struct;
Vehicle.Chassis.Body.BodyGeometry.Opacity.Value = 1;
Vehicle.Chassis.Body.BodyGeometry.Type = 'BodyGeometry';
Vehicle.Chassis.Body.BodyGeometry.Instance = 'Box_C37';

%% ===================== FRONT SUSPENSION (Axle 1) =====================
% Type: Double wishbone pushrod - decoupled configuration
% Damper: DNM RCP2S 210mm
% Configuration: Acceleration/deceleration optimized setup

Vehicle.Chassis.SuspA1 = struct;
Vehicle.Chassis.SuspA1.class = struct;
Vehicle.Chassis.SuspA1.class.Value = 'Linkage';

Vehicle.Chassis.SuspA1.Linkage = struct;
Vehicle.Chassis.SuspA1.Linkage.class = struct;
Vehicle.Chassis.SuspA1.Linkage.class.Value = 'DoubleWishbonePushrodNoSteer';

% Front track width: 1200 mm -> wheel center Y = 0.600 m
% These hardpoints should be extracted from CAD

% Lower wishbone
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone = struct;
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardF = struct;
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardF.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardF.Comments = 'NEED FROM CAD';
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardF.Value = [0.05 0.15 0.10];

Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardR = struct;
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardR.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sInboardR.Value = [-0.10 0.15 0.10];

Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sOutboard = struct;
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sOutboard.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.sOutboard.Value = [0 0.55 0.12];

Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.m = struct;
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.m.Units = 'kg';
Vehicle.Chassis.SuspA1.Linkage.LowerWishbone.m.Value = 1.5;

% Upper wishbone
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone = struct;
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardF = struct;
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardF.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardF.Value = [0.04 0.22 0.24];

Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardR = struct;
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardR.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sInboardR.Value = [-0.12 0.22 0.24];

Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sOutboard = struct;
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sOutboard.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.sOutboard.Value = [-0.02 0.53 0.30];

Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.m = struct;
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.m.Units = 'kg';
Vehicle.Chassis.SuspA1.Linkage.UpperWishbone.m.Value = 1.2;

% Pushrod
Vehicle.Chassis.SuspA1.Linkage.Pushrod = struct;
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sLowerArm = struct;
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sLowerArm.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sLowerArm.Comments = 'Pushrod attachment to lower A-arm';
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sLowerArm.Value = [0.02 0.50 0.14];

Vehicle.Chassis.SuspA1.Linkage.Pushrod.sBellcrank = struct;
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sBellcrank.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sBellcrank.Comments = 'Pushrod attachment to bellcrank';
Vehicle.Chassis.SuspA1.Linkage.Pushrod.sBellcrank.Value = [-0.02 0.22 0.36];

Vehicle.Chassis.SuspA1.Linkage.Pushrod.m = struct;
Vehicle.Chassis.SuspA1.Linkage.Pushrod.m.Units = 'kg';
Vehicle.Chassis.SuspA1.Linkage.Pushrod.m.Value = 0.25;

% Bellcrank
Vehicle.Chassis.SuspA1.Linkage.Bellcrank = struct;
Vehicle.Chassis.SuspA1.Linkage.Bellcrank.sPivot = struct;
Vehicle.Chassis.SuspA1.Linkage.Bellcrank.sPivot.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.Bellcrank.sPivot.Value = [-0.05 0.21 0.36];

Vehicle.Chassis.SuspA1.Linkage.Bellcrank.m = struct;
Vehicle.Chassis.SuspA1.Linkage.Bellcrank.m.Units = 'kg';
Vehicle.Chassis.SuspA1.Linkage.Bellcrank.m.Value = 0.4;

% Upright
Vehicle.Chassis.SuspA1.Linkage.Upright = struct;
Vehicle.Chassis.SuspA1.Linkage.Upright.sWheelCentre = struct;
Vehicle.Chassis.SuspA1.Linkage.Upright.sWheelCentre.Units = 'm';
Vehicle.Chassis.SuspA1.Linkage.Upright.sWheelCentre.Comments = 'Front track = 1200mm';
Vehicle.Chassis.SuspA1.Linkage.Upright.sWheelCentre.Value = [0 0.600 0.225];  % R13 tire

Vehicle.Chassis.SuspA1.Linkage.Upright.m = struct;
Vehicle.Chassis.SuspA1.Linkage.Upright.m.Units = 'kg';
Vehicle.Chassis.SuspA1.Linkage.Upright.m.Value = 2.5;

Vehicle.Chassis.SuspA1.Linkage.Upright.aToe = struct;
Vehicle.Chassis.SuspA1.Linkage.Upright.aToe.Units = 'deg';
Vehicle.Chassis.SuspA1.Linkage.Upright.aToe.Comments = 'Static toe - NEED VERIFICATION';
Vehicle.Chassis.SuspA1.Linkage.Upright.aToe.Value = 0;

Vehicle.Chassis.SuspA1.Linkage.Upright.aCamber = struct;
Vehicle.Chassis.SuspA1.Linkage.Upright.aCamber.Units = 'deg';
Vehicle.Chassis.SuspA1.Linkage.Upright.aCamber.Comments = 'Static camber (typically -1 to -2 deg)';
Vehicle.Chassis.SuspA1.Linkage.Upright.aCamber.Value = -1.5;

Vehicle.Chassis.SuspA1.Linkage.Type = 'Linkage';
Vehicle.Chassis.SuspA1.Linkage.Instance = 'DoubleWishbonePushrod_C37_f';

%% ===================== REAR SUSPENSION (Axle 2) =====================
% Type: Double wishbone pushrod - decoupled configuration
% Damper: DNM RCP2S 190mm
% Configuration: Acceleration/deceleration optimized setup

Vehicle.Chassis.SuspA2 = struct;
Vehicle.Chassis.SuspA2.class = struct;
Vehicle.Chassis.SuspA2.class.Value = 'Linkage';

Vehicle.Chassis.SuspA2.Linkage = struct;
Vehicle.Chassis.SuspA2.Linkage.class = struct;
Vehicle.Chassis.SuspA2.Linkage.class.Value = 'DoubleWishbonePushrodNoSteer';

% Rear track width: 1180 mm -> wheel center Y = 0.590 m

% Lower wishbone
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone = struct;
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardF = struct;
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardF.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardF.Comments = 'NEED FROM CAD';
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardF.Value = [0.22 0.20 0.10];

Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardR = struct;
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardR.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sInboardR.Value = [-0.05 0.10 0.10];

Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sOutboard = struct;
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sOutboard.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.sOutboard.Value = [0 0.50 0.12];

Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.m = struct;
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.m.Units = 'kg';
Vehicle.Chassis.SuspA2.Linkage.LowerWishbone.m.Value = 1.8;

% Upper wishbone
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone = struct;
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardF = struct;
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardF.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardF.Value = [0.22 0.20 0.24];

Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardR = struct;
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardR.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sInboardR.Value = [0.02 0.16 0.22];

Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sOutboard = struct;
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sOutboard.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.sOutboard.Value = [0.03 0.47 0.30];

Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.m = struct;
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.m.Units = 'kg';
Vehicle.Chassis.SuspA2.Linkage.UpperWishbone.m.Value = 1.4;

% Pushrod
Vehicle.Chassis.SuspA2.Linkage.Pushrod = struct;
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sLowerArm = struct;
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sLowerArm.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sLowerArm.Comments = 'Pushrod attachment to lower A-arm';
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sLowerArm.Value = [0.04 0.45 0.14];

Vehicle.Chassis.SuspA2.Linkage.Pushrod.sBellcrank = struct;
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sBellcrank.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sBellcrank.Comments = 'Pushrod attachment to bellcrank';
Vehicle.Chassis.SuspA2.Linkage.Pushrod.sBellcrank.Value = [0 0.18 0.38];

Vehicle.Chassis.SuspA2.Linkage.Pushrod.m = struct;
Vehicle.Chassis.SuspA2.Linkage.Pushrod.m.Units = 'kg';
Vehicle.Chassis.SuspA2.Linkage.Pushrod.m.Value = 0.3;

% Bellcrank
Vehicle.Chassis.SuspA2.Linkage.Bellcrank = struct;
Vehicle.Chassis.SuspA2.Linkage.Bellcrank.sPivot = struct;
Vehicle.Chassis.SuspA2.Linkage.Bellcrank.sPivot.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.Bellcrank.sPivot.Value = [-0.03 0.17 0.38];

Vehicle.Chassis.SuspA2.Linkage.Bellcrank.m = struct;
Vehicle.Chassis.SuspA2.Linkage.Bellcrank.m.Units = 'kg';
Vehicle.Chassis.SuspA2.Linkage.Bellcrank.m.Value = 0.5;

% Upright
Vehicle.Chassis.SuspA2.Linkage.Upright = struct;
Vehicle.Chassis.SuspA2.Linkage.Upright.sWheelCentre = struct;
Vehicle.Chassis.SuspA2.Linkage.Upright.sWheelCentre.Units = 'm';
Vehicle.Chassis.SuspA2.Linkage.Upright.sWheelCentre.Comments = 'Rear track = 1180mm';
Vehicle.Chassis.SuspA2.Linkage.Upright.sWheelCentre.Value = [0 0.590 0.225];

Vehicle.Chassis.SuspA2.Linkage.Upright.m = struct;
Vehicle.Chassis.SuspA2.Linkage.Upright.m.Units = 'kg';
Vehicle.Chassis.SuspA2.Linkage.Upright.m.Value = 3.0;  % Heavier with differential/drive

Vehicle.Chassis.SuspA2.Linkage.Upright.aToe = struct;
Vehicle.Chassis.SuspA2.Linkage.Upright.aToe.Units = 'deg';
Vehicle.Chassis.SuspA2.Linkage.Upright.aToe.Value = 0;

Vehicle.Chassis.SuspA2.Linkage.Upright.aCamber = struct;
Vehicle.Chassis.SuspA2.Linkage.Upright.aCamber.Units = 'deg';
Vehicle.Chassis.SuspA2.Linkage.Upright.aCamber.Value = -1.0;

Vehicle.Chassis.SuspA2.Linkage.Type = 'Linkage';
Vehicle.Chassis.SuspA2.Linkage.Instance = 'DoubleWishbonePushrod_C37_r';

%% ===================== SPRINGS =====================
% DNM RCP2S coilover springs
% Motion ratio front ≈ 0.85 (strut type), rear ≈ 0.70 (pushrod)

Vehicle.Chassis.Spring = struct;
Vehicle.Chassis.Spring.class = struct;
Vehicle.Chassis.Spring.class.Value = 'Independent';

% Front spring (pushrod, 210mm shock)
% Wheel rate target: 38 N/mm (acceleration config) -> Spring rate = 38/0.72^2 ≈ 73.3 N/mm = 73300 N/m
Vehicle.Chassis.Spring.Axle1 = struct;
Vehicle.Chassis.Spring.Axle1.class = struct;
Vehicle.Chassis.Spring.Axle1.class.Value = 'Linear';
Vehicle.Chassis.Spring.Axle1.K = struct;
Vehicle.Chassis.Spring.Axle1.K.Units = 'N/m';
Vehicle.Chassis.Spring.Axle1.K.Comments = 'Front spring rate for accel config (wheel rate 38 N/mm, MR 0.72)';
Vehicle.Chassis.Spring.Axle1.K.Value = 73300;
Vehicle.Chassis.Spring.Axle1.xPreload = struct;
Vehicle.Chassis.Spring.Axle1.xPreload.Units = 'm';
Vehicle.Chassis.Spring.Axle1.xPreload.Value = 0.015;
Vehicle.Chassis.Spring.Axle1.Type = 'Spring';
Vehicle.Chassis.Spring.Axle1.Instance = 'C37_Front_Spring';

% Rear spring (pushrod, 190mm shock)
% Wheel rate target: 52 N/mm (acceleration config, stiffer rear for traction) -> Spring rate = 52/0.68^2 ≈ 112.5 N/mm = 112500 N/m
Vehicle.Chassis.Spring.Axle2 = struct;
Vehicle.Chassis.Spring.Axle2.class = struct;
Vehicle.Chassis.Spring.Axle2.class.Value = 'Linear';
Vehicle.Chassis.Spring.Axle2.K = struct;
Vehicle.Chassis.Spring.Axle2.K.Units = 'N/m';
Vehicle.Chassis.Spring.Axle2.K.Comments = 'Rear spring rate for accel config (wheel rate 52 N/mm, MR 0.68)';
Vehicle.Chassis.Spring.Axle2.K.Value = 112500;
Vehicle.Chassis.Spring.Axle2.xPreload = struct;
Vehicle.Chassis.Spring.Axle2.xPreload.Units = 'm';
Vehicle.Chassis.Spring.Axle2.xPreload.Value = 0.015;
Vehicle.Chassis.Spring.Axle2.Type = 'Spring';
Vehicle.Chassis.Spring.Axle2.Instance = 'C37_Rear_Spring';

Vehicle.Chassis.Spring.Type = 'Springs';
Vehicle.Chassis.Spring.Instance = 'C37_Independent';

%% ===================== DAMPERS =====================
% DNM RCP2S dampers

Vehicle.Chassis.Damper = struct;
Vehicle.Chassis.Damper.class = struct;
Vehicle.Chassis.Damper.class.Value = 'Independent';

% Front damper (210mm travel)
% Damping ratio ζ ≈ 0.7, critical damping c_c = 2*sqrt(k*m_corner)
% m_corner = 52.5 kg (front corner weight), k = 73300 N/m at wheel
% c_wheel = 0.7 * 2 * sqrt(73300 * 52.5) = 8660 N/(m/s)
Vehicle.Chassis.Damper.Axle1 = struct;
Vehicle.Chassis.Damper.Axle1.class = struct;
Vehicle.Chassis.Damper.Axle1.class.Value = 'Linear';
Vehicle.Chassis.Damper.Axle1.Damping = struct;
Vehicle.Chassis.Damper.Axle1.Damping.d = struct;
Vehicle.Chassis.Damper.Axle1.Damping.d.Units = 'N/(m/s)';
Vehicle.Chassis.Damper.Axle1.Damping.d.Comments = 'Damping coefficient for accel config (ζ=0.7)';
Vehicle.Chassis.Damper.Axle1.Damping.d.Value = 8660;
Vehicle.Chassis.Damper.Axle1.Type = 'Damper';
Vehicle.Chassis.Damper.Axle1.Instance = 'DNM_RCP2S_210';

% Rear damper (190mm travel)
% m_corner = 97.5 kg (rear corner weight), k = 112500 N/m at wheel
% c_wheel = 0.7 * 2 * sqrt(112500 * 97.5) = 14650 N/(m/s)
Vehicle.Chassis.Damper.Axle2 = struct;
Vehicle.Chassis.Damper.Axle2.class = struct;
Vehicle.Chassis.Damper.Axle2.class.Value = 'Linear';
Vehicle.Chassis.Damper.Axle2.Damping = struct;
Vehicle.Chassis.Damper.Axle2.Damping.d = struct;
Vehicle.Chassis.Damper.Axle2.Damping.d.Units = 'N/(m/s)';
Vehicle.Chassis.Damper.Axle2.Damping.d.Comments = 'Damping coefficient for accel config (ζ=0.7)';
Vehicle.Chassis.Damper.Axle2.Damping.d.Value = 14650;
Vehicle.Chassis.Damper.Axle2.Type = 'Damper';
Vehicle.Chassis.Damper.Axle2.Instance = 'DNM_RCP2S_190';

Vehicle.Chassis.Damper.Type = 'Dampers';
Vehicle.Chassis.Damper.Instance = 'C37_Independent';

%% ===================== TIRES =====================
% G-max 170/50 R13 bias-ply tires

Vehicle.Chassis.TireA1 = struct;
Vehicle.Chassis.TireA1.class = struct;
Vehicle.Chassis.TireA1.class.Value = 'MFMbody';  % Pacejka Magic Formula

% Tire dimensions
% 170/50 R13: Width=170mm, Aspect=50%, Rim=13"
% Tire OD = 13*25.4 + 2*0.50*170 = 330.2 + 170 = 500.2mm -> radius ≈ 0.25m
% Loaded radius ≈ 0.225m (accounting for deflection)

Vehicle.Chassis.TireA1.tirFile = struct;
Vehicle.Chassis.TireA1.tirFile.Units = '';
Vehicle.Chassis.TireA1.tirFile.Comments = 'G-max 170/50 R13 tire model';
Vehicle.Chassis.TireA1.tirFile.Value = 'which(''C37_Gmax_170_50R13.tir'')';

Vehicle.Chassis.TireA1.Pressure = struct;
Vehicle.Chassis.TireA1.Pressure.Units = 'kPa';
Vehicle.Chassis.TireA1.Pressure.Comments = 'Tire pressure for acceleration configuration (hot)';
Vehicle.Chassis.TireA1.Pressure.Value = 140;  % 20.3 psi hot

Vehicle.Chassis.TireA1.Mass = struct;
Vehicle.Chassis.TireA1.Mass.Units = 'kg';
Vehicle.Chassis.TireA1.Mass.Comments = 'Tire + wheel assembly mass';
Vehicle.Chassis.TireA1.Mass.Value = 8;  % Estimated for 13" bias tire + rim

Vehicle.Chassis.TireA1.Inertia = struct;
Vehicle.Chassis.TireA1.Inertia.Units = 'kg*m^2';
Vehicle.Chassis.TireA1.Inertia.Comments = '[Ixx Iyy] rotational inertia';
Vehicle.Chassis.TireA1.Inertia.Value = [0.4 0.8];

Vehicle.Chassis.TireA1.Type = 'Tire';
Vehicle.Chassis.TireA1.Instance = 'Gmax_170_50R13';

% Rear tires (same as front for now)
Vehicle.Chassis.TireA2 = Vehicle.Chassis.TireA1;
Vehicle.Chassis.TireA2.Pressure.Value = 145;  % 21.0 psi hot (slightly higher for traction)
Vehicle.Chassis.TireA2.Instance = 'Gmax_170_50R13';

%% ===================== AERODYNAMICS =====================
% Minimal aero (no aero package on C37 based on design report)

Vehicle.Chassis.Aero = struct;
Vehicle.Chassis.Aero.class = struct;
Vehicle.Chassis.Aero.class.Value = 'sedan';  % Simple aero model

Vehicle.Chassis.Aero.CL = struct;
Vehicle.Chassis.Aero.CL.Units = '';
Vehicle.Chassis.Aero.CL.Comments = 'No aero package - minimal lift';
Vehicle.Chassis.Aero.CL.Value = 0;  % No downforce

Vehicle.Chassis.Aero.CD = struct;
Vehicle.Chassis.Aero.CD.Units = '';
Vehicle.Chassis.Aero.CD.Comments = 'Drag coefficient for open-wheel formula car';
Vehicle.Chassis.Aero.CD.Value = 0.8;

Vehicle.Chassis.Aero.rho_air = struct;
Vehicle.Chassis.Aero.rho_air.Units = 'kg/m^3';
Vehicle.Chassis.Aero.rho_air.Value = 1.225;

Vehicle.Chassis.Aero.ARef = struct;
Vehicle.Chassis.Aero.ARef.Units = 'm^2';
Vehicle.Chassis.Aero.ARef.Comments = 'Frontal area (estimated)';
Vehicle.Chassis.Aero.ARef.Value = 0.9;

Vehicle.Chassis.Aero.sPressureCentre = struct;
Vehicle.Chassis.Aero.sPressureCentre.Units = 'm';
Vehicle.Chassis.Aero.sPressureCentre.Value = [-0.78 0 0.45];

Vehicle.Chassis.Aero.Type = 'Aero';
Vehicle.Chassis.Aero.Instance = 'C37_NoAero';

%% ===================== BRAKES =====================
% Disc brakes front and rear

Vehicle.Brakes = struct;
Vehicle.Brakes.class = struct;
Vehicle.Brakes.class.Value = 'PedalAbstract_DiscDisc';

% Front brakes
Vehicle.Brakes.Axle1 = struct;
Vehicle.Brakes.Axle1.DiscAndPad = struct;
Vehicle.Brakes.Axle1.DiscAndPad.lMeanRadius = struct;
Vehicle.Brakes.Axle1.DiscAndPad.lMeanRadius.Units = 'm';
Vehicle.Brakes.Axle1.DiscAndPad.lMeanRadius.Comments = 'Brake disc effective radius';
Vehicle.Brakes.Axle1.DiscAndPad.lMeanRadius.Value = 0.10;  % 200mm disc

Vehicle.Brakes.Axle1.DiscAndPad.rMuStatic = struct;
Vehicle.Brakes.Axle1.DiscAndPad.rMuStatic.Value = 0.45;

Vehicle.Brakes.Axle1.DiscAndPad.rMuKinetic = struct;
Vehicle.Brakes.Axle1.DiscAndPad.rMuKinetic.Value = 0.40;

Vehicle.Brakes.Axle1.Caliper = struct;
Vehicle.Brakes.Axle1.Caliper.APiston = struct;
Vehicle.Brakes.Axle1.Caliper.APiston.Units = 'm^2';
Vehicle.Brakes.Axle1.Caliper.APiston.Value = 0.001;  % 1000 mm²

% Rear brakes (smaller)
Vehicle.Brakes.Axle2 = struct;
Vehicle.Brakes.Axle2.DiscAndPad = struct;
Vehicle.Brakes.Axle2.DiscAndPad.lMeanRadius = struct;
Vehicle.Brakes.Axle2.DiscAndPad.lMeanRadius.Units = 'm';
Vehicle.Brakes.Axle2.DiscAndPad.lMeanRadius.Value = 0.08;  % 160mm disc

Vehicle.Brakes.Axle2.DiscAndPad.rMuStatic = struct;
Vehicle.Brakes.Axle2.DiscAndPad.rMuStatic.Value = 0.45;

Vehicle.Brakes.Axle2.DiscAndPad.rMuKinetic = struct;
Vehicle.Brakes.Axle2.DiscAndPad.rMuKinetic.Value = 0.40;

Vehicle.Brakes.Axle2.Caliper = struct;
Vehicle.Brakes.Axle2.Caliper.APiston = struct;
Vehicle.Brakes.Axle2.Caliper.APiston.Units = 'm^2';
Vehicle.Brakes.Axle2.Caliper.APiston.Value = 0.0008;

%% ===================== ICE POWERTRAIN =====================
% Royal Enfield Classic 350 engine with 20mm restrictor
% This is NOT compatible with the EV-only Simscape project
% Use Vehicle Dynamics Blockset for ICE powertrain

Vehicle.Powertrain = struct;
Vehicle.Powertrain.Type = 'ICE_Manual_RWD';

% Engine parameters
Vehicle.Powertrain.Engine = struct;
Vehicle.Powertrain.Engine.Type = 'SI_Naturally_Aspirated';
Vehicle.Powertrain.Engine.Displacement = struct;
Vehicle.Powertrain.Engine.Displacement.Units = 'cc';
Vehicle.Powertrain.Engine.Displacement.Value = 346;

Vehicle.Powertrain.Engine.Cylinders = 1;
Vehicle.Powertrain.Engine.Cooling = 'Air';

% Restrictor effect on torque/power
Vehicle.Powertrain.Engine.Restrictor = struct;
Vehicle.Powertrain.Engine.Restrictor.Diameter = struct;
Vehicle.Powertrain.Engine.Restrictor.Diameter.Units = 'mm';
Vehicle.Powertrain.Engine.Restrictor.Diameter.Value = 20;

% RPM breakpoints
Vehicle.Powertrain.Engine.RPM = struct;
Vehicle.Powertrain.Engine.RPM.Units = 'rpm';
Vehicle.Powertrain.Engine.RPM.Value = [1500 2000 2500 3000 3500 4000 4500 5000 5250 5500 6000 6500 7000];

% Torque curve WITH 20mm restrictor (estimated ~40% reduction at high RPM)
% Unrestricted: 28 Nm @ 4000 rpm
% Restricted estimate based on restrictor flow limitation
Vehicle.Powertrain.Engine.Torque = struct;
Vehicle.Powertrain.Engine.Torque.Units = 'N*m';
Vehicle.Powertrain.Engine.Torque.Comments = 'Estimated torque with 20mm restrictor';
Vehicle.Powertrain.Engine.Torque.Value = [15.0 18.0 21.0 23.5 25.0 26.0 25.5 24.0 22.5 21.0 18.5 16.0 14.0];

% Power curve (calculated from torque)
% P = T * omega = T * RPM * 2*pi/60
Vehicle.Powertrain.Engine.Power = struct;
Vehicle.Powertrain.Engine.Power.Units = 'kW';
Vehicle.Powertrain.Engine.Power.Comments = 'Power calculated from torque curve';
% Power calculated: P(kW) = T(Nm) * RPM / 9549
Vehicle.Powertrain.Engine.Power.Value = [2.36 3.77 5.50 7.38 9.16 10.89 12.02 12.57 12.37 12.09 11.62 10.89 10.26];

% BSFC map (g/kWh)
Vehicle.Powertrain.Engine.BSFC = struct;
Vehicle.Powertrain.Engine.BSFC.Units = 'g/kWh';
Vehicle.Powertrain.Engine.BSFC.Comments = 'Typical BSFC for small SI engine with restrictor';
Vehicle.Powertrain.Engine.BSFC.Value = [320 300 280 265 260 255 260 270 280 295 320 350 400];

% Idle and redline
Vehicle.Powertrain.Engine.IdleRPM = 1200;
Vehicle.Powertrain.Engine.RedlineRPM = 7000;

% Inertia
Vehicle.Powertrain.Engine.Inertia = struct;
Vehicle.Powertrain.Engine.Inertia.Units = 'kg*m^2';
Vehicle.Powertrain.Engine.Inertia.Value = 0.08;  % Estimated for 346cc single

%% ===================== TRANSMISSION =====================
% 6-speed manual gearbox (per design report)

Vehicle.Powertrain.Transmission = struct;
Vehicle.Powertrain.Transmission.Type = 'Manual_6Speed';
Vehicle.Powertrain.Transmission.NumGears = 6;

% Gear ratios (estimated - design report says 6-speed but RE Classic is 5-speed)
Vehicle.Powertrain.Transmission.GearRatios = struct;
Vehicle.Powertrain.Transmission.GearRatios.Units = '';
Vehicle.Powertrain.Transmission.GearRatios.Comments = 'Estimated gear ratios for 6-speed conversion';
Vehicle.Powertrain.Transmission.GearRatios.Value = [2.833 2.062 1.565 1.227 1.042 0.909];

% Primary drive ratio
Vehicle.Powertrain.Transmission.PrimaryDrive = struct;
Vehicle.Powertrain.Transmission.PrimaryDrive.Units = '';
Vehicle.Powertrain.Transmission.PrimaryDrive.Value = 3.0;  % Estimated

% Final drive ratio (52T/17T sprockets from design report)
Vehicle.Powertrain.Transmission.FinalDrive = struct;
Vehicle.Powertrain.Transmission.FinalDrive.Units = '';
Vehicle.Powertrain.Transmission.FinalDrive.Comments = '52T/17T chain drive';
Vehicle.Powertrain.Transmission.FinalDrive.Value = 52/17;  % 3.059:1

% Transmission efficiency
Vehicle.Powertrain.Transmission.Efficiency = 0.92;

% Clutch parameters
Vehicle.Powertrain.Clutch = struct;
Vehicle.Powertrain.Clutch.MaxTorque = struct;
Vehicle.Powertrain.Clutch.MaxTorque.Units = 'N*m';
Vehicle.Powertrain.Clutch.MaxTorque.Value = 60;

%% ===================== DIFFERENTIAL =====================
% Chain drive to rear axle with open or limited slip diff

Vehicle.Powertrain.Differential = struct;
Vehicle.Powertrain.Differential.Type = 'Open';  % or 'LSD' if equipped
Vehicle.Powertrain.Differential.Ratio = 1.0;  % Included in final drive
Vehicle.Powertrain.Differential.Efficiency = 0.98;

%% ===================== STEERING =====================
Vehicle.Chassis.Steer = struct;
Vehicle.Chassis.Steer.class = struct;
Vehicle.Chassis.Steer.class.Value = 'RackPinion';

Vehicle.Chassis.Steer.Ratio = struct;
Vehicle.Chassis.Steer.Ratio.Units = '';
Vehicle.Chassis.Steer.Ratio.Comments = 'Steering wheel to road wheel ratio';
Vehicle.Chassis.Steer.Ratio.Value = 4.0;  % Typical FSAE quick ratio

Vehicle.Chassis.Steer.RackTravel = struct;
Vehicle.Chassis.Steer.RackTravel.Units = 'm';
Vehicle.Chassis.Steer.RackTravel.Value = 0.10;  % ±50mm travel

Vehicle.Chassis.Steer.Type = 'Steer';
Vehicle.Chassis.Steer.Instance = 'RackPinion_C37';

%% ===================== SUMMARY =====================
fprintf('===========================================\n');
fprintf('Team Unwired C37 Vehicle Data Summary\n');
fprintf('===========================================\n');
fprintf('Total Mass:        300 kg (incl. 70 kg driver)\n');
fprintf('Wheelbase:         %.0f mm\n', abs(Vehicle.Chassis.Body.sAxle2.Value(1))*1000);
fprintf('Front Track:       %.0f mm\n', Vehicle.Chassis.SuspA1.Linkage.Upright.sWheelCentre.Value(2)*2000);
fprintf('Rear Track:        %.0f mm\n', Vehicle.Chassis.SuspA2.Linkage.Upright.sWheelCentre.Value(2)*2000);
fprintf('Weight Dist (F:R): 35:65\n');
fprintf('CG Height:         %.0f mm\n', Vehicle.Chassis.Body.sCG.Value(3)*1000);
fprintf('Engine:            RE Classic 350 (346cc) with 20mm restrictor\n');
fprintf('Peak Power:        %.1f kW @ %.0f rpm (restricted)\n', max(Vehicle.Powertrain.Engine.Power.Value), ...
    Vehicle.Powertrain.Engine.RPM.Value(find(Vehicle.Powertrain.Engine.Power.Value == max(Vehicle.Powertrain.Engine.Power.Value), 1)));
fprintf('Peak Torque:       %.1f Nm @ %.0f rpm (restricted)\n', max(Vehicle.Powertrain.Engine.Torque.Value), ...
    Vehicle.Powertrain.Engine.RPM.Value(find(Vehicle.Powertrain.Engine.Torque.Value == max(Vehicle.Powertrain.Engine.Torque.Value), 1)));
fprintf('Transmission:      6-speed manual\n');
fprintf('Final Drive:       %.3f:1 (52T/17T)\n', Vehicle.Powertrain.Transmission.FinalDrive.Value);
fprintf('Front Suspension:  Double wishbone pushrod (decoupled)\n');
fprintf('Rear Suspension:   Double wishbone pushrod (decoupled)\n');
fprintf('Configuration:     Acceleration/deceleration optimized\n');
fprintf('Front Spring:      %.1f N/mm wheel rate (%.1f N/m spring)\n', 38, Vehicle.Chassis.Spring.Axle1.K.Value);
fprintf('Rear Spring:       %.1f N/mm wheel rate (%.1f N/m spring)\n', 52, Vehicle.Chassis.Spring.Axle2.K.Value);
fprintf('Front Damping:     %.0f N/(m/s) (ζ=0.7)\n', Vehicle.Chassis.Damper.Axle1.Damping.d.Value);
fprintf('Rear Damping:      %.0f N/(m/s) (ζ=0.7)\n', Vehicle.Chassis.Damper.Axle2.Damping.d.Value);
fprintf('Tire Pressure:     %.0f kPa F / %.0f kPa R\n', Vehicle.Chassis.TireA1.Pressure.Value, Vehicle.Chassis.TireA2.Pressure.Value);
fprintf('Corner Weights:    FL/FR: %.1f kg, RL/RR: %.1f kg\n', 52.5, 97.5);
fprintf('Tires:             G-max 170/50 R13\n');
fprintf('===========================================\n');

% Save to MAT file for use with Simscape
save('Vehicle_data_C37.mat', 'Vehicle');
fprintf('Vehicle data saved to Vehicle_data_C37.mat\n');
