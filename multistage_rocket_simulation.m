%==========================================================================
% MATLAB Simulation for Multistage Rocket Optimization
%
% This script implements the concepts and solves the problems
% on Multistage Rockets.
%
%==========================================================================

clear;
clc;
close all;

% Define universal constants
g0 = 9.80665; % Standard gravity in m/s^2

%% Section 1: General Optimal Staging Function
% This function is placed at the end of the script. It calculates the
% optimal staging parameters for a rocket where each stage can have
% different characteristics (Isp and structural coefficient).

%% Section 2: Solution to Problem 1
% A two-stage rocket to LEO with different stage properties.

fprintf('====================================================\n');
fprintf('Problem 1: Two-Stage LEO Rocket\n');
fprintf('====================================================\n');

% --- Part A: Launch from Kennedy Space Center (KSC) ---
fprintf('\n--- Part A: Launch from Kennedy Space Center (KSC) ---\n');

% Given parameters
ML = 1000; % Payload mass, [kg]
V_rot_ksc = 427; % Earth rotation assist at KSC, [m/s]
V_loss_grav = 1200; % Gravitational velocity loss, [m/s]
V_loss_aero = 500; % Aerodynamic velocity loss, [m/s]
V_orbit_leo = 7800; % Typical LEO orbital velocity, [m/s] (assumed)

% Total required delta-V (Vn)
Vn_req_p1 = V_orbit_leo + V_loss_grav + V_loss_aero - V_rot_ksc;

% Stage parameters
n_p1 = 2; % Number of stages
Isp_p1 = [320, 450]; % Specific impulse for each stage, sec
epsilon_p1 = [0.05, 0.07]; % Structural coefficient for each stage

fprintf('Required Delta-V (Vn) for KSC launch: %.2f m/s\n', Vn_req_p1);

% Call the general optimization function
[alpha_p1, lambda_p1, R_p1, Gamma_p1] = ...
    optimizeRocket(Vn_req_p1, n_p1, Isp_p1, epsilon_p1, g0);

% Calculate the total initial mass (M01)
% From Gamma = ML / M01  =>  M01 = ML / Gamma
M01_p1 = ML / Gamma_p1;

% Display results
fprintf('\nOptimal Design Results for KSC Launch:\n');
fprintf('Lagrange Multiplier (alpha): %.2f m/s\n', alpha_p1);
fprintf('Overall Payload Fraction (Gamma): %.4f (%.2f%%)\n', Gamma_p1, Gamma_p1*100);
fprintf('Total Vehicle Liftoff Mass (M01): %.2f kg\n\n', M01_p1);
for i = 1:n_p1
    fprintf('--- Stage %d ---\n', i);
    fprintf('  Optimal Payload Ratio (lambda_%d): %.4f\n', i, lambda_p1(i));
    fprintf('  Mass Ratio (R_%d): %.4f\n', i, R_p1(i));
end

% --- Part B: Launch from Kodiak, Alaska ---
fprintf('\n--- Part B: Launch to North-South Orbit from Kodiak ---\n');

% For a north-south (polar) orbit, there is no benefit from Earth's rotation.
V_rot_kodiak = 0;
Vn_req_kodiak = V_orbit_leo + V_loss_grav + V_loss_aero - V_rot_kodiak;
fprintf('Required Delta-V (Vn) for Kodiak launch: %.2f m/s\n', Vn_req_kodiak);
fprintf('Note: The same vehicle technology (Isp, epsilon) is used.\n');

% The question "How does the mass of the payload change?" can be
% interpreted as: If we build a new optimal rocket for this mission with
% the SAME total liftoff mass as the KSC rocket, what payload can it carry?

% Step 1: Find the optimal payload fraction for the Kodiak mission.
[~, ~, ~, Gamma_kodiak] = ...
    optimizeRocket(Vn_req_kodiak, n_p1, Isp_p1, epsilon_p1, g0);

% Step 2: Calculate the new payload mass using the liftoff mass from KSC.
ML_kodiak = Gamma_kodiak * M01_p1;

fprintf('\nOptimal Payload Fraction (Gamma) for Kodiak Mission: %.4f (%.2f%%)\n', ...
    Gamma_kodiak, Gamma_kodiak*100);
fprintf('For the same total liftoff mass of %.2f kg, the new payload is:\n', M01_p1);
fprintf('New Payload Mass (ML) from Kodiak: %.2f kg\n', ML_kodiak);
fprintf('Payload mass changed by: %.2f kg (a reduction of %.2f%%)\n\n', ...
    abs(ML_kodiak - ML), (1 - ML_kodiak/ML)*100);


%% Section 3: Solution to Problem 2
% A four-stage rocket to escape velocity with identical stages.

fprintf('====================================================\n');
fprintf('Problem 2: Four-Stage Escape Velocity Rocket\n');
fprintf('====================================================\n');

% Given parameters
n_p2 = 4;
V_escape = 11176; % Earth escape velocity, m/s
V_rot_ksc_p2 = 427; % m/s
V_loss_grav_p2 = 1500; % m/s
V_loss_aero_p2 = 600; % m/s
Isp_p2 = 330; % sec, same for all stages
epsilon_p2 = 0.1; % same for all stages

% Calculate required Vn
Vn_req_p2 = V_escape + V_loss_grav_p2 + V_loss_aero_p2 - V_rot_ksc_p2;

% Convert Isp to exhaust velocity C
C_p2 = Isp_p2 * g0;

fprintf('Required Delta-V (Vn): %.2f m/s\n', Vn_req_p2);
fprintf('Number of identical stages (n): %d\n', n_p2);
fprintf('Exhaust Velocity (C): %.2f m/s\n', C_p2);
fprintf('Structural Coefficient (epsilon): %.2f\n', epsilon_p2);

% A finite payload (Gamma > 0) is possible if and only if the ideal
% maximum delta-V of the rocket is greater than the required delta-V.
% From Eq (8.23), for Gamma > 0, we need: 1 - epsilon * exp(Vn/(n*C)) > 0
% This rearranges to: Vn < n * C * ln(1/epsilon)

Vn_max_ideal = n_p2 * C_p2 * log(1 / epsilon_p2);

fprintf('\nMaximum theoretical Delta-V for this rocket with a zero payload: %.2f m/s\n', Vn_max_ideal);

% Check if the mission is possible
if Vn_req_p2 < Vn_max_ideal
    % It is possible, so we can calculate the payload fraction using Eq (8.23)
    exp_term = exp(Vn_req_p2 / (n_p2 * C_p2));
    numerator = 1 - epsilon_p2 * exp_term;
    denominator = (1 - epsilon_p2) * exp_term;
    Gamma_p2 = (numerator / denominator)^n_p2; % Eq (8.23)
    
    fprintf('Result: The required Vn is LESS than the max ideal Vn.\n');
    fprintf('Therefore, the payload fraction is greater than zero.\n');
    fprintf('Payload Fraction (Gamma): %.6f (%.4f%%)\n\n', Gamma_p2, Gamma_p2*100);
else
    fprintf('Result: The required Vn is GREATER than the max ideal Vn.\n');
    fprintf('Therefore, it is impossible to reach escape velocity with a finite payload.\n');
    fprintf('The payload fraction is zero or negative (not physically possible).\n\n');
end

%% Section 4: Solution to Problem 3
% A low-cost four-stage rocket. Find the required structural efficiency.

fprintf('====================================================\n');
fprintf('Problem 3: Low-Cost Four-Stage Rocket\n');
fprintf('====================================================\n');

% Given parameters
n_p3 = 4;
Isp_p3 = 200; % sec, very low Isp

% The target is "to orbit". We assume LEO and same losses as Problem 1.
V_orbit_leo_p3 = 7800; % m/s
V_loss_grav_p3 = 1200; % m/s
V_loss_aero_p3 = 500; % m/s
V_rot_ksc_p3 = 427; % m/s, assuming KSC launch for benefit

% Required Vn
Vn_req_p3 = V_orbit_leo_p3 + V_loss_grav_p3 + V_loss_aero_p3 - V_rot_ksc_p3;

% Convert Isp to C
C_p3 = Isp_p3 * g0;

fprintf('Assumed Required Delta-V (Vn) to reach LEO: %.2f m/s\n', Vn_req_p3);
fprintf('Number of identical stages (n): %d\n', n_p3);
fprintf('Exhaust Velocity (C): %.2f m/s\n', C_p3);

% To have a finite payload (Gamma > 0), we use the same condition as in Problem 2:
% Vn < n * C * ln(1/epsilon)
% We need to solve for epsilon:
% Vn / (n*C) < ln(1/epsilon)
% exp(Vn / (n*C)) < 1/epsilon
% epsilon < 1 / exp(Vn / (n*C))
% epsilon < exp(-Vn / (n*C))

epsilon_max = exp(-Vn_req_p3 / (n_p3 * C_p3));

fprintf('\nTo achieve orbit with a finite payload, the structural coefficient (epsilon)\n');
fprintf('must be less than the calculated maximum value.\n');
fprintf('Maximum allowable structural coefficient (epsilon_max): %.4f\n\n', epsilon_max);
fprintf('This means the structural mass must be less than %.2f%% of each stage''s total mass.\n', epsilon_max*100);
fprintf('This is an extremely high structural efficiency requirement.\n');

%% Example problem -  exhaust velocity and structural coefficient the same 
% for all stages.

Isp_ex = 360; %[sec]
Vn_ex_req = 9077; %[m/sec]
epsilon_ex = 0.1;
n_ex = 3;
g0 = 9.80665;

[alpha_ex, lambda_ex, R_ex, Gamma_ex] = optimizeRocket(Vn_ex_req, n_ex, Isp_ex, epsilon_ex, g0);

% Display results
fprintf('\nOptimal Design Results for KSC Launch:\n');
fprintf('Lagrange Multiplier (alpha): %.2f m/s\n', alpha_ex);
fprintf('Overall Payload Fraction (Gamma): %.4f (%.2f%%)\n', Gamma_ex, Gamma_ex*100);
fprintf('Total Vehicle Liftoff Mass (M01): %.2f kg\n\n', M01_p1);
for i = 1:n_p1
    fprintf('--- Stage %d ---\n', i);
    fprintf('  Optimal Payload Ratio (lambda_%d): %.4f\n', i, lambda_p1(i));
    fprintf('  Mass Ratio (R_%d): %.4f\n', i, R_p1(i));
end

%% Function Definition for General Rocket Optimization (Revised and More Robust)
function [alpha, lambda, R, Gamma] = optimizeRocket(Vn_req, n, Isp, epsilon, g0)
    % Implements the general optimization for a multistage rocket.
    % This revised version can handle scalar inputs for Isp and epsilon if
    % all stages are identical.
    %
    % INPUTS:
    %   Vn_req  - Required total delta-V (m/s)
    %   n       - The number of stages
    %   Isp     - Specific impulse(s). Can be a SCALAR (if all stages are
    %             identical) or a VECTOR of length 'n'.
    %   epsilon - Structural coefficient(s). Can be a SCALAR or a VECTOR
    %             of length 'n'.
    %   g0      - Standard gravity (m/s^2)
    %
    % OUTPUTS:
    %   alpha   - The calculated Lagrange multiplier (m/s)
    %   lambda  - Vector of optimal payload ratios for each stage
    %   R       - Vector of mass ratios for each stage
    %   Gamma   - The overall optimal payload fraction

    % --- Input Validation and Expansion ---
    % If Isp is a scalar, expand it into a vector of length n.
    if isscalar(Isp)
        Isp = ones(1, n) * Isp;
    elseif length(Isp) ~= n
        error('The Isp vector must be a scalar or have length n.');
    end
    
    % If epsilon is a scalar, expand it into a vector of length n.
    if isscalar(epsilon)
        epsilon = ones(1, n) * epsilon;
    elseif length(epsilon) ~= n
        error('The epsilon vector must be a scalar or have length n.');
    end

    % --- Main Calculation (Unchanged) ---
    % Convert Isp to exhaust velocity C
    C = Isp * g0;

    % The goal is to find the Lagrange multiplier 'alpha' by solving Eq (8.19).
    % Vn = sum( Ci * ln((Ci - alpha) / (epsilon_i * Ci)) )
    % We define a function whose root is the desired value of alpha.
    % f(alpha) = sum(...) - Vn_req = 0
    alpha_equation = @(a) sum(C .* log((C - a) ./ (epsilon .* C))) - Vn_req;
    
    % We must find a valid search interval for the root finder 'fzero'.
    % The argument of the logarithm must be positive: (C - a) > 0 => a < C.
    % From Eq (8.18), the denominator must also be positive for lambda to be
    % positive. Ci - Ci*epsilon_i - a > 0 => a < Ci*(1-epsilon_i)
    upper_bound = min(C .* (1 - epsilon));
    
    % Check if the mission is possible. If Vn_req is too high, fzero will fail.
    Vn_max_possible = sum(C .* log(1 ./ epsilon));
    if Vn_req >= Vn_max_possible
        error('Required Delta-V (%.f m/s) is greater than or equal to the maximum possible Delta-V (%.f m/s) for this rocket configuration. Mission is impossible.', Vn_req, Vn_max_possible);
    end

    % Use fzero to find alpha. Start search from 0 up to just below the theoretical max.
    search_interval = [0, upper_bound - 1e-6];
    options = optimset('Display','off'); % Suppress fzero output
    alpha = fzero(alpha_equation, search_interval, options);

    % With alpha found, calculate the optimal parameters.
    lambda = (alpha .* epsilon) ./ (C - C .* epsilon - alpha); % Eq 8.18
    R = (1 + lambda) ./ (epsilon + lambda); % Eq 8.4
    Gamma = prod(lambda ./ (1 + lambda)); % Eq 8.6
end