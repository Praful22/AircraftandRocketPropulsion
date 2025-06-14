%==========================================================================
% MATLAB Simulation for Multistage Rocket Optimization
%
% This script implements the concepts from "Chapter 8: Multistage Rockets"
% and solves the accompanying problems. The output is formatted as a
% comprehensive report.
%
%==========================================================================

clear;
clc;
close all;

% Define universal constants
g0 = 9.80665; % Standard gravity in m/s^2

%==========================================================================
% Report Header
%==========================================================================
fprintf('******************************************************************\n');
fprintf('*        Analysis Report: Multistage Rocket Optimization         *\n');
fprintf('******************************************************************\n\n');
fprintf('This report details the analysis of several multistage rocket design\n');
fprintf('problems. The calculations are based on the method of Lagrange\n');
fprintf('multipliers to optimize payload fraction for a given delta-V requirement.\n\n');
fprintf('All equations are referenced from the provided PDF, Chapter 8.\n\n');

%==========================================================================
% Section 1: Simulation of the PDF Example (Section 8.3)
%==========================================================================
fprintf('====================================================\n');
fprintf('Analysis 1: Verification of PDF Example (Section 8.3)\n');
fprintf('====================================================\n');
fprintf('Objective: Replicate the results for a 3-stage rocket with identical stages.\n\n');

% Parameters from the PDF example
Isp_ex = 360;       % [sec]
Vn_ex_req = 9077;   % [m/sec]
epsilon_ex = 0.1;
n_ex = 3;           % Number of stages

% Display given parameters
fprintf('Given Parameters:\n');
fprintf('  - Required Delta-V (Vn): %.f m/s\n', Vn_ex_req);
fprintf('  - Number of Stages (n): %d\n', n_ex);
fprintf('  - Specific Impulse (Isp): %d sec (for all stages)\n', Isp_ex);
fprintf('  - Structural Coefficient (epsilon): %.2f (for all stages)\n\n', epsilon_ex);

% Call the optimization function
[alpha_ex, lambda_ex, R_ex, Gamma_ex] = ...
    optimizeRocket(Vn_ex_req, n_ex, Isp_ex, epsilon_ex, g0);

% Display results and compare with PDF
fprintf('Calculated Optimal Design Parameters:\n');
fprintf('  - Lagrange Multiplier (alpha): %.f m/s    (PDF value: 2696 m/s)\n', alpha_ex);
fprintf('  - Per-Stage Payload Ratio (lambda): %.3f  (PDF value: 0.563)\n', lambda_ex(1));
fprintf('  - Per-Stage Mass Ratio (R): %.4f       (PDF value: 2.3575)\n', R_ex(1));
fprintf('  - Overall Payload Fraction (Gamma): %.3f  (PDF value: 0.047)\n\n', Gamma_ex);

%==========================================================================
% Section 2: Solution to Problem 1
%==========================================================================
fprintf('====================================================\n');
fprintf('Analysis 2: Solution to Problem 1 (Two-Stage LEO Rocket)\n');
fprintf('====================================================\n');
fprintf('Objective: Design a two-stage rocket to deliver a 1000 kg payload to LEO\n');
fprintf('and assess the impact of launch site location.\n\n');

% --- Part A: Launch from Kennedy Space Center (KSC) ---
fprintf('--- Part A: Launch from Kennedy Space Center (Eastward Launch) ---\n');

% Given parameters
ML = 1000; % Payload mass, [kg]
V_rot_ksc = 427; % Earth rotation assist at KSC, [m/s]
V_loss_grav = 1200; % Gravitational velocity loss, [m/s]
V_loss_aero = 500; % Aerodynamic velocity loss, [m/s]
V_orbit_leo = 7800; % Typical LEO orbital velocity, [m/s]
Vn_req_p1 = V_orbit_leo + V_loss_grav + V_loss_aero - V_rot_ksc;

% Stage parameters
n_p1 = 2;
Isp_p1 = [320, 450];
epsilon_p1 = [0.05, 0.07];

fprintf('Mission Parameters:\n');
fprintf('  - Target Payload (ML): %d kg\n', ML);
fprintf('  - Required Delta-V (Vn): %.2f m/s\n', Vn_req_p1);
fprintf('  - Stage 1: Isp = %d s, epsilon = %.2f\n', Isp_p1(1), epsilon_p1(1));
fprintf('  - Stage 2: Isp = %d s, epsilon = %.2f\n\n', Isp_p1(2), epsilon_p1(2));

% Call the optimization function
[alpha_p1, lambda_p1, R_p1, Gamma_p1] = ...
    optimizeRocket(Vn_req_p1, n_p1, Isp_p1, epsilon_p1, g0);
M01_p1 = ML / Gamma_p1;

fprintf('Optimal Design Results for KSC Launch:\n');
fprintf('  - Overall Payload Fraction (Gamma): %.4f (%.2f%%)\n', Gamma_p1, Gamma_p1*100);
fprintf('  - Total Vehicle Liftoff Mass (M01): %.2f kg\n', M01_p1);
fprintf('  - Stage 1: Payload Ratio = %.4f, Mass Ratio = %.4f\n', lambda_p1(1), R_p1(1));
fprintf('  - Stage 2: Payload Ratio = %.4f, Mass Ratio = %.4f\n\n', lambda_p1(2), R_p1(2));

% --- Part B: Launch from Kodiak, Alaska ---
fprintf('--- Part B: Launch from Kodiak (Polar Orbit) ---\n');
Vn_req_kodiak = V_orbit_leo + V_loss_grav + V_loss_aero; % No rotational assist

fprintf('Mission Parameters:\n');
fprintf('  - Required Delta-V (Vn): %.2f m/s\n', Vn_req_kodiak);
fprintf('Objective: Determine the payload capacity for the Kodiak launch assuming\n');
fprintf('the same vehicle liftoff mass (%.2f kg) as the KSC design.\n\n', M01_p1);

[~, ~, ~, Gamma_kodiak] = ...
    optimizeRocket(Vn_req_kodiak, n_p1, Isp_p1, epsilon_p1, g0);
ML_kodiak = Gamma_kodiak * M01_p1;

fprintf('Results for Kodiak Launch:\n');
fprintf('  - Optimal Payload Fraction (Gamma) for this mission: %.4f (%.2f%%)\n', Gamma_kodiak, Gamma_kodiak*100);
fprintf('  - New Payload Mass (ML): %.2f kg\n', ML_kodiak);
fprintf('  - Conclusion: The payload capability is reduced by %.2f kg (a %.2f%% decrease)\n',...
    abs(ML_kodiak - ML), (1 - ML_kodiak/ML)*100);
fprintf('  due to the higher delta-V requirement without Earth''s rotational assist.\n\n');

%==========================================================================
% Section 3: Solution to Problem 2
%==========================================================================
fprintf('====================================================\n');
fprintf('Analysis 3: Solution to Problem 2 (Four-Stage Escape Rocket)\n');
fprintf('====================================================\n');
fprintf('Objective: Determine if a four-stage rocket with identical stages can\n');
fprintf('reach Earth escape velocity with a non-zero payload.\n\n');

% Given parameters
n_p2 = 4;
V_escape = 11176;
V_rot_ksc_p2 = 427;
V_loss_grav_p2 = 1500;
V_loss_aero_p2 = 600;
Isp_p2 = 330;
epsilon_p2 = 0.1;
Vn_req_p2 = V_escape + V_loss_grav_p2 + V_loss_aero_p2 - V_rot_ksc_p2;
C_p2 = Isp_p2 * g0;

fprintf('Given Parameters:\n');
fprintf('  - Required Delta-V (Vn): %.2f m/s\n', Vn_req_p2);
fprintf('  - Number of Stages (n): %d\n', n_p2);
fprintf('  - Per-Stage Isp: %d s\n', Isp_p2);
fprintf('  - Per-Stage Epsilon: %.2f\n\n', epsilon_p2);

Vn_max_ideal = n_p2 * C_p2 * log(1 / epsilon_p2);
fprintf('Feasibility Analysis:\n');
fprintf('  - Maximum theoretical Delta-V (for zero payload): %.2f m/s\n', Vn_max_ideal);

if Vn_req_p2 < Vn_max_ideal
    exp_term = exp(Vn_req_p2 / (n_p2 * C_p2));
    numerator = 1 - epsilon_p2 * exp_term;
    denominator = (1 - epsilon_p2) * exp_term;
    Gamma_p2 = (numerator / denominator)^n_p2;
    
    fprintf('  - Result: The required Vn is LESS than the max ideal Vn.\n');
    fprintf('  - Conclusion: The mission is possible with a finite payload.\n');
    fprintf('  - Calculated Payload Fraction (Gamma): %.6f (%.4f%%)\n\n', Gamma_p2, Gamma_p2*100);
else
    fprintf('  - Result: The required Vn is GREATER than the max ideal Vn.\n');
    fprintf('  - Conclusion: The mission is not possible with a finite payload.\n\n');
end

%==========================================================================
% Section 4: Solution to Problem 3
%==========================================================================
fprintf('====================================================\n');
fprintf('Analysis 4: Solution to Problem 3 (Low-Cost Rocket Design)\n');
fprintf('====================================================\n');
fprintf('Objective: Determine the required structural efficiency (epsilon) for a low-cost,\n');
fprintf('four-stage rocket to reach orbit with a finite payload.\n\n');

% Parameters
n_p3 = 4;
Isp_p3 = 200;
V_orbit_leo_p3 = 7800;
V_loss_grav_p3 = 1200;
V_loss_aero_p3 = 500;
V_rot_ksc_p3 = 427;
Vn_req_p3 = V_orbit_leo_p3 + V_loss_grav_p3 + V_loss_aero_p3 - V_rot_ksc_p3;
C_p3 = Isp_p3 * g0;

fprintf('Given Parameters:\n');
fprintf('  - Assumed Required Delta-V (Vn) to LEO: %.2f m/s\n', Vn_req_p3);
fprintf('  - Number of Stages (n): %d\n', n_p3);
fprintf('  - Per-Stage Isp: %d s\n\n', Isp_p3);

epsilon_max = exp(-Vn_req_p3 / (n_p3 * C_p3));

fprintf('Design Constraint Analysis:\n');
fprintf('  To achieve orbit with a finite payload (Gamma > 0), the structural\n');
fprintf('  coefficient must be less than a maximum theoretical value.\n\n');
fprintf('  - Maximum Allowable Structural Coefficient (epsilon_max): %.4f\n', epsilon_max);
fprintf('  - Conclusion: The structural mass of each stage must be less than %.2f%% of\n', epsilon_max*100);
fprintf('    the stage''s total mass (structure + propellant). This represents a\n');
fprintf('    very challenging, though not impossible, structural efficiency requirement.\n\n');

fprintf('******************************************************************\n');
fprintf('*                      End of Report                             *\n');
fprintf('******************************************************************\n');


%% ========================================================================
%  Function Definition: General Rocket Optimization
%  ========================================================================
function [alpha, lambda, R, Gamma] = optimizeRocket(Vn_req, n, Isp, epsilon, g0)
    % Implements the general optimization for a multistage rocket.
    % This revised version can handle scalar inputs for Isp and epsilon if
    % all stages are identical.
    if isscalar(Isp)
        Isp = ones(1, n) * Isp;
    elseif length(Isp) ~= n
        error('The Isp vector must be a scalar or have length n.');
    end
    if isscalar(epsilon)
        epsilon = ones(1, n) * epsilon;
    elseif length(epsilon) ~= n
        error('The epsilon vector must be a scalar or have length n.');
    end
    C = Isp * g0;
    Vn_max_possible = sum(C .* log(1 ./ epsilon));
    if Vn_req >= Vn_max_possible
        error('Required Delta-V (%.f m/s) is greater than or equal to the maximum possible Delta-V (%.f m/s) for this rocket configuration. Mission is impossible.', Vn_req, Vn_max_possible);
    end
    alpha_equation = @(a) sum(C .* log((C - a) ./ (epsilon .* C))) - Vn_req;
    upper_bound = min(C .* (1 - epsilon));
    search_interval = [0, upper_bound - 1e-6];
    options = optimset('Display','off');
    alpha = fzero(alpha_equation, search_interval, options);
    lambda = (alpha .* epsilon) ./ (C - C .* epsilon - alpha);
    R = (1 + lambda) ./ (epsilon + lambda);
    Gamma = prod(lambda ./ (1 + lambda));
end