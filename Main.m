clear; close all; clc;

% qi = [ri; pi];
% ri = [xi; yi; zi];
% pi = [e0; e1; e2; e3];
% vi = [rdi; omegai];
% omegai = [omegax; omegay; omegaz];

% Geometry
L1 = 3.0;
L2 = 3.0;

% Constant Vectors
sA0 = [0; 0; 0];
sA1m = [-L1; 0; 0];
sB1m = [ L2; 0; 0];

% Initial position
r1 = [L1; 0; 0];
p1 = [0; 0; 0];

% Initial guess
q_initial = [r1; p1;
    zeros(6,1)]; % Body 1 (position and quaternion)

% Refine initial guess using Newton-Raphson method
tol = 1e-6; % Tolerance for convergence
relax = 1.0; % Relaxation factor
q_refined = NewtonRaphson(@Phi, q_initial, 0, tol, relax);

disp('Initial Guess:');
disp(q_initial);

disp('Refined Solution:');
disp(q_refined);

MKE  = Phi(q_initial(1:6,1), 0)
MKE1 = Jacobian(q_initial(1:6, 1))
MKE2 = Gamma(q_initial(1:6, 1), q_initial(7:12, 1), 0)

% Simulation parameters
dt = 0.0025; % Time step
t_final = 3; % Final time
num_steps = t_final / dt;

[Q, ReacForces, Accelerations, Velocities, Ekin, Epot] = RK4(q_refined, num_steps, dt);

figure;

plot3(Q(:,1), Q(:,2), Q(:,3))
grid on
axis equal
% Define the rotation matrix from the quaternion
R = quatToRotMatrix(q_refined(4:7));

% Calculate the transformed points
sA1 = R * sA1m + r1;
sB1 = R * sB1m + r1;

% Plotting
figure;
hold on;
% Plot the body
plot3([sA0(1) sA1(1)], [sA0(2) sA1(2)], [sA0(3) sA1(3)], 'r', 'LineWidth', 2); % sA0 to sA1
plot3([sA1(1) sB1(1)], [sA1(2) sB1(2)], [sA1(3) sB1(3)], 'b', 'LineWidth', 2); % sA1 to sB1

% Mark the points
plot3(sA0(1), sA0(2), sA0(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot3(sA1(1), sA1(2), sA1(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(sB1(1), sB1(2), sB1(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

% Plot the global coordinate system
quiver3(0, 0, 0, 1, 0, 0, 'k', 'LineWidth', 1.5);
quiver3(0, 0, 0, 0, 1, 0, 'k', 'LineWidth', 1.5);
quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 1.5);
text(1, 0, 0, 'X', 'FontSize', 12, 'Color', 'k');
text(0, 1, 0, 'Y', 'FontSize', 12, 'Color', 'k');
text(0, 0, 1, 'Z', 'FontSize', 12, 'Color', 'k');

% Plot the local coordinate system
quiver3(r1(1), r1(2), r1(3), R(1,1), R(2,1), R(3,1), 'r', 'LineWidth', 1.5);
quiver3(r1(1), r1(2), r1(3), R(1,2), R(2,2), R(3,2), 'g', 'LineWidth', 1.5);
quiver3(r1(1), r1(2), r1(3), R(1,3), R(2,3), R(3,3), 'b', 'LineWidth', 1.5);
text(r1(1) + R(1,1), r1(2) + R(2,1), r1(3) + R(3,1), 'X1', 'FontSize', 12, 'Color', 'r');
text(r1(1) + R(1,2), r1(2) + R(2,2), r1(3) + R(3,2), 'Y1', 'FontSize', 12, 'Color', 'g');
text(r1(1) + R(1,3), r1(2) + R(2,3), r1(3) + R(3,3), 'Z1', 'FontSize', 12, 'Color', 'b');

% Labels and settings
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Body 1 Construction with Local and Global Coordinate Systems');
grid on;
axis equal;
view(3);
hold off;