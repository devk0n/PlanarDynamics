function [qd, reacFor, qdd] = dynamics(q)

    % Geometry
    L1 = 3.0;
    L2 = 3.0;
    
    g = -10;

    % Mass properties
    m1 = 50;
    Iyz1 = 1/12 * m1 * (L1 + L2)^2;
    Ixx1 = 0.1;
    J_G1 = diag([Ixx1, Iyz1, Iyz1]);
    M = blkdiag(diag([m1, m1, m1]), J_G1);

    % Gravity force
    Fgrav = M * [0; 0; g; 0; 0; 0]; % Extend to 6x1 for the full body force vector

    % Compute Jacobian and Gamma
    J = Jacobian(q(1:6));
    G = Gamma(q(1:6), q(7:12));

    alpha = 10;
    beta = 3;

    % Combined external forces
    Fext = Fgrav;
    RH = [Fext; -G - 2 * alpha * J * q(7:end) - beta^2 * Phi(q,0)]; % Note the sign of Gamma

    % Ensure the dimensions are consistent
    [num_constraints, num_gen_coords] = size(J);  % Total number of generalized coordinates

    % Adjust J to match the number of constraints and generalized coordinates
    J_ext = [J, zeros(num_constraints, num_gen_coords - size(J, 2))];

    Coef = [M, -J'; J, zeros(num_constraints)];

    % Solving the equations of motion and kinematic constraints
    x = Coef\RH;
    qdd = x(1:num_gen_coords,1);

    % Derivative of the state vector
    qd = [q(7:12); qdd];
    
    % Reaction forces
    lambda = x(num_gen_coords+1:end,1);
    reacFor = J' * lambda;

end