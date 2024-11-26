function g = Gamma(q, qd, t)
    % Extract positions and quaternions from generalized coordinates
    r1 = q(1:3, 1);    % Position vector of body 1
    p1 = q(4:6, 1);    % Quaternion of body 1
    
    % Extract velocities from generalized velocities
    r1d = qd(1:3, 1);    % Linear velocity of body 1
    omega1m = qd(4:6, 1); % Angular velocity in the body frame

    % Convert quaternion to rotation matrix
    p1 = eulerAnglesToQuaternion(q(4), q(5), q(6))';

    A1 = quatToRotMatrix(p1);

    % Compute angular velocity in the global frame
    omega1 = 2 * [-p1(2), -p1(3), -p1(4);
                  p1(1), -p1(4),  p1(3);
                  p1(4),  p1(1), -p1(2);
                 -p1(3),  p1(2),  p1(1)] * omega1m;

    % Geometry of the bodies
    L1 = 3.0;
    L2 = 3.0;

    % Constant vectors defining local attachment points in the body frames
    s0A  = [0; 0; 0];      % Attachment point in global frame
    s1Am = [-L1; 0; 0];    % Attachment point in body 1 frame
    s1Bm = [L2; 0; 0];     % Another attachment point in body 1 frame (not used here)

    % Spherical joint between body 1 and ground at A
    g(1:3, 1) = A1 * skew(omega1) * skew(s1Am) * omega1(2:4);
end