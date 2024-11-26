function J = Jacobian(q)
    % Proper notation for the coordinates for each body
    r1 = q(1:3, 1);
    p1 = q(4:6, 1);
    
    p1Q = eulerAnglesToQuaternion(p1(1), p1(2), p1(3))';

    A1 = quatToRotMatrix(p1Q);
    
    % Geometry
    L1 = 3.0;
    L2 = 3.0;
    
    % Constant Vectors
    s0A  = [  0; 0; 0];
    s1Am = [-L1; 0; 0];
    s1Bm = [ L2; 0; 0];
    
    % Initialization of the Jacobian matrix
    I3 = eye(3);
    
    % Spherical joint between body 1 and ground A
    J = zeros(3, 6);
    J(1:3, 1:6) = [I3, -A1 * skew(s1Am)];
end