function Phi = Phi(q, t)
    % Geometry
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

    % Spherical joint constraint at point A
    Phi = zeros(3,1);
    Phi(1:3, 1) = r1 + A1 * s1Am - s0A;
end