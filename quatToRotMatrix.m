function R = quatToRotMatrix(q)
    % Ensure the quaternion is normalized
    q = q / norm(q);
    
    % Extract the scalar and vector parts
    qw = q(1);
    qx = q(2);
    qy = q(3);
    qz = q(4);

    % Compute the rotation matrix
    R = [1 - 2*qy^2 - 2*qz^2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw;
             2*qx*qy + 2*qz*qw, 1 - 2*qx^2 - 2*qz^2,     2*qy*qz - 2*qx*qw;
             2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx^2 - 2*qy^2];
end