function q = eulerAnglesToQuaternion(phi, theta, psi)
    % Converts Euler angles (roll, pitch, yaw) to Euler parameters (quaternions)
    % Input:
    %   phi   - roll angle in radians
    %   theta - pitch angle in radians
    %   psi   - yaw angle in radians
    % Output:
    %   q - quaternion [q0, q1, q2, q3]
    
    % Compute the half angles
    halfPhi = phi / 2;
    halfTheta = theta / 2;
    halfPsi = psi / 2;
    
    % Calculate the trigonometric functions of the half angles
    c_phi = cos(halfPhi);
    s_phi = sin(halfPhi);
    
    c_theta = cos(halfTheta);
    s_theta = sin(halfTheta);
    
    c_psi = cos(halfPsi);
    s_psi = sin(halfPsi);
    
    % Calculate the quaternion components
    q0 = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;
    q1 = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
    q2 = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
    q3 = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
    
    % Output the quaternion
    q = [q0, q1, q2, q3];
end