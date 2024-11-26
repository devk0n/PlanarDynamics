function [Q, ReacForces, Accelerations, Velocities, Ekin, Epot] = RK4(q_refined, num_steps, dt)
    Q = zeros(num_steps+1, length(q_refined));
    Q(1, :) = q_refined';
    ReacForces = zeros(num_steps+1, 6);
    Accelerations = zeros(num_steps+1, 6);
    Velocities = zeros(num_steps+1, 6);
    Ekin = zeros(num_steps+1, 1);
    Epot = zeros(num_steps+1, 1);
    
    initial_velocity = q_refined(7:12);
    
    for i = 1:num_steps
        q = Q(i, :)';
        
        [k1, reacFor1, qdd1] = dynamics(q);
        [k2, reacFor2, qdd2] = dynamics(q + 0.5 * dt * k1);
        [k3, reacFor3, qdd3] = dynamics(q + 0.5 * dt * k2);
        [k4, reacFor4, qdd4] = dynamics(q + dt * k3);
        
        q_new = (q + dt * (k1 + 2*k2 + 2*k3 + k4) / 6)';
        Q(i+1, :) = q_new;
        
        ReacForces(i+1, :) = (reacFor1 + 2*reacFor2 + 2*reacFor3 + reacFor4)' / 6;
        Accelerations(i+1, :) = (qdd1 + 2*qdd2 + 2*qdd3 + qdd4)' / 6;
        
        avg_acceleration = (qdd1 + 2*qdd2 + 2*qdd3 + qdd4) / 6;
        Velocities(i+1, :) = initial_velocity' + (dt * avg_acceleration)';
        
        initial_velocity = Velocities(i+1, :)';
        
        % Ekin(i+1) = 0.5 * initial_velocity' * M * initial_velocity;
        % Epot(i+1) = q(1:6)' * (-Fgrav);
    end
end