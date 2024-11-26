function x = NewtonRaphson(FunFcn, x, p, tol, relax)
    if nargin < 4 || isempty(tol), tol = 1e-10; end
    if nargin < 5 || isempty(relax), relax = 1.0; end

    n = length(x);
    pert = 1e-10;
    i = 0;
    b = feval(FunFcn, x, p);
    n2 = length(b);

    while norm(b) > tol && i < 100
        i = i + 1;
        jacob = zeros(n2, n);
        for j = 1:n
            temp = x(j);  
            if x(j) == 0  
                x(j) = x(j) + pert;
                delta = pert;
            else
                x(j) = x(j) * (1.0 + pert);
                delta = pert * temp;
            end  
            bPert = feval(FunFcn, x, p);
            x(j) = temp;
            for k = 1:n2
                jacob(k, j) = (bPert(k) - b(k)) / delta;
            end
        end  
        x = x - relax * (jacob \ b);
        b = feval(FunFcn, x, p);
        % Print the current state for debugging
        disp(['Iteration ', num2str(i), ': norm(b) = ', num2str(norm(b))]);
        disp('Current x:');
        disp(x);
    end
end