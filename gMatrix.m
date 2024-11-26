function G = gMatrix(q)
    e0 = q(1);
    e1 = q(2);
    e2 = q(3);
    e3 = q(4);
    
    G = [
        -e1, e0,  e3, -e2;
        -e2, e3,  e0, e1;
        -e3, e2, -e1, e0
        ];
end