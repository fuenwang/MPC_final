function [A, B, C] = Augment(Am, Bm, Cm)
    [m1, ~] = size(Cm);
    [n1, n_in] = size(Bm);
    %Cm = -Cm;
    A = eye(n1+m1, n1+m1);
    A(1:n1, 1:n1) = Am;
    A(n1+1:n1+m1, 1:n1) = Cm * Am;
    
    B = zeros(n1+m1, n_in);
    B(1:n1, :) = Bm;
    B(n1+1:n1+m1, :) = Cm * Bm;
    
    C = zeros(m1, n1+m1);
    C(:, n1+1:n1+m1) = eye(m1, m1);
end