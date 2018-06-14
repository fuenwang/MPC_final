function [F, Phi,A,B,C] = mpcgain(Am,Bm,Cm,Nc,Np)
    [A,B,C] = Augment(Am, Bm, Cm);
    h(1,:) = C;
    F(1,:) = C*A;
    for kk=2:Np
        h(kk,:) = h(kk-1,:)*A;
        F(kk,:) = F(kk-1,:)*A;
    end
    v = h*B;
    Phi = zeros(Np,Nc);
    Phi(:,1) = v;
    for i=2:Nc
        Phi(:,i) = [zeros(i-1,1); v(1:Np-i+1,1)];
    end
end