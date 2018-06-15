function eta = QPhild(H, f, A_cons, b)
    [n1, m1] = size(A_cons);
    eta = -H\f;
    kk=0;
    for i=1:n1
        if (A_cons(i, :)*eta>b(i))
            kk = kk + 1;
        end
    end
    if kk == 0
        return;
    end
    P = A_cons*(H\A_cons');
    d = (A_cons*(H\f)+b);
    [n, m] = size(d);
    x_ini = zeros(n ,m);
    lambda = x_ini;
    al = 10;
    for km = 1:38
        lambda_p = lambda;
        for i = 1:n
            w = P(i, :) * lambda - P(i, i)*lambda(i, 1);
            w = w + d(i, 1);
            la = -w / P(i, i);
            lambda(i, 1) = max(0, la);
        end
        al = (lambda - lambda_p)' * (lambda - lambda_p);
        if al < 10e-8
            break;
        end
    end
end