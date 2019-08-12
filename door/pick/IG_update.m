function [I, Tau] = IG_update(m1,l1,m2,l2,ld,lp,r,g)

I1 = m1*(3*(r)^2+l1^2)/12 + m1*lp^2;
I2 = m2*(3*(r)^2+l2^2)/12 + m2*(ld-lp)^2;

tau1 = -m1*(2*lp/l1)*g*(l1/2);
if (ld-lp)>(l2/2)
    tau2 = m2*g*(ld-lp);
elseif 0<(ld-lp)&&(ld-lp)<=(l2/2)
    tau2 = m2*2*((ld-lp)/l2)*g*(l2/2);
else
    tau2 = m2*2*((ld-lp)/l2)*g*(l2/2);
end

I = I1 + I2;
Tau = tau1 + tau2;

end