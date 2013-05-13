function P=line_cov(P, kl, kr, B, d)
C=eye(3);
kl2 = kl*kl;
kr2 = kr*kr;
C(1,1) = d*(kl2+kr2)/4;
C(1,2) = d^2*(kr2-kl2)/4/B;
C(1,3) = d*(kr2-kl2)/2/B;
C(2,1) = C(1,2);
C(2,2) = d^3*(kl2+kr2)/3/B/B;
C(2,3) = d^2*(kr2+kl2)/2/B/B;
C(3,1) = C(1,3);
C(3,2) = C(2,3);
C(3,3) = d*(kl2+kr2)/B/B;
phi=eye(3);
phi(2,3) = d;
P = phi*P*phi'+C;
end