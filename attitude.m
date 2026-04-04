q = [0.7071, 0.7071, 0, 0];
beta  = 0.03;
angles = zeros(length(Madgwick(:, 1)),3);
quats = zeros(length(Madgwick(:, 1)),4);

for k = 2:length(Madgwick(:, 1))
   dt = Madgwick(k, 1) - Madgwick(k-1, 1);
   a = [Madgwick(k, 2), Madgwick(k, 3), Madgwick(k, 4)];
   a = a/norm(a);
   m = [Madgwick(k, 8), Madgwick(k, 9), Madgwick(k, 10)];
   m = m/norm(m);
   om = [Madgwick(k, 5), Madgwick(k, 6), Madgwick(k, 7)]; 
   wx = om(1); wy = om(2); wz = om(3);
   q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

    qdot = 0.5 * [
    -q1*wx - q2*wy - q3*wz     q0*wx + q2*wz - q3*wy     q0*wy - q1*wz + q3*wx     q0*wz + q1*wy - q2*wx
    ];

   hx = (1 - 2*(q2^2 + q3^2))*m(1) + 2*(q1*q2 - q0*q3)*m(2) + 2*(q1*q3 + q0*q2)*m(3);
hy = 2*(q1*q2 + q0*q3)*m(1) + (1 - 2*(q1^2 + q3^2))*m(2) + 2*(q2*q3 - q0*q1)*m(3);
hz = 2*(q1*q3 - q0*q2)*m(1) + 2*(q2*q3 + q0*q1)*m(2) + (1 - 2*(q1^2 + q2^2))*m(3);

h = [hx, hy, hz];
   b = [sqrt(h(1)^2 + h(2)^2), 0, h(3)];
   bx = b(1); bz = b(3);

   q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);

   g_est = [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), q0^2 - q1^2 - q2^2 + q3^2];
   fg = g_est - a;

   m_est = [ b(1)*(1 - 2*(q2^2 + q3^2)) + b(3)*(2*(q1*q3 - q0*q2)),    b(1)*(2*(q1*q2 - q0*q3))   + b(3)*(2*(q0*q1 + q2*q3)),    b(1)*(2*(q0*q2 + q1*q3))   + b(3)*(1 - 2*(q1^2 + q2^2))];

   fm = m_est - m;
   F = [fg fm]';

   Jg = [ ...
   -2*q2,   2*q3,  -2*q0,   2*q1;
    2*q1,   2*q0,   2*q3,   2*q2;
    0,       -4*q1,  -4*q2,   0 ...
    ];

   Jm = [ ...
   -2*bz*q(3),              2*bz*q(4),             -4*bx*q(3) - 2*bz*q(1),   -4*bx*q(4) + 2*bz*q(2);
   -2*bx*q(4) + 2*bz*q(2),  2*bx*q(3) + 2*bz*q(1),  2*bx*q(2) + 2*bz*q(4),   -2*bx*q(1) + 2*bz*q(3);
    2*bx*q(3),              2*bx*q(4) - 4*bz*q(2),  2*bx*q(1) - 4*bz*q(3),    2*bx*q(2) ...
    ];

   J = [Jg; Jm];

   grad = (J'*F)';
   if(norm(grad) > 0)
    grad = grad/norm(grad);
   end
   qdot = qdot - beta*grad;

   q = q+dt*qdot;
   q = q/norm(q);
    
   quats(k, :) = q;
   [yaw, pitch, roll] = quat2ypr(q);
   angles(k,:) = [yaw, pitch, roll];
end