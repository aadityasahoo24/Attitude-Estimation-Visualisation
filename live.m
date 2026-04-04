clear; clc;

%% init
url = 'http://192.168.1.3:8080/get?graX&graY&graZ&gyrX&gyrY&gyrZ&magX&magY&magZ';

beta = 0.05;
q = [1 0 0 0]; 
prev_time = tic;

figure;
axis equal
grid on
xlabel('X'), ylabel('Y'), zlabel('Z')
xlim([-1 1]), ylim([-1 1]), zlim([-1 1])
view(3)
hold on

while true
    %% GET DATA FROM PHYPHOX
    data = webread(url);

    ax = data.buffer.graX.buffer(end);
    ay = data.buffer.graY.buffer(end);
    az = data.buffer.graZ.buffer(end);

    gx = data.buffer.gyrX.buffer(end);
    gy = data.buffer.gyrY.buffer(end);
    gz = data.buffer.gyrZ.buffer(end);

    mx = data.buffer.magX.buffer(end);
    my = data.buffer.magY.buffer(end);
    mz = data.buffer.magZ.buffer(end);

    %% TIME STEP
    dt = toc(prev_time);
    prev_time = tic;

    %% NORMALIZE SENSORS
    a = [ax ay az]; a = a / norm(a);
    m = [mx my mz]; m = m / norm(m);

    %% GYRO UPDATE
    qdot = 0.5 * quatmultiply(q, [0 gx gy gz]);

    %% MAG REFERENCE
    h = quatmultiply(q, quatmultiply([0 m], quatconj(q)));
    hx = h(2); hy = h(3); hz = h(4);
    bx = sqrt(hx^2 + hy^2);
    bz = hz;
    
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);

    %% GRAVITY ERROR
    fg = [
        2*(q1*q3 - q0*q2) - a(1);
        2*(q0*q1 + q2*q3) - a(2);
        q0^2 - q1^2 - q2^2 + q3^2 - a(3)
    ];

    %% MAG ERROR
    m_est = [
        bx*(1 - 2*(q2^2 + q3^2)) + bz*(2*(q1*q3 - q0*q2));
        bx*(2*(q1*q2 - q0*q3))   + bz*(2*(q0*q1 + q2*q3));
        bx*(2*(q0*q2 + q1*q3))   + bz*(1 - 2*(q1^2 + q2^2))
    ];

    fm = m_est - m(:);

    %%  COMBINED ERROR
    F = [fg; fm];

    %%  JACOBIAN
    Jg = [
        -2*q2,  2*q3, -2*q0,  2*q1;
         2*q1,  2*q0,  2*q3,  2*q2;
         0,    -4*q1, -4*q2,  0
    ];

    Jm = [
        -2*bz*q2,              2*bz*q3,             -4*bx*q2 - 2*bz*q0,   -4*bx*q3 + 2*bz*q1;
        -2*bx*q3 + 2*bz*q1,    2*bx*q2 + 2*bz*q0,    2*bx*q1 + 2*bz*q3,   -2*bx*q0 + 2*bz*q2;
         2*bx*q2,              2*bx*q3 - 4*bz*q1,    2*bx*q0 - 4*bz*q2,    2*bx*q1
    ];

    J = [Jg; Jm];

    %%  GRADIENT
    grad = J' * F;
    if norm(grad) > 0
        grad = grad / norm(grad);
    end

    %% FINAL UPDATE
    qdot = qdot - beta * grad';
    q = q + dt * qdot;
    q = q / norm(q);

    %% VISUALIZATION

    % Rotation matrix
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [ ...
        1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
        2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
        2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)
    ];

    % Phone dimensions
w = 0.3;   % width (X)
h = 0.6;   % height (Y)
t = 0.05;  % thickness (Z) 

V = [ ...
    -w/2 -h/2 -t/2;
     w/2 -h/2 -t/2;
     w/2  h/2 -t/2;
    -w/2  h/2 -t/2;
    -w/2 -h/2  t/2;
     w/2 -h/2  t/2;
     w/2  h/2  t/2;
    -w/2  h/2  t/2];

F = [ ...
    1 2 3 4; % back
    5 6 7 8; % front (screen)
    1 2 6 5; % bottom
    2 3 7 6; % right
    3 4 8 7; % top
    4 1 5 8]; % left

V_rot = (R * V')';

cla

patch('Vertices', V_rot, ...
      'Faces', F, ...
      'FaceVertexCData', [ ...
        0.7 0.7 0.7;  % back
        0.2 0.2 1.0;  % screen (blue)
        0.7 0.7 0.7;
        1.0 0.2 0.2;  % right (red)
        0.2 1.0 0.2;  % top (green)
        0.7 0.7 0.7], ...
      'FaceColor','flat');

hold on

    % Draw axes
    quiver3(0,0,0, R(1,1),R(2,1),R(3,1),'r','LineWidth',2)
    quiver3(0,0,0, R(1,2),R(2,2),R(3,2),'g','LineWidth',2)
    quiver3(0,0,0, R(1,3),R(2,3),R(3,3),'b','LineWidth',2)

    quiver3(0,0,0, 0,0,-1, 'k', 'LineWidth', 2)

    title('Real-Time Phone Orientation')

    drawnow

end
