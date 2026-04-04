t = Madgwick(:,1);

figure;
subplot(3,1,1)
plot(t, angles(:,1))
title('Yaw'), ylabel('rad')

subplot(3,1,2)
plot(t, angles(:,2))
title('Pitch'), ylabel('rad')

subplot(3,1,3)
plot(t, angles(:,3))
title('Roll'), ylabel('rad'), xlabel('Time (s)')

figure;
axis equal
grid on
xlabel('X'), ylabel('Y'), zlabel('Z')
xlim([-1 1]), ylim([-1 1]), zlim([-1 1])
view(3)
hold on
for k = 1:size(quats,1)

    q = quats(k,:); % [w x y z]

    % Rotation matrix from quaternion
    w = q(1); x = q(2); y = q(3); z = q(4);

    R = [ ...
        1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
        2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
        2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)
    ];

    % Body axes
    origin = [0 0 0];
    x_axis = R(:,1)';
    y_axis = R(:,2)';
    z_axis = R(:,3)';

    cla

    % Draw axes
    quiver3(0,0,0, x_axis(1), x_axis(2), x_axis(3), 'r', 'LineWidth', 2)
    quiver3(0,0,0, y_axis(1), y_axis(2), y_axis(3), 'g', 'LineWidth', 2)
    quiver3(0,0,0, z_axis(1), z_axis(2), z_axis(3), 'b', 'LineWidth', 2)

    title('Phone Orientation')

    drawnow

    pause(0.01) % adjust speed
end