function [yaw, pitch, roll] = quat2ypr(q)
% quat2ypr Convert quaternion to yaw-pitch-roll (ZYX)
% Input:
%   q = [w x y z]
% Output:
%   yaw   (psi)   - rotation about Z axis (rad)
%   pitch (theta) - rotation about Y axis (rad)
%   roll  (phi)   - rotation about X axis (rad)

% Normalize quaternion (important!)
q = q / norm(q);

w = q(1);
x = q(2);
y = q(3);
z = q(4);

% ----- Yaw (Z axis) -----
yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));

% ----- Pitch (Y axis) -----
sinp = 2*(w*y - z*x);
if abs(sinp) >= 1
    pitch = sign(sinp) * pi/2; % use 90 degrees if out of range
else
    pitch = asin(sinp);
end

% ----- Roll (X axis) -----
roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));

end