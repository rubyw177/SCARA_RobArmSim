% Circle parameters
radius = 2.691;  % Radius of the circle
center_x = -1;  % x-coordinate of the circle's center
center_y = -1.8;  % y-coordinate of the circle's center

% Angle parameter
theta = 0:0.1:2*pi;  % Increment the angle in steps of 0.1 radians

% Calculate x-y coordinates
x = center_x + radius * cos(theta);
y = center_y + radius * sin(theta);
z = 0*theta + 1.0;
phi = 0*theta;

% Display the array
disp(z);

% tinggal ganti2 radius sama titik centernya
