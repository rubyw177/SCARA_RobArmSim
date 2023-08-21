clear;

% ---Generate circle pose
% Circle parameters
radius = 5;  % Radius of the circle
center_x = 1;  % x-coordinate of the circle's center
center_y = -1.8;  % y-coordinate of the circle's center

% Angle parameter
theta = 0:0.1:2*pi;  % Increment the angle in steps of 0.1 radians

% Calculate x-y coordinates
x_pose = center_x + radius * cos(theta);
y_pose = center_y + radius * sin(theta);
z_pose = 0.0*theta + 1.0;
phi_pose = 0.0*theta;

last_x = x_pose(1,end);
last_y = y_pose(1, end);
first_x = x_pose(1, 1);
first_y = y_pose(1, 1);

% ---Cartesian space formulation
path_pose = [

  first_x x_pose   first_x first_x   5.0  5.0  3.0  1.0 -1.0 -3.0 -3.0 first_x; ... % x
  first_y y_pose   first_y first_y   0.0  0.0 -5.0 -1.8 -5.0  0.0  0.0 first_y; ... % y
  2.0     z_pose   1.0     2.0       2.0  1.0  1.0  1.0  1.0  1.0  2.0 2.0; ... % z
  0.0     phi_pose 0.0     0.0       0.0  0.0  0.0  0.0  0.0  0.0  0.0 0.0 % phi
    
    ]; 
path_velo = 0.75;
disp(path_pose);

path_time = velo2time(path_pose,path_velo);
disp([path_time; path_pose]);

tupdate = 0.1;
[t,x,y,z,phi] = trajplanscara(path_time,path_pose,tupdate);
plot3(x,y,z,'bo',x(1),y(1),z(1),'ro');

figure;plot(t,x,t,y,t,z,t,phi); legend('x','y','z','phi')

td = t(2:length(t),1);
figure;plot(td,diff(x),td,diff(y),td,diff(z),td,diff(phi));

% --- Robot parameters
d1 = 6;
a1 = 4;
a2 = 4;
d4 = 1;

% --- Convert cartesian to joint space
[th1,th2,d3,th4] = invkinemscara(d1,a1,a2,d4,x,y,z,phi,0);
figure;plot(t,th1,t,th2,t,d3,t,th4); legend('th1','th2','d3','th4')

figure; plot(td,diff(th1),td,diff(th2),td,diff(d3),td,diff(th4));

DoF = 4;
NoD = length(t); % Number of Data
MotionData = [DoF;NoD;t;rad2deg(th1);rad2deg(th2);d3;rad2deg(th4)];
save('TugasMotionDataScara_1.dat','-ascii','MotionData');
save('TugasMotionDataScara_2.dat','-ascii','MotionData');
disp('End of program');


% --- To be used function
function tp = velo2time(path, velo)
    % VELO2TIME - Convert data from constant velocity to time sequence
    [~, number_of_path] = size(path);
    dp = [0];
    dpsum = 0;
    for i=1:number_of_path-1
        xd2 = (path(1,i+1)-path(1,i))^2;
        yd2 = (path(2,i+1)-path(2,i))^2;
        zd2 = (path(3,i+1)-path(3,i))^2;
        dis = sqrt(xd2+yd2+zd2);
        dpsum = dpsum + dis;
        dp = [dp dpsum];
    end
    tp = [0];
    tpsum = 0;
    for i=1:number_of_path-1
        ts = (dp(i+1)-dp(i))/velo;
        tpsum = tpsum + ts;
        tp =[tp tpsum];
    end
end

function [t, p1, p2, p3, p4] = trajplanscara(tp, pp, tupdate)
    % TRAJPLANSCARA - General trajectory planning for Scara Robot
    nop = length(tp);
    t = []; p1 = []; p2 = []; p3 = []; p4 = [];
    for i = 1:nop-1
        tnow = (tp(i):tupdate:tp(i+1))';
        p1now = ltraj(pp(1,i),pp(1,i+1),tnow);
        p2now = ltraj(pp(2,i),pp(2,i+1),tnow);
        p3now = ltraj(pp(3,i),pp(3,i+1),tnow);
        p4now = ltraj(pp(4,i),pp(4,i+1),tnow);
        t = [t;tnow];
        p1 = [p1;p1now];
        p2 = [p2;p2now];
        p3 = [p3;p3now];
        p4 = [p4;p4now];
    end
end

function q = ltraj(qs,qf,t)
    % LTRAJ - Linear trajectory
     ts = min(t); tf = max(t);
     a0 = qf - tf*(qf-qs)/(tf-ts);
     a1 = (qf-qs)/(tf-ts);
     q = a0 + a1 * t;
end

function [th1,th2,d3,th4]=invkinemscara(d1,a1,a2,d4,x,y,z,phi,elbowconf)
    % INVKINEMSCARA - Inverse kinematics for Scara robot
    th2 = acos((x.^2+y.^2-a1^2-a2^2)/(2*a1*a2));
    if elbowconf ~= 0
        th2 = -th2;
    end
    th1 = atan2(y,x)-atan2(a2*sin(th2),a1+a2*cos(th2));
    d3 = d1-d4-z;
    th4 = phi-th1-th2;
   end
   



