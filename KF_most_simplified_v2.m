%% KF

%% generate data
clc, clear
% mechanic model
beta = 0; %3.14/8;
phi = 0; %3.14/6;
d = 0.40;
delta = deg2rad(57); % megnetic field inclination in Shijiazhuang

% acc in sine wave
T = 0.05;
time_slice = T:T:15;

ang_acc = sin(linspace(0, pi/16, length(time_slice)));
ang_vel = [0];
for i = 1:length(ang_acc)-1
    ang_vel(i+1) = ang_vel(i) + T*ang_acc(i);
end
ang = [0];
for i = 1:length(ang_vel)-1
    ang(i+1) = ang(i) + T*ang_vel(i);
end
mega = [];
for i = 1:length(ang)
    mega(:, i) = [cos(ang(i))*cos(delta); -sin(ang(i))*cos(delta)];
end

acc = [sin(linspace(0, 2*pi, length(time_slice))); sin(linspace(0, pi, length(time_slice)))];
vel = [0; 0];
for i = 1:size(acc, 2)-1
    vel(:, i+1) = vel(:, i) + T*acc(:, i);
end
pst = [0; 0];
for i = 1:size(vel, 2)-1
    pst(:, i+1) = pst(:, i) + T*[cos(ang(i)), -sin(ang(i)); sin(ang(i)), cos(ang(i))]*vel(:, i);
end

sensor_data = [vel; mega];
sensor_data = sensor_data + randn(size(sensor_data))*0.15;

figure; hold on;
for i = 1:4
    plot(time_slice, sensor_data(i, :));
end
hold off; xlabel("Time(s)"); ylabel("measuremeant"); title("sensor data"); grid on
legend(["v_x", "v_y", "m_1", "m_2"], "Location", "best");

%% KF
state_vector = zeros([5, 1]);
P = 0.01*diag(ones([1, 5]));
Q = 0.000*diag(ones([1, 5])); % tunable
R = 0.0225*diag(ones([1, 4])); % tunable(0.15^2)
epsilon_trace = [];
for i = 1:size(sensor_data, 2)-1
    % time update
    state_vector(:, i+1) = state_vector(:, i);
    state_vector(1:2, i+1) = state_vector(1:2, i+1) + ...
                            [T*state_vector(4, i)*cos(state_vector(3, i)) - T*state_vector(5, i)*sin(state_vector(3, i));
                            T*state_vector(4, i)*sin(state_vector(3, i)) + T*state_vector(5, i)*cos(state_vector(3, i))];
    state_vector(:, i+1) = state_vector(:, i+1) + ...
                            [(T^2)/2 * cos(state_vector(3, i)) * acc(1, i) - (T^2)/2 * sin(state_vector(3, i)) * acc(2, i);
                            (T^2)/2 * sin(state_vector(3, i)) * acc(1, i) + (T^2)/2 * cos(state_vector(3, i)) * acc(2, i);
                            T*ang_vel(i); T*acc(1, i); T*acc(2, i)];
    F = zeros([5, 5]) + diag(ones([1, 5]));
    F(1:2, 3:5) = [-T*state_vector(4, i)*sin(state_vector(3, i))-T*state_vector(5, i)*cos(state_vector(3, i)), ...
                    T*cos(state_vector(3, i)), -T*sin(state_vector(3, i)); ...
                    T*state_vector(4, i)*cos(state_vector(3, i))-T*state_vector(5, i)*sin(state_vector(3, i)), ...
                    T*sin(state_vector(3, i)), T*cos(state_vector(3, i))];
    P = F * P * F' + Q;
    % measurement update
    H = [0, 0, 0, 1, 0; ...
        0, 0, 0, 0, 1; ...
        0, 0, -sin(state_vector(3, i+1))*cos(delta), 0, 0; ...
        0, 0, -cos(state_vector(4, i+1))*cos(delta), 0, 0];
    S = H * P * H' + R;
    K = P * H' * S^(-1);
    y = [state_vector(4, i+1); state_vector(5, i+1); ...
        cos(state_vector(3, i+1))*cos(delta); ...
        -sin(state_vector(3, i+1))*cos(delta)];
    epsilon = sensor_data(:, i+1) - y;
    epsilon_trace = [epsilon_trace, epsilon];
    state_vector(:, end) = state_vector(:, end) + K * epsilon;
    P = P - K * S * K';
end

%% plot
figure; hold on;
plot(state_vector(1, :), state_vector(2, :));
plot(pst(1, :), pst(2, :));
legend(["estimated motion", "real motion"], "Location", "best");
xlabel("X(m)"); ylabel("Y(m)"); grid on; axis equal;
title("motion"); hold off

figure; hold on
plot(time_slice, state_vector(1, :));
plot(time_slice, pst(1, :));
plot(time_slice, state_vector(2, :));
plot(time_slice, pst(2, :));
legend(["estimated pst_x", "real pst_x", "estimated pst_y", "real pst_y"], "Location", "best");
xlabel("Time(s)"); ylabel("pst(m)"); grid on; 
title("pst"); hold off

figure; hold on
plot(time_slice, state_vector(4, :));
plot(time_slice, vel(1, :));
plot(time_slice, state_vector(5, :));
plot(time_slice, vel(2, :));
legend(["estimated vel_x", "real vel_x", "estimated vel_y", "real vel_y"], "Location", "best");
xlabel("Time(s)"); ylabel("vel(m/s)"); grid on; 
title("vel"); hold off

figure; hold on
plot(time_slice, state_vector(3, :));
plot(time_slice, ang);
legend(["estimated angular", "real angular"], "Location", "best");
xlabel("Time(s)"); ylabel("ang(rad)"); grid on; 
title("ang"); hold off