%% Plot velocities saved by gantry_log_velocities.exe
figure; hold on
data = csvread('gantry_move.csv');
for i = 2:4
    plot(data(:,1), data(:,i))
end
xlabel('Time (ms)')
ylabel('Velocity (m/s)')

%% Sanity check: should be something sensible
dt = diff(data(:,1)) / 1000;
dist = sum(data(2:end, 2:end) .* dt);

disp('Finishing position:')
disp(dist)
