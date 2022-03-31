figure; hold on
data = csvread('gantry_move.csv');
for i = 2:4
    plot(data(:,1), data(:,i))
end
xlabel('Time (ms)')
ylabel('Velocity (m/s)')
