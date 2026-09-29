clear,clc,close all;
here = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
figDir = fullfile(here, '..', 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end
%% get data sim
sim_out = load(fullfile(dataDir, "RSHAC_real_condition.mat")).out.data_RSHAC;
data_sim = sim_out.Data;
t_sim = sim_out.Time;

x_sim  = data_sim(:,1);
xd_sim = data_sim(:,2);
q_sim  = rad2deg(data_sim(:,3));
qd_sim = rad2deg(data_sim(:,4));
u_sim  = data_sim(:,5);

idx_end_sim = find(abs(data_sim(:,1)) < 0.02 & abs(data_sim(:,2)) < 0.02 & ...
    abs(rad2deg(data_sim(:,3))) < 0.5 & abs(rad2deg(data_sim(:,4))) < 0.5, 1);
t_goal_sim = t_sim(idx_end_sim)
%% get data real
data_real = rmmissing(readmatrix(fullfile(dataDir, "RSHAC_real_swing_up.csv")), 1);

idx_start = find(abs(rad2deg(data_real(:,3)-2*pi))<=15, 1);
if isempty(idx_start)
    error('Could not find the start of the upright real-data segment.');
end
nAligned = min(numel(t_sim), size(data_real, 1) - idx_start + 1);
if nAligned < 1
    error('The real-data file has no samples after the detected upright segment.');
end
data_sim = data_sim(1:nAligned, :);
t_sim = t_sim(1:nAligned);
data_real = data_real(idx_start:idx_start+nAligned-1, :);

x_real  = data_real(:,1);
xd_real = data_real(:,2);
q_real  = rad2deg(data_real(:,3)-2*pi);
qd_real = rad2deg(data_real(:,4));
u_real  = data_real(:,5);

init_cont = [x_real(1),xd_real(1),q_real(1),qd_real(1)]

idx_end_real = find(abs(x_real) < 0.02 & abs(xd_real) < 0.02 & ...
    abs(q_real) < 0.5 & abs(qd_real) < 0.5, 1);
if isempty(idx_end_real)
    t_goal_real = NaN;
else
    t_goal_real = t_sim(idx_end_real)
end
%% generate time
dt = 1e-3;
t = (0:size(u_real,1)-1).*dt;
t = reshape(t, size(t,2), 1);

%% ref signal
x_ref = @(t) 0.*t;
ref_zero = @(t) 0.*t;
%% plot prep
tiledlayout(3, 2, "TileIndexing","rowmajor", "TileSpacing","tight", "Padding","tight");
scaleCoeff = 2;
fig = gcf;
fig.Position(1) = 0;
fig.Position(2) = -100;
fig.Position(3) = 2 * fig.Position(3);
fig.Position(4) = scaleCoeff * fig.Position(4);

lineWidth = 1.5;
legendLoc = 'northeast';

%% plotting
nexttile;
plot(t, [x_sim, x_real], ...
    "LineWidth", lineWidth);
hold on;
fplot(x_ref, [0 8], "k--", "LineWidth", lineWidth);
legend("Simulation","Real experiment", "x_{ref}",'NumColumns',2,'Location', legendLoc);
title('Cart position');
xlabel("time (s)");
ylabel("(m)");
grid on;
hold off;

nexttile;
plot(t, [xd_sim, xd_real], ...
    "LineWidth", lineWidth);
hold on;
fplot(ref_zero, [0 8], "k--", "LineWidth", lineWidth);
legend("Simulation","Real experiment", "x_{d ref}",'NumColumns',2,'Location', legendLoc);
title('Cart velocity');
xlabel("time (s)");
ylabel("(m/s)");
grid on;
hold off;

nexttile;
plot(t, [q_sim, q_real], ...
    "LineWidth", lineWidth);
hold on;
fplot(ref_zero, [0 8], "k--", "LineWidth", lineWidth);
legend("Simulation","Real experiment", "q_{ref}",'NumColumns',2,'Location', legendLoc);
title('Pendulum angle');
xlabel("time (s)");
ylabel("(deg)");
grid on;
hold off;

nexttile;
plot(t, [qd_sim, qd_real], ...
    "LineWidth", lineWidth);
hold on;
fplot(ref_zero, [0 8], "k--", "LineWidth", lineWidth);
legend("Simulation","Real experiment", "q_{d ref}",'NumColumns',2,'Location', legendLoc);
title('Pendulum angular velocity');
xlabel("time (s)");
ylabel("(deg/s)");
grid on;
hold off;

nexttile;
plot(t, [u_sim, u_real], ...
    "LineWidth", lineWidth);
legend("Simulation","Real experiment",'NumColumns',2,'Location', legendLoc);
title('Input');
xlabel("time (s)");
ylabel("(m/s2)");
grid on;

%% save fig as PDF
fileName = 'RSHAC_real.pdf';
exportgraphics(gcf, fullfile(figDir, fileName), 'ContentType', 'vector');
