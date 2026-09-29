%MAKE_THEORY_FIGURES  Figures for the SQSM mappings, control lines, and certificate projections.
%
%   F1  fig_sqsm_resolution   (a) SQSM breakpoints, n = 7, theta = 1/2, alpha in {0.3, 0.5, 0.8}
%                             (b) the four control lines of Table 3 in the semantic domain,
%                                 dashed where the lookup extrapolates (Lemma 1)
%   F2  fig_control_actions   intermediate actions u_i(x_i) of Table 3 in physical units with the
%                             small-signal tangent and the sector lines on the certified box
%   F3  fig_certified_region  (q, qd) and (x, xd) projections of the certified box and of the
%                             invariant ellipsoid (nonlinear and linearised plant models)
%
%   Needs certificate_lmi.mat and certificate_projections.mat
%   (rshac_lmi_certificate.m, validate_invariant_set.m).
%   Output: figures/*.pdf (vector) and *.png (300 dpi) at the package root.

clear; clc; close all;
here   = fileparts(mfilename('fullpath'));
figDir = fullfile(here, '..', 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end

P    = rshac_params();
C    = load(fullfile(here, 'certificate_lmi.mat'));
Pr   = load(fullfile(here, 'certificate_projections.mat'));
nl   = find(strcmp({C.S.plant}, 'nonlinear'), 1);
lin  = find(strcmp({C.S.plant}, 'linearised'), 1);
cols = lines(4);
set(groot, 'defaultAxesFontSize', 9, 'defaultTextInterpreter', 'latex', ...
    'defaultAxesTickLabelInterpreter', 'latex', 'defaultLegendInterpreter', 'latex');

%% ---- F1: resolution law ------------------------------------------------------------------
f1 = figure('Units', 'centimeters', 'Position', [2 2 17 6.5], 'Color', 'w');
subplot(1, 2, 1); hold on; box on; grid on;
alphas = [0.3 0.5 0.8];
for m = 1:numel(alphas)
    v = sqsm_values(7, alphas(m), 0.5);
    plot(v, alphas(m)*ones(size(v)), '-o', 'Color', cols(m, :), 'MarkerFaceColor', cols(m, :), 'MarkerSize', 4);
end
xline(0.5, ':k');
xlim([0 1]); ylim([0.15 0.95]);
xlabel('semantic value $v(k)$'); ylabel('$\alpha$');
title('(a) SQSM breakpoints, $n_\le = 7$, $\theta = 1/2$');

subplot(1, 2, 2); hold on; box on; grid on;
s  = linspace(0, 1, 401);
hl = gobjects(1, 4);
labels = {'$x$', '$\dot x$', '$q$', '$\dot q$'};
for i = 1:4
    c  = P.chan{i};
    bp = sqsm_values(c.n, c.a_i, c.th_i);
    tb = sqsm_values(c.n, c.a_u, c.th_u);
    us = interp1(bp, tb, s, 'linear', 'extrap');
    in = s >= bp(1) & s <= bp(end);
    hl(i) = plot(s(in), us(in), '-', 'Color', cols(i, :), 'LineWidth', 1.2);
    plot(s(s <= bp(1)), us(s <= bp(1)), '--', 'Color', cols(i, :));
    plot(s(s >= bp(end)), us(s >= bp(end)), '--', 'Color', cols(i, :));
    plot(bp, tb, 'o', 'Color', cols(i, :), 'MarkerSize', 3, 'MarkerFaceColor', cols(i, :));
end
plot([0 1], [0 1], ':', 'Color', [0.5 0.5 0.5]);
xlim([0 1]); ylim([-0.1 1.1]);
xlabel('semantic state $s_i$'); ylabel('semantic action $u_{is}$');
legend(hl, labels, 'Location', 'southeast');
title('(b) control lines of Table 3 (dashed: extrapolation)');
export(f1, fullfile(figDir, 'fig_sqsm_resolution'));

%% ---- F2: intermediate actions, tangent and sectors ----------------------------------------
f2 = figure('Units', 'centimeters', 'Position', [2 2 17 11], 'Color', 'w');
K    = rshac_gain(P);
env  = [P.env.x P.env.xd P.env.q P.env.qd];
Xbar = C.S(nl).Xbar;
unit = {'$x$ [m]', '$\dot x$ [m/s]', '$q$ [deg]', '$\dot q$ [deg/s]'};
toPlot = [1 1 180/pi 180/pi];
ylab = {'$u_x$ [m/s$^2$]', '$u_{\dot x}$ [m/s$^2$]', '$u_q$ [m/s$^2$]', '$u_{\dot q}$ [m/s$^2$]'};
for i = 1:4
    subplot(2, 2, i); hold on; box on; grid on;
    v  = linspace(-env(i), env(i), 801);
    ui = zeros(size(v));
    for j = 1:numel(v)
        X = zeros(4, 1);  X(i) = v(j);
        [~, uu] = rshac_law(X, P);
        ui(j) = uu(i);
    end
    vb = linspace(-Xbar(i), Xbar(i), 2);
    fill(toPlot(i)*[vb, fliplr(vb)], [C.S(nl).k1(i)*vb, fliplr(C.S(nl).k2(i)*vb)], [0.85 0.85 0.85], 'EdgeColor', 'none');
    plot(toPlot(i)*v, ui, '-', 'Color', cols(i, :), 'LineWidth', 1.3);
    plot(toPlot(i)*v, 4*K(i)*v, '--k');
    yline(P.u_max, ':r');  yline(P.u_min, ':r');
    xlim(toPlot(i)*[-env(i) env(i)]);
    xlabel(unit{i}); ylabel(ylab{i});
end
sgtitle('intermediate actions (solid), small-signal tangent (dashed), sector on the certified box (grey)', 'Interpreter', 'latex', 'FontSize', 9);
export(f2, fullfile(figDir, 'fig_control_actions'));

%% ---- F3: certified region ----------------------------------------------------------------
f3 = figure('Units', 'centimeters', 'Position', [2 2 17 6.5], 'Color', 'w');
b  = C.S(nl).Xbar;
subplot(1, 2, 1); hold on; box on; grid on;
rectangle('Position', rad2deg([-b(3) -b(4) 2*b(3) 2*b(4)]), 'EdgeColor', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1);
plot(rad2deg(Pr.Proj(nl).q_qd(1, :)), rad2deg(Pr.Proj(nl).q_qd(2, :)), '-', 'Color', cols(1, :), 'LineWidth', 1.3);
xlim(1.25*rad2deg(b(3))*[-1 1]);  ylim(1.25*rad2deg(b(4))*[-1 1]);
xlabel('$q$ [deg]'); ylabel('$\dot q$ [deg/s]');
title('(a) $(q, \dot q)$');
subplot(1, 2, 2); hold on; box on; grid on;
rectangle('Position', [-b(1) -b(2) 2*b(1) 2*b(2)], 'EdgeColor', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1);
plot(Pr.Proj(nl).x_xd(1, :), Pr.Proj(nl).x_xd(2, :), '-', 'Color', cols(1, :), 'LineWidth', 1.3);
xlim(1.25*b(1)*[-1 1]);  ylim(1.25*b(2)*[-1 1]);
xlabel('$x$ [m]'); ylabel('$\dot x$ [m/s]');
title('(b) $(x, \dot x)$');
sgtitle('certified box (dashed) and invariant ellipsoid (solid), nonlinear plant (12)', 'Interpreter', 'latex', 'FontSize', 9);
export(f3, fullfile(figDir, 'fig_certified_region'));

fprintf('Figures written to %s\n', figDir);

%% ======================================================================================
function export(fig, base)
exportgraphics(fig, [base '.pdf'], 'ContentType', 'vector');
exportgraphics(fig, [base '.png'], 'Resolution', 300);
end
