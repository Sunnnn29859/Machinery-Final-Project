%% Term Project: 2103-322 Design of Mechanism
% Symbolic Position & Trajectory Analysis: Vertical Rocket Launch 6-Bar Chebyshev Mechanism
clear; clc; close all;
warning('off', 'symbolic:solve:SolutionsDependOnConditions');

%% 1. Define Mechanism Dimensions (Original Lengths Preserved - Vertical Orientation)
L_AC = 80;          % ระยะห่างจุดหมุนฐาน A ถึง C ตามแนวดิ่ง (mm)
L1   = 100;         % ความยาว Rocker ล่าง AB - สีแดง (mm)
L2   = 100;         % ความยาว Rocker บน CD - สีเขียว (mm)
L3   = 40;          % ความยาว Coupler BD - สีเหลืองทอง (mm)
L_BG = 20;          % ตำแหน่งจุดหมุนจรวด G กึ่งกลาง Coupler (mm)

% พิกัดจุดหมุนตรึงบนเสาฐาน (สลับแกน X-Y จากแนวนอนให้ตั้งขึ้นตรงตามโมเดล CAD)
Ax = 0;      Ay = 0;      % จุดหมุนล่าง A (0, 0)
Cx = 0;      Cy = L_AC;   % จุดหมุนบน C (0, 80)
Ex = 15;     Ey = 150;    % จุดหมุนมือหมุน E (15, 150)

% ชุดขับเคลื่อน Grashof Crank-Rocker (ความยาวเดิมไม่เปลี่ยนแปลง)
L4 = 26;                  % ความยาว Crank EF - สีน้ำเงิน (mm)
L5 = 118;                 % ความยาวก้านส่งกำลัง FB - สีดำ (mm)

%% 2. Symbolic Position Analysis (Vector Loop-Closure Equations)
syms theta4 theta1 theta5 theta2 theta3 real

% Loop 1: Drive Dyad (E-F-B-A)
Fx = Ex + L4*cos(theta4);
Fy = Ey + L4*sin(theta4);
Bx = Ax + L1*cos(theta1);
By = Ay + L1*sin(theta1);

sol1 = solve([Bx == Fx + L5*cos(theta5), By == Fy + L5*sin(theta5)], [theta1, theta5]);
th1_sym = sol1.theta1(2); % เลือก Branch กางออกทางขวา (X > 0)

Bx_sym = subs(Bx, theta1, th1_sym);
By_sym = subs(By, theta1, th1_sym);

% Loop 2: Chebyshev 4-Bar Linkage (A-B-D-C)
Dx = Cx + L2*cos(theta2);
Dy = Cy + L2*sin(theta2);

sol2 = solve([Bx + L3*cos(theta3) == Dx, By + L3*sin(theta3) == Dy], [theta2, theta3]);
th2_sym = subs(sol2.theta2(1), theta1, th1_sym); % Crossed X-Configuration (แนวดิ่ง)
th3_sym = subs(sol2.theta3(1), theta1, th1_sym);

Dx_sym = subs(Dx, theta2, th2_sym);
Dy_sym = subs(Dy, theta2, th2_sym);

% พิกัดจุดยึดจรวด Point G (Output Vertical Straight-Line Point)
Gx_sym = Bx_sym + L_BG*cos(th3_sym);
Gy_sym = By_sym + L_BG*sin(th3_sym);

%% 3. Numerical Evaluation & Vertical Straightness Efficiency
N = 180;
th4_vals = linspace(0, 2*pi, N);
th4_deg  = rad2deg(th4_vals);

fn_Bx = matlabFunction(Bx_sym, 'Vars', theta4);
fn_By = matlabFunction(By_sym, 'Vars', theta4);
fn_Dx = matlabFunction(Dx_sym, 'Vars', theta4);
fn_Dy = matlabFunction(Dy_sym, 'Vars', theta4);
fn_Fx = matlabFunction(Fx,     'Vars', theta4);
fn_Fy = matlabFunction(Fy,     'Vars', theta4);
fn_Gx = matlabFunction(Gx_sym, 'Vars', theta4);
fn_Gy = matlabFunction(Gy_sym, 'Vars', theta4);

Bx_v = fn_Bx(th4_vals);  By_v = fn_By(th4_vals);
Dx_v = fn_Dx(th4_vals);  Dy_v = fn_Dy(th4_vals);
Fx_v = fn_Fx(th4_vals);  Fy_v = fn_Fy(th4_vals);
Gx_v = fn_Gx(th4_vals);  Gy_v = fn_Gy(th4_vals);

% คำนวณระยะพุ่งขึ้นแนวดิ่ง (Vertical Stroke) และความคลาดเคลื่อนแนวนอน (Horizontal Deviation)
stroke_Y     = max(Gy_v) - min(Gy_v);          % ระยะพุ่งขึ้นแนวดิ่ง Start -> Finish (mm)
deviation_X  = max(Gx_v) - min(Gx_v);          % ความเบี่ยงเบนซ้าย-ขวาสูงสุด (mm)
mean_X       = mean(Gx_v);                     % แนวเส้นตรงแนวดิ่งอุดมคติ (mm)
eta_straight = (1 - (deviation_X / stroke_Y)) * 100; % ประสิทธิภาพความตรง (%)

fprintf('=== Vertical Rocket Launch Performance ===\n');
fprintf('Vertical Stroke (Delta Y)     : %.2f mm\n', stroke_Y);
fprintf('Horizontal Deviation (Delta X): %.3f mm\n', deviation_X);
fprintf('Straightness Efficiency       : %.2f %%\n', eta_straight);

%% 4. Define Color Palette (Matching 3D CAD Model)
c_L1   = [0.68, 0.16, 0.16]; % Link 1 (AB): สีแดงเข้ม
c_L2   = [0.18, 0.49, 0.22]; % Link 2 (CD): สีเขียวเข้ม
c_L3   = [0.85, 0.65, 0.13]; % Link 3 (BD): สีเหลืองทอง
c_L4   = [0.15, 0.22, 0.60]; % Link 4 (EF): สีน้ำเงินเข้ม
c_L5   = [0.18, 0.18, 0.20]; % Link 5 (FB): สีดำ/เทาเข้ม
c_traj = [0.95, 0.25, 0.05]; % Trajectory G: สีส้มแดง

%% 5. Plot Figure 1: Vertical Workspace & Rocket Launch Trajectory
fig1 = figure('Name', 'Figure 1: Vertical Straight-Line Trajectory', 'Color', 'w', 'Position', [60 60 720 680]);
hold on; grid on; axis equal;

% วาดกรอบอ้างอิงแผ่นฐาน A4 แนวตั้ง (210 x 297 mm)
rectangle('Position', [-25, -25, 165, 220], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1);
text(-20, 188, 'A4 Wall-Mounted Board (Vertical)', 'Color', [0.4 0.4 0.4], 'FontSize', 9);

% พล็อตวิถีการเคลื่อนที่แนวดิ่งของจุด G และป้าย Start / Finish
plot(Gx_v, Gy_v, '-', 'Color', c_traj, 'LineWidth', 3.5, 'DisplayName', sprintf('Rocket Trajectory G (Stroke = %.2f mm)', stroke_Y));
plot([mean_X mean_X], [min(Gy_v) max(Gy_v)], ':k', 'LineWidth', 1.2, 'DisplayName', sprintf('Ideal Vertical Line (X = %.1f mm)', mean_X));
text(mean_X + 18, min(Gy_v), '-- Start',  'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);
text(mean_X + 18, max(Gy_v), '-- Finish', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);

plot(Bx_v, By_v, '--', 'Color', c_L1, 'LineWidth', 1, 'DisplayName', 'Joint B Path');
plot(Dx_v, Dy_v, '--', 'Color', c_L2, 'LineWidth', 1, 'DisplayName', 'Joint D Path');
plot(Fx_v, Fy_v, ':',  'Color', c_L4, 'LineWidth', 1.2, 'DisplayName', 'Hand Crank Path F');
plot([Ax Cx Ex], [Ay Cy Ey], '^k', 'MarkerSize', 10, 'MarkerFaceColor', [0.8 0.8 0.8], 'DisplayName', 'Fixed Ground Pivots');

xlabel('X Position (mm)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Y Position (mm)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('Vertical Rocket Launch Mechanism: Straight-Line Path (\\eta = %.2f%%)', eta_straight), 'FontSize', 11);
xlim([-35, 150]);
ylim([-35, 205]);
legend('Location', 'southeast');

%% 6. Plot Figure 2: Vertical Lift & Horizontal Deviation Profile
fig2 = figure('Name', 'Figure 2: Vertical Straightness Profile', 'Color', 'w', 'Position', [800 60 680 680]);

subplot(2,1,1);
plot(th4_deg, Gy_v, '-', 'Color', c_L4, 'LineWidth', 2); grid on;
ylabel('Vertical Lift Height G_y (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('Vertical Rocket Lift (G_y) vs Hand Crank Angle \theta_4', 'FontSize', 11);
xlim([0 360]);

subplot(2,1,2);
plot(th4_deg, Gx_v, '-', 'Color', c_traj, 'LineWidth', 2); hold on;
yline(mean_X, '--k', sprintf('Ideal Vertical Axis X = %.2f mm', mean_X), 'LineWidth', 1.2);
grid on;
xlabel('Hand Crank Angle \theta_4 (deg)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Horizontal Position G_x (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title(sprintf('Horizontal Deviation \\DeltaX = %.3f mm (Straightness Efficiency = %.2f%%)', deviation_X, eta_straight), 'FontSize', 11);
xlim([0 360]);
ylim([mean_X - 1, mean_X + 1]);

%% 7. Infinite Real-Time Animation with Self-Leveling Gravity Rocket
figure(fig1);
hold on;

h_L2 = plot([Cx Dx_v(1)],      [Cy Dy_v(1)],      '-', 'Color', c_L2, 'LineWidth', 5.5, 'HandleVisibility', 'off');
h_L3 = plot([Bx_v(1) Dx_v(1)], [By_v(1) Dy_v(1)], '-', 'Color', c_L3, 'LineWidth', 5.5, 'HandleVisibility', 'off');
h_L5 = plot([Fx_v(1) Bx_v(1)], [Fy_v(1) By_v(1)], '-', 'Color', c_L5, 'LineWidth', 5.5, 'HandleVisibility', 'off');
h_L1 = plot([Ax Bx_v(1)],      [Ay By_v(1)],      '-', 'Color', c_L1, 'LineWidth', 5.5, 'HandleVisibility', 'off');
h_L4 = plot([Ex Fx_v(1)],      [Ey Fy_v(1)],      '-', 'Color', c_L4, 'LineWidth', 5.5, 'HandleVisibility', 'off');

% จำลองรูปทรงจรวดที่จุด G (ตั้งตรงตามแรงโน้มถ่วงเสมอด้วย Revolute Joint)
rocket_dx = [-4, 4, 4, 0, -4, -4];
rocket_dy = [-12, -12, 8, 16, 8, -12];
h_rocket  = patch(Gx_v(1) + rocket_dx, Gy_v(1) + rocket_dy, [0.75 0.75 0.78], 'EdgeColor', 'k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
h_G       = plot(Gx_v(1), Gy_v(1), 'o', 'Color', 'k', 'MarkerFaceColor', c_traj, 'MarkerSize', 7, 'HandleVisibility', 'off');

k = 1;
while ishandle(fig1)
    set(h_L1, 'XData', [Ax Bx_v(k)],      'YData', [Ay By_v(k)]);
    set(h_L2, 'XData', [Cx Dx_v(k)],      'YData', [Cy Dy_v(k)]);
    set(h_L3, 'XData', [Bx_v(k) Dx_v(k)], 'YData', [By_v(k) Dy_v(k)]);
    set(h_L4, 'XData', [Ex Fx_v(k)],      'YData', [Ey Fy_v(k)]);
    set(h_L5, 'XData', [Fx_v(k) Bx_v(k)], 'YData', [Fy_v(k) By_v(k)]);
    
    set(h_rocket, 'XData', Gx_v(k) + rocket_dx, 'YData', Gy_v(k) + rocket_dy);
    set(h_G,      'XData', Gx_v(k),             'YData', Gy_v(k));
    
    drawnow limitrate;
    pause(0.015);
    
    k = k + 1;
    if k > N
        k = 1;
    end
end