%% Term Project: 2103-322 Design of Mechanism
% Symbolic Position & Trajectory Analysis: 6-Bar Chebyshev Straight-Line Mechanism
clear; clc; close all;
warning('off', 'symbolic:solve:SolutionsDependOnConditions');

%% 1. Define Mechanism Dimensions (A4 Model Scale & Wide-Stroke)
L_AC = 80;          % ระยะห่างจุดหมุนฐาน A ถึง C (mm)
L1   = 100;         % ความยาว Rocker ซ้าย AB (mm)
L2   = 100;         % ความยาว Rocker ขวา CD (mm)
L3   = 40;          % ความยาว Coupler บน BD (mm)
L_BG = 20;          % ตำแหน่งจุด G กึ่งกลาง Coupler (mm)

% พิกัดจุดหมุนตรึงบนฐาน
Ax = 0;      Ay = 0;
Cx = L_AC;   Cy = 0;
Ex = 150;    Ey = 15;   % จุดหมุนมอเตอร์ขับ E

% ชุดขับเคลื่อน Grashof Crank-Rocker
L4 = 26;                % ความยาว Crank EF (mm)
L5 = 118;               % ความยาวก้านส่งกำลัง FB (mm)

%% 2. Symbolic Position Analysis (Vector Loop-Closure Equations)
syms theta4 theta1 theta5 theta2 theta3 real

% Loop 1: Drive Dyad (E-F-B-A)
Fx = Ex + L4*cos(theta4);
Fy = Ey + L4*sin(theta4);
Bx = Ax + L1*cos(theta1);
By = Ay + L1*sin(theta1);

sol1 = solve([Bx == Fx + L5*cos(theta5), By == Fy + L5*sin(theta5)], [theta1, theta5]);
th1_sym = sol1.theta1(1); % เลือก Branch ยกตัวขึ้นด้านบน (Y > 0)

Bx_sym = subs(Bx, theta1, th1_sym);
By_sym = subs(By, theta1, th1_sym);

% Loop 2: Chebyshev 4-Bar Linkage (A-B-D-C)
Dx = Cx + L2*cos(theta2);
Dy = Cy + L2*sin(theta2);

sol2 = solve([Bx + L3*cos(theta3) == Dx, By + L3*sin(theta3) == Dy], [theta2, theta3]);
th2_sym = subs(sol2.theta2(2), theta1, th1_sym); % Crossed X-Configuration
th3_sym = subs(sol2.theta3(2), theta1, th1_sym);

Dx_sym = subs(Dx, theta2, th2_sym);
Dy_sym = subs(Dy, theta2, th2_sym);

% พิกัดจุดใช้งานปลายทาง Point G (Output Straight-Line Point)
Gx_sym = Bx_sym + L_BG*cos(th3_sym);
Gy_sym = By_sym + L_BG*sin(th3_sym);

%% 3. Numerical Evaluation & Straightness Efficiency Calculation
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

Bx_v = fn_Bx(th4_vals);
By_v = fn_By(th4_vals);
Dx_v = fn_Dx(th4_vals);
Dy_v = fn_Dy(th4_vals);
Fx_v = fn_Fx(th4_vals);
Fy_v = fn_Fy(th4_vals);
Gx_v = fn_Gx(th4_vals);
Gy_v = fn_Gy(th4_vals);

% คำนวณระยะชักและประสิทธิภาพของเส้นตรงแบบประมาณ (Straightness Efficiency)
stroke_X     = max(Gx_v) - min(Gx_v);          % ระยะการสไลด์แนวนอน (mm)
deviation_Y  = max(Gy_v) - min(Gy_v);          % ความคลาดเคลื่อนแนวดิ่งสูงสุด (mm)
mean_Y       = mean(Gy_v);                     % ระดับความสูงเฉลี่ยของเส้นตรง (mm)
eta_straight = (1 - (deviation_Y / stroke_X)) * 100; % ประสิทธิภาพความตรง (%)

fprintf('=== Straight-Line Performance Summary ===\n');
fprintf('Horizontal Stroke (Delta X) : %.2f mm\n', stroke_X);
fprintf('Vertical Deviation (Delta Y): %.3f mm\n', deviation_Y);
fprintf('Straightness Efficiency     : %.2f %%\n', eta_straight);

%% 4. Define Color Palette (Matching 3D CAD Model)
c_L1   = [0.68, 0.16, 0.16]; % Link 1 (AB): สีแดงเข้ม
c_L2   = [0.18, 0.49, 0.22]; % Link 2 (CD): สีเขียวเข้ม
c_L3   = [0.85, 0.65, 0.13]; % Link 3 (BD): สีเหลืองทอง
c_L4   = [0.15, 0.22, 0.60]; % Link 4 (EF): สีน้ำเงินเข้ม
c_L5   = [0.18, 0.18, 0.20]; % Link 5 (FB): สีดำ/เทาเข้ม
c_traj = [0.95, 0.25, 0.05]; % Trajectory G: สีส้มแดงสว่าง (ตัดกับก้านสีเหลืองทอง)

%% 5. Plot Figure 1: Mechanism Workspace & Straight-Line Trajectory
fig1 = figure('Name', 'Figure 1: Straight-Line Trajectory & Workspace', 'Color', 'w', 'Position', [60 80 880 580]);
hold on; grid on; axis equal;

% วาดกรอบอ้างอิงขนาดกระดาษ A4 (210 x 297 mm)
rectangle('Position', [-15, -15, 200, 135], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1);
text(-10, 114, 'A4 Mounting Board Boundary', 'Color', [0.4 0.4 0.4], 'FontSize', 9);

% พล็อตวิถีการเคลื่อนที่ของจุด G และข้อต่อต่างๆ
plot(Gx_v, Gy_v, '-', 'Color', c_traj, 'LineWidth', 3.5, 'DisplayName', sprintf('Trajectory Point G (Stroke = %.1f mm)', stroke_X));
plot([min(Gx_v) max(Gx_v)], [mean_Y mean_Y], ':k', 'LineWidth', 1.2, 'DisplayName', sprintf('Ideal Straight Line (Y = %.1f mm)', mean_Y));
plot(Bx_v, By_v, '--', 'Color', c_L1, 'LineWidth', 1, 'DisplayName', 'Joint B Path');
plot(Dx_v, Dy_v, '--', 'Color', c_L2, 'LineWidth', 1, 'DisplayName', 'Joint D Path');
plot(Fx_v, Fy_v, ':',  'Color', c_L4, 'LineWidth', 1.2, 'DisplayName', 'Crank Path F');
plot([Ax Cx Ex], [Ay Cy Ey], '^k', 'MarkerSize', 10, 'MarkerFaceColor', [0.8 0.8 0.8], 'DisplayName', 'Fixed Ground Pivots');

xlabel('X Position (mm)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Y Position (mm)', 'FontSize', 11, 'FontWeight', 'bold');
title(sprintf('6-Bar Chebyshev Mechanism: Approximate Straight-Line Generation (\\eta = %.2f%%)', eta_straight), 'FontSize', 12);
xlim([-20, 190]);
ylim([-20, 125]);
legend('Location', 'southeast');

%% 6. Plot Figure 2: Output Position & Straightness Accuracy Profile
fig2 = figure('Name', 'Figure 2: Output Position Profile', 'Color', 'w', 'Position', [960 80 720 580]);

subplot(2,1,1);
plot(th4_deg, Gx_v, '-', 'Color', c_L4, 'LineWidth', 2); grid on;
ylabel('Horizontal Position G_x (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title('Horizontal Sliding Stroke (G_x) vs Input Crank Angle \theta_4', 'FontSize', 11);
xlim([0 360]);

subplot(2,1,2);
plot(th4_deg, Gy_v, '-', 'Color', c_traj, 'LineWidth', 2); hold on;
yline(mean_Y, '--k', sprintf('Mean Height = %.2f mm', mean_Y), 'LineWidth', 1.2);
grid on;
xlabel('Input Crank Angle \theta_4 (deg)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Vertical Height G_y (mm)', 'FontSize', 10, 'FontWeight', 'bold');
title(sprintf('Vertical Deviation \\DeltaY = %.3f mm (Straightness Efficiency = %.2f%%)', deviation_Y, eta_straight), 'FontSize', 11);
xlim([0 360]);
ylim([mean_Y - 1, mean_Y + 1]);

%% 7. Infinite Continuous Real-Time Animation on Figure 1
figure(fig1);
hold on;

% สร้างเส้นชิ้นส่วนตามลำดับ Layer ให้ตรงกับชิ้นงาน 3D CAD
h_L2 = plot([Cx Dx_v(1)],      [Cy Dy_v(1)],      '-', 'Color', c_L2,   'LineWidth', 5.5, 'HandleVisibility', 'off'); % Link 2 (เขียว) อยู่ชั้นล่าง
h_L3 = plot([Bx_v(1) Dx_v(1)], [By_v(1) Dy_v(1)], '-', 'Color', c_L3,   'LineWidth', 5.5, 'HandleVisibility', 'off'); % Link 3 (เหลืองทอง)
h_L5 = plot([Fx_v(1) Bx_v(1)], [Fy_v(1) By_v(1)], '-', 'Color', c_L5,   'LineWidth', 5.5, 'HandleVisibility', 'off'); % Link 5 (ดำ/เทาเข้ม)
h_L1 = plot([Ax Bx_v(1)],      [Ay By_v(1)],      '-', 'Color', c_L1,   'LineWidth', 5.5, 'HandleVisibility', 'off'); % Link 1 (แดง) ไขว้ทับด้านบน
h_L4 = plot([Ex Fx_v(1)],      [Ey Fy_v(1)],      '-', 'Color', c_L4,   'LineWidth', 5.5, 'HandleVisibility', 'off'); % Link 4 (น้ำเงิน)
h_G  = plot(Gx_v(1),           Gy_v(1),           'o', 'Color', 'k',    'MarkerFaceColor', c_traj, 'MarkerSize', 9, 'HandleVisibility', 'off'); % จุด G

% วนลูปเคลื่อนที่ไปเรื่อยๆ ไม่มีหยุด (ปิดหน้าต่าง Figure 1 หรือกด Ctrl+C ใน Command Window เพื่อหยุด)
k = 1;
while ishandle(fig1)
    set(h_L1, 'XData', [Ax Bx_v(k)],      'YData', [Ay By_v(k)]);
    set(h_L2, 'XData', [Cx Dx_v(k)],      'YData', [Cy Dy_v(k)]);
    set(h_L3, 'XData', [Bx_v(k) Dx_v(k)], 'YData', [By_v(k) Dy_v(k)]);
    set(h_L4, 'XData', [Ex Fx_v(k)],      'YData', [Ey Fy_v(k)]);
    set(h_L5, 'XData', [Fx_v(k) Bx_v(k)], 'YData', [Fy_v(k) By_v(k)]);
    set(h_G,  'XData', Gx_v(k),           'YData', Gy_v(k));
    
    drawnow limitrate;
    pause(0.015);
    
    k = k + 1;
    if k > N
        k = 1; % วนกลับมาเริ่มมุม 0 องศาใหม่ต่อเนื่อง
    end
end