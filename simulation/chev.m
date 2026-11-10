%% Term Project: 2103-322 Design of Mechanism
% Kinematic Analysis & Simulation of 6-Bar Chebyshev Straight-Line Mechanism
clear; clc; close all;

warning('off', 'symbolic:solve:SolutionsDependOnConditions');

%% 1. Define Mechanism Dimensions (Wide-Stroke & Grashof Crank-Rocker)
% สัดส่วน 4 บาร์ Chebyshev คงที่เพื่อสร้างเส้นตรงแนวนอน
L_AC = 80;          % ระยะห่างฐาน A ถึง C
L1   = 100;         % ความยาวลิงก์ 1 (AB - Rocker ซ้าย)
L2   = 100;         % ความยาวลิงก์ 2 (CD - Rocker ขวา)
L3   = 40;          % ความยาวลิงก์ 3 (BD - Coupler บน)
L_BG = 20;          % กึ่งกลาง Coupler สำหรับจุด G

% จุดหมุนตรึงบนฐาน
Ax = 0;      Ay = 0;
Cx = L_AC;   Cy = 0;
Ex = 150;    Ey = 15;   % ตำแหน่งฐานมอเตอร์ขับ Crank E (ขยับออกเพื่อขยายมุม)

% ปรับขนาดตัวขับ Dyad เพื่อเพิ่มช่วงชัก (Stroke) ให้กวาดได้กว้างขึ้น
L4 = 26;                % ความยาว Crank ลิงก์ 4 (EF) - เพิ่มเพื่อขยายระยะชัก
L5 = 118;               % ความยาวก้านส่งกำลัง ลิงก์ 5 (FB)

omega4 = 2*pi;          % ความเร็วเชิงมุมของ Crank (rad/s)
alpha4 = 0;             % ความเร่งเชิงมุมของ Crank (rad/s^2)

%% 2. Declare Symbolic Variables
syms theta4 theta1 theta5 theta2 theta3 real

%% 3. Symbolic Position Analysis - Loop 1 (Drive Dyad Loop: A-B-F-E)
Fx = Ex + L4*cos(theta4);
Fy = Ey + L4*sin(theta4);

Bx = Ax + L1*cos(theta1);
By = Ay + L1*sin(theta1);

eq1_x = (Bx == Fx + L5*cos(theta5));
eq1_y = (By == Fy + L5*sin(theta5));

disp('Solving Loop 1 Symbolically...');
sol1 = solve([eq1_x, eq1_y], [theta1, theta5]);

% เลือก Branch 1 เพื่อให้ก้านยกตัวขึ้นด้านบน (Y > 0)
branch1 = 1;
th1_sym = sol1.theta1(branch1);
th5_sym = sol1.theta5(branch1);

Bx_sym = subs(Bx, theta1, th1_sym);
By_sym = subs(By, theta1, th1_sym);

%% 4. Symbolic Position Analysis - Loop 2 (Chebyshev 4-Bar Loop: A-B-D-C)
Dx = Cx + L2*cos(theta2);
Dy = Cy + L2*sin(theta2);

eq2_x = (Bx + L3*cos(theta3) == Dx);
eq2_y = (By + L3*sin(theta3) == Dy);

disp('Solving Loop 2 Symbolically...');
sol2 = solve([eq2_x, eq2_y], [theta2, theta3]);

% เลือก Branch 2 สำหรับ Crossed Configuration ของ Chebyshev
branch2 = 2;
th2_sym = subs(sol2.theta2(branch2), theta1, th1_sym);
th3_sym = subs(sol2.theta3(branch2), theta1, th1_sym);

Dx_sym = subs(Dx, theta2, th2_sym);
Dy_sym = subs(Dy, theta2, th2_sym);

Gx_sym = Bx_sym + L_BG*cos(th3_sym);
Gy_sym = By_sym + L_BG*sin(th3_sym);

%% 5. Symbolic Velocity & Acceleration Analysis (Differentiation)
disp('Computing Symbolic Velocities and Accelerations of Point G...');
VGx_sym = diff(Gx_sym, theta4) * omega4;
VGy_sym = diff(Gy_sym, theta4) * omega4;

AGx_sym = diff(VGx_sym, theta4) * omega4 + diff(Gx_sym, theta4) * alpha4;
AGy_sym = diff(VGy_sym, theta4) * omega4 + diff(Gy_sym, theta4) * alpha4;

%% 6. Numerical Evaluation over 1 Full Cycle (0 to 360 deg)
N = 180;
th4_vals = linspace(0, 2*pi, N);

fn_Bx  = matlabFunction(Bx_sym,  'Vars', theta4);
fn_By  = matlabFunction(By_sym,  'Vars', theta4);
fn_Dx  = matlabFunction(Dx_sym,  'Vars', theta4);
fn_Dy  = matlabFunction(Dy_sym,  'Vars', theta4);
fn_Fx  = matlabFunction(Fx,      'Vars', theta4);
fn_Fy  = matlabFunction(Fy,      'Vars', theta4);
fn_Gx  = matlabFunction(Gx_sym,  'Vars', theta4);
fn_Gy  = matlabFunction(Gy_sym,  'Vars', theta4);

fn_VGx = matlabFunction(VGx_sym, 'Vars', theta4);
fn_VGy = matlabFunction(VGy_sym, 'Vars', theta4);
fn_AGx = matlabFunction(AGx_sym, 'Vars', theta4);
fn_AGy = matlabFunction(AGy_sym, 'Vars', theta4);

Bx_v = fn_Bx(th4_vals);  By_v = fn_By(th4_vals);
Dx_v = fn_Dx(th4_vals);  Dy_v = fn_Dy(th4_vals);
Fx_v = fn_Fx(th4_vals);  Fy_v = fn_Fy(th4_vals);
Gx_v = fn_Gx(th4_vals);  Gy_v = fn_Gy(th4_vals);

VGx_v = fn_VGx(th4_vals); VGy_v = fn_VGy(th4_vals);
AGx_v = fn_AGx(th4_vals); AGy_v = fn_AGy(th4_vals);

%% 7. Plot Figure 1: Workspace & Static Trajectories
fig1 = figure('Name', 'Figure 1: Kinematics & Workspace Analysis', 'Color', 'w', 'Position', [60 60 950 620]);
hold on; grid on; axis equal;

plot(Gx_v, Gy_v, '-r', 'LineWidth', 2.8, 'DisplayName', 'Trajectory Point G (Straight-Line)');
plot(Bx_v, By_v, '--b', 'LineWidth', 1.2, 'DisplayName', 'Path B');
plot(Dx_v, Dy_v, '--c', 'LineWidth', 1.2, 'DisplayName', 'Path D');
plot(Fx_v, Fy_v, ':m',  'LineWidth', 1.2, 'DisplayName', 'Crank Path F');
plot([Ax Cx Ex], [Ay Cy Ey], '^k', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Ground Pivots');

xlabel('X Position (mm)', 'FontSize', 11);
ylabel('Y Position (mm)', 'FontSize', 11);
title('6-Bar Chebyshev Straight-Line Mechanism Workspace (Wide Stroke)', 'FontSize', 12);
xlim([-15, 185]);
ylim([-15, 125]);
legend('Location', 'best');

%% 8. Plot Figure 2: Kinematic Plots of Point G
fig2 = figure('Name', 'Figure 2: Kinematic Analysis of Output Point G', 'Color', 'w', 'Position', [1020 60 780 720]);

subplot(3,1,1);
plot(rad2deg(th4_vals), Gx_v, '-b', 'LineWidth', 1.6); hold on;
plot(rad2deg(th4_vals), Gy_v, '-r', 'LineWidth', 1.6); grid on;
ylabel('Position (mm)'); title('Position of Point G vs Input Crank Angle \theta_4');
legend('G_x', 'G_y (Straight-line height)');

subplot(3,1,2);
plot(rad2deg(th4_vals), VGx_v, '-b', 'LineWidth', 1.6); hold on;
plot(rad2deg(th4_vals), VGy_v, '-r', 'LineWidth', 1.6); grid on;
ylabel('Velocity (mm/s)'); title('Velocity of Point G vs Input Crank Angle \theta_4');
legend('V_{Gx}', 'V_{Gy}');

subplot(3,1,3);
plot(rad2deg(th4_vals), AGx_v, '-b', 'LineWidth', 1.6); hold on;
plot(rad2deg(th4_vals), AGy_v, '-r', 'LineWidth', 1.6); grid on;
xlabel('Input Crank Angle \theta_4 (deg)'); ylabel('Acceleration (mm/s^2)');
title('Acceleration of Point G vs Input Crank Angle \theta_4');
legend('A_{Gx}', 'A_{Gy}');

%% 9. Real-Time Animation (Runs on Figure 1)
% ระบุเป้าหมายให้วาดบน Figure 1 เท่านั้น ป้องกันการวาดทับกราฟความเร่ง
figure(fig1);
hold on;

% สร้าง Plot Handles สำหรับอัปเดตก้านแต่ละท่อน
h_L1 = plot([Ax Bx_v(1)], [Ay By_v(1)], '-b', 'LineWidth', 2.5);
h_L2 = plot([Cx Dx_v(1)], [Cy Dy_v(1)], '-c', 'LineWidth', 2.5);
h_L3 = plot([Bx_v(1) Dx_v(1)], [By_v(1) Dy_v(1)], '-g', 'LineWidth', 2.5);
h_L4 = plot([Ex Fx_v(1)], [Ey Fy_v(1)], '-m', 'LineWidth', 2.5);
h_L5 = plot([Fx_v(1) Bx_v(1)], [Fy_v(1) By_v(1)], 'Color', [0.7 0.5 0.3], 'LineWidth', 2);
h_G  = plot(Gx_v(1), Gy_v(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% วนลูปแอนิเมชัน 3 รอบ
num_cycles = 3;
for cycle = 1:num_cycles
    for k = 1:N
        set(h_L1, 'XData', [Ax Bx_v(k)],       'YData', [Ay By_v(k)]);
        set(h_L2, 'XData', [Cx Dx_v(k)],       'YData', [Cy Dy_v(k)]);
        set(h_L3, 'XData', [Bx_v(k) Dx_v(k)],  'YData', [By_v(k) Dy_v(k)]);
        set(h_L4, 'XData', [Ex Fx_v(k)],       'YData', [Ey Fy_v(k)]);
        set(h_L5, 'XData', [Fx_v(k) Bx_v(k)],  'YData', [Fy_v(k) By_v(k)]);
        set(h_G,  'XData', Gx_v(k),            'YData', Gy_v(k));
        
        drawnow limitrate;
        pause(0.015);
    end
end