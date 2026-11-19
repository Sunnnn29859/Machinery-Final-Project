%% --- 1. ดึงเวลาและข้อมูลดิบจาก Simscape ---
t_sim = squeeze(out.t_sim); 

% ใช้ try-catch เพื่อความปลอดภัยในการดึงข้อมูลจาก out
try
    raw_Gx = squeeze(out.Gx_sim.Data); 
    raw_Gy = squeeze(out.Gy_sim.Data);
catch
    raw_Gx = squeeze(out.Gx_sim); 
    raw_Gy = squeeze(out.Gy_sim);
end

%% --- 2. ดึงข้อมูลพิกัด (ดึงตรงๆ ไม่ต้องสลับแกน) ---
Gx_sim = raw_Gx;  % แกนราบ (Horizontal Deviation)
Gy_sim = raw_Gy;  % แกนดิ่ง (Vertical Stroke)

%% --- 3. คำนวณค่าวิเคราะห์เส้นตรง (Simscape Only) ---
stroke_y  = max(Gy_sim) - min(Gy_sim);          % ระยะชักแนวดิ่ง
delta_x   = max(Gx_sim) - min(Gx_sim);          % ความเบี่ยงเบนแนวราบ
X_mean    = mean(Gx_sim);                       % เส้นแกนอุดมคติ
max_dev   = max(abs(Gx_sim - X_mean));          % คลาดเคลื่อนสูงสุดจากแกน (+/-)

% ป้องกัน Error กรณีที่กลไกล็อกจน Stroke = 0
if stroke_y > 0
    eff = (1 - (delta_x / stroke_y)) * 100; 
else
    eff = 0; 
end

%% --- 4. เตรียมข้อมูลมุมข้อเหวี่ยงสำหรับพล็อตกราฟ ---
omega4 = 2; % ความเร็วมอเตอร์ rad/s
initial_angle = 3.14159; 

% เช็คว่ามีการดึงมุม q_sim ออกมาจากหน้าต่าง Simulink หรือไม่
if exist('out', 'var') && isprop(out, 'q_sim')
    try
        theta4_deg = squeeze(out.q_sim.Data);
    catch
        theta4_deg = squeeze(out.q_sim);
    end
    % แปลงมุมจาก rad เป็น deg หากเซนเซอร์ตั้งค่าออกเป็น rad
    if max(theta4_deg) < 10
        theta4_deg = rad2deg(theta4_deg);
    end
else
    % ถ้าไม่ได้ส่งค่าเซนเซอร์มุมมา ให้จำลองมุมจากเวลา t_sim
    theta4_rad = (omega4 * t_sim) + initial_angle;
    theta4_deg = mod(rad2deg(theta4_rad - initial_angle), 360);
end

% ตัดข้อมูลให้เหลือ 1 รอบการหมุน เพื่อให้เส้นกราฟคมชัด ไม่ลากทับกัน
T_period = (2*pi) / omega4;
idx_1rev = find(t_sim <= T_period);
if isempty(idx_1rev)
    idx_1rev = 1:length(t_sim);
end

[th4_sorted, sort_idx] = sort(theta4_deg(idx_1rev));
Gy_plot = Gy_sim(idx_1rev(sort_idx));
Gx_plot = Gx_sim(idx_1rev(sort_idx));

%% --- 5. พล็อตกราฟ (Simscape Only) ---
% === Figure 1: กราฟวิเคราะห์ตามมุมข้อเหวี่ยง ===
figure('Name', 'Simscape Kinematic Analysis', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(th4_sorted, Gy_plot, 'r-', 'LineWidth', 2.5);
title(sprintf('Vertical Lift Height (G_y) vs Crank Angle (Stroke = %.2f mm)', stroke_y));
xlabel('Crank Angle \theta_4 (deg)'); ylabel('Vertical Height G_y (mm)');
xlim([0 360]); grid on;

subplot(2,1,2);
plot(th4_sorted, Gx_plot, 'r-', 'LineWidth', 2.5); hold on;
yline(X_mean, 'b--', sprintf('Ideal Vertical Axis X = %.2f mm', X_mean), 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right');
title(sprintf('Horizontal Deviation (\\DeltaX) = %.3f mm | Efficiency = %.2f%%', delta_x, eff));
xlabel('Crank Angle \theta_4 (deg)'); ylabel('Horizontal Position G_x (mm)');
xlim([0 360]); 
% ปรับสเกลแกน Y ให้เห็นคลื่น Error ชัดๆ
ylim([X_mean - max(delta_x)*1.5, X_mean + max(delta_x)*1.5]); 
grid on; hold off;

% === Figure 2: กราฟเส้นทางวิถี 2D (Trajectory) ===
figure('Name', 'Simscape Vertical Trajectory', 'Position', [950, 100, 400, 500]);
plot(Gx_sim, Gy_sim, 'r-', 'LineWidth', 2.5); hold on;
xline(X_mean, 'k:', 'Ideal Vertical', 'LineWidth', 1.5);
title(sprintf('Vertical Trajectory (Efficiency = %.2f%%)', eff));
xlabel('X Position (mm)'); ylabel('Y Position (mm)');
axis equal; grid on; hold off;

%% --- 6. พิมพ์ผลลัพธ์ลง Command Window ---
fprintf('\n======================================================\n');
fprintf('  สรุปผลการวิเคราะห์วิถีการเคลื่อนที่แนวตั้ง (Simscape Model)\n');
fprintf('======================================================\n');
fprintf('1. เส้นแกนตั้งอุดมคติ (X_mean)      : %10.3f mm\n', X_mean);
fprintf('2. ระยะยกตัวแนวดิ่งสูงสุด (Y_max)    : %10.3f mm\n', max(Gy_sim));
fprintf('3. ระยะยกตัวแนวดิ่งต่ำสุด (Y_min)    : %10.3f mm\n', min(Gy_sim));
fprintf('4. ระยะชักแนวดิ่ง (Stroke Delta Y)  : %10.3f mm\n', stroke_y);
fprintf('5. ความเบี่ยงเบนแนวราบ (Delta X)    : %10.4f mm\n', delta_x);
fprintf('6. ความคลาดเคลื่อนจากแกนกลาง (+/-) : %10.4f mm\n', max_dev);
fprintf('7. ประสิทธิภาพความตรง (Efficiency) : %10.2f %%\n', eff);
fprintf('======================================================\n');