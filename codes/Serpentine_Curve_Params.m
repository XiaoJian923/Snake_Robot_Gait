% 作者: 海南大学21级本科生yyy
a_values = [pi/3, pi/2, 2*pi/3]; % a 的不同取值
b_values = [2*pi, 6*pi, 10*pi]; % b 的不同取值
c_values = [0, pi/2, pi]; % c 的不同取值

% 定义 s 的范围，适当扩大范围以展示更多曲线特征
s = 0:0.01:1;

% 绘制 a 取不同值的曲线（第一个 figure）
figure(1);
b_fixed = 2*pi;
c_fixed = 0;
colors = ['b', 'r', 'g']; % 为 a 变化的图定义颜色：蓝、红、绿
linestyles = {'-', '--', '*'};
for i = 1:length(a_values)
    a = a_values(i);
    xi = @(sigma) a*cos(b_fixed*sigma)+c_fixed*sigma;
    % 提高积分精度
    x = @(s) integral(@(sigma) cos(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    y = @(s) integral(@(sigma) sin(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    x_vals = arrayfun(x, s);
    y_vals = arrayfun(y, s);
    plot(x_vals, y_vals, linestyles{i}, 'DisplayName', ['a = ', num2str(a)], 'Color', colors(i)); % 设置颜色
    hold on;
end
xlabel('');
ylabel('');
title('');
legend('Location', 'northwest');
% 调整绘图范围
xlim([0, 0.8]);
ylim([-0.4, 0.4]);
text(0.2, 0.2, ['b = ', num2str(b_fixed)], 'HorizontalAlignment', 'center');
text(0.2, 0.1, ['c = ', num2str(c_fixed)], 'HorizontalAlignment', 'center');
hold off;

% 绘制 b 取不同值的曲线（第二个 figure）
figure(2);
a_fixed = pi/2;
c_fixed = 0;
colors = ['b', 'r', 'g']; % 为 b 变化的图定义颜色：洋红、黄、青
linestyles = {'-', '--', '*'};
for i = 1:length(b_values)
    b = b_values(i);
    xi = @(sigma) a_fixed*cos(b*sigma)+c_fixed*sigma;
    % 提高积分精度
    x = @(s) integral(@(sigma) cos(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    y = @(s) integral(@(sigma) sin(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    x_vals = arrayfun(x, s);
    y_vals = arrayfun(y, s);
    plot(x_vals, y_vals, linestyles{i}, 'DisplayName', ['b = ', num2str(b)], 'Color', colors(i)); % 设置颜色
    hold on;
end
xlabel('');
ylabel('');
title('');
legend('Location', 'northwest');
xlim([0, 0.6]);
ylim([-0.2, 0.2]);
text(0.2, 0.1, ['a = ', num2str(a_fixed)], 'HorizontalAlignment', 'center');
text(0.2, 0.05, ['c = ', num2str(c_fixed)], 'HorizontalAlignment', 'center');
hold off;

% 绘制 c 取不同值的曲线（第三个 figure）
figure(3);
a_fixed = pi/2;
b_fixed = 10*pi;
colors = ['b', 'r', 'g']; % 为 c 变化的图定义颜色：黑、白、蓝
linestyles = {'-', '--', '*'};
for i = 1:length(c_values)
    c = c_values(i);
    xi = @(sigma) a_fixed*cos(b_fixed*sigma)+c*sigma;
    % 提高积分精度
    x = @(s) integral(@(sigma) cos(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    y = @(s) integral(@(sigma) sin(xi(sigma)), 0, s, 'ArrayValued', true, 'RelTol',1e-8,'AbsTol',1e-8);
    x_vals = arrayfun(x, s);
    y_vals = arrayfun(y, s);
    plot(x_vals, y_vals, linestyles{i}, 'DisplayName', ['c = ', num2str(c)], 'Color', colors(i)); % 设置颜色
    hold on;
end
xlabel('');
ylabel('');
title('');
legend('Location', 'northwest');
xlim([0, 0.6]);
ylim([-0.1, 0.4]);
text(0.2, 0.25, ['a = ', num2str(a_fixed)], 'HorizontalAlignment', 'center');
text(0.2, 0.2, ['b = ', num2str(b_fixed)], 'HorizontalAlignment', 'center');
hold off;
