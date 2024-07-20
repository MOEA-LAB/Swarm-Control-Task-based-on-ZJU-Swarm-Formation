% MATLAB代码

% 清空所有图形、变量和控制台
clc;
clear;
close all;

% 定义x和y的范围
x_range = [-1.8, 1.8];
y_range = [-2, 2];

% 创建一个图形窗口
figure;
hold on;
axis([x_range, y_range]);
grid on;

% 初始化一个空数组来存储点的位置
points = zeros(7, 2);

% 提示用户点击九个点
disp('请在图形上点击九个点。');

% 循环九次获取用户点击的点
for i = 1:7
    [x, y] = ginput(1); % 获取点击的点
    plot(x, y, 'ro'); % 显示点击的点
    points(i, :) = [x, y]; % 将点的位置存储到数组中
    text(x, y, sprintf('(%0.2f, %0.2f)', x, y), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right'); % 显示坐标
end

% 显示所有点击的点的位置
disp('点击的点的位置：');
disp(points);

% 将点的位置列表返回
points
