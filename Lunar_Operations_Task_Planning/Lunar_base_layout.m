close all
figure('Units', 'normalized', 'Position', [0 0 1 1], ...
           'Color', 'black', 'MenuBar', 'none', ...
           'ToolBar', 'none', 'WindowState', 'fullscreen');
ax = axes('Color', 'black', 'Position', [0 0 1 1]);
    hold on;

r = 80;
[x, y] = meshgrid(-r:0.1:r);
z = sqrt(r^2-x.^2-y.^2);
mesh(real(z));
hold on 
