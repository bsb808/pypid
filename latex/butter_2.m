%%  Second Order Butterworth
wc = 10;
G = tf(wc^2,[1 sqrt(2)*wc wc^2]);
bode(G)
tstr = sprintf('Second-Order Butterworth: \\omega_c=%.1f rad/s',wc);
title(tstr)

%% Effect of Warping
eps = 0.1;
xx = linspace(0,10,100);
yy = cot(xx)
figure(2);
clf();
plot(xx,yy)
