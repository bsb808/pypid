
%% Freq. Resp. of Difference Eqn.

% Cut-off freq.
wc = 10;  % rad/s
dt = 1/1000;  % sample time
fs = 1/dt;  % sample freq.
% Define transfer function: H(z)
% Bilinear - first order
c = cot(wc*dt/2);
num = [1, 1];
den = [1+c, 1-c];
% Simple finite difference
alpha = (wc*dt)/(wc*dt+1);
num2 = alpha;
den2 = [1, -(1-alpha)];
% Second order Butter
num3 = [1,2,1];
den3 = [c^2+sqrt(2)*c+1,...
    -2*(c^2-1),...
    c^2-sqrt(2)*c+1];

% Determine normalized frequencies 
w = logspace(log10(wc/100),log10(wc*100),1024);  % rad/s
% freqz considers frequency in rad/sample, so
w = w/fs;  % rad/sample

% Freq Resp
[h]=freqz(num,den,w);
[h2]=freqz(num2,den2,w);
[h3]=freqz(num3,den3,w);

% Convert back to real frequencies
%ff = w*fn;  % in hz
%ww = ff*(2*pi);
ww = w*fs;  % rad/sample to rad/s



% Plot

figure(1);
clf();
ax1=subplot(211)
semilogx(ww,20*log10(abs(h)),'b','linewidth',2)
hold on
semilogx(ww,20*log10(abs(h2)),'r--','linewidth',2)
semilogx(ww,20*log10(abs(h3)),'k')
grid on
ylabel('Mag. [dB]')
ylim([-20,3])
title(sprintf('Freq. Resp. Digital Filter \\omega_c=%.1f rad/s',wc));
legend('1 order: Bilinear','1 order: Alpha','2 order: Butterworth','location','southwest')
ax2=subplot(212)
semilogx(ww,angle(h)*180/pi,'b','linewidth',2)
hold on
semilogx(ww,angle(h2)*180/pi,'r--','linewidth',2)
semilogx(ww,angle(h3)*180/pi,'k')
grid on
ylabel('Phase [deg]')
xlabel('Freq. [rad/s]')
linkaxes([ax1,ax2],'x')
xlim([min(ww),max(ww)])
