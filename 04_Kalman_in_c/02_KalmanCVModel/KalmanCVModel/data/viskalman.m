
clc; close all; clear;
A= load('out_from_vs_len800_1.txt');

t = A(:, 2);
px = A(:, 3);
pz = A(:, 4);

dt = mean(diff(t));
disp(sprintf('mean(dt) = %f sec', dt));

figure;
subplot(2,1,1);
plot(t,px);hold on;
plot(t,A(:, 5));
plot(t,A(:, 7));
title('gx');

subplot(2,1,2);
plot(t,pz); hold on;
plot(t,A(:, 6));
plot(t,A(:, 8));
title('gz');

figure;
plot(t,px);hold on;
plot(t,pz); 
legend('px', 'pz');
