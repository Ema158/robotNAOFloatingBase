function [aBase,tau] = testID(I1,F,H,p1,C)
% n = 24+6;
% qpp = zeros(24,1);
% qpp(1) = -0.1;
% A = zeros(n,n);
% A(1:6,1:6) = I1;
% A(7:end,1:6) = F';
% A(7:end,7:end) = H;
% b = [-F*qpp - p1; -H*qpp - C];
% x = A\b;
% aBase = x(1:6);
% tau = F'*aBase + H*qpp + C;

n = 24+6;
qpp = zeros(24,1);
qpp(1) = -0.1;
A = zeros(n,n);
A(1:6,1:6) = I1;
A(7:end,1:6) = F';
A(7:end,7:end) = -eye(24,24);
b = [-F*qpp - p1; -H*qpp - C];
x = A\b;
aBase = x(1:6);
% tau = F'*aBase + H*qpp + C;
tau = x(7:end);
end