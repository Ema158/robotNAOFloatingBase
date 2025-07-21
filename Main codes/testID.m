function [aBase,tau] = testID(I1,F,H,p1,C,qpp)
aBase = I1\(-F*qpp - p1);
tau = F'*aBase + H*qpp + C;
end