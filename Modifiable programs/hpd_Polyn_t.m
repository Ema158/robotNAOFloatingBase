function hp_d = hpd_Polyn_t(gait_parameters,t)
PolyCoeff = gait_parameters.PolyCoeff;

% Velocity
% ------------------------
hp_d = zeros(24,1);
for i=1:24
    hp_d(i) =  polyval(polyder(PolyCoeff.(['hd', int2str(i)]) ),t);
end
