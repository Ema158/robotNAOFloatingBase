
global OptionContVar
if isempty(OptionContVar)  % If OptionContVar is empty we decide to chose the first option as default    
    error(['The variable "OptionContVar" is empty, you need to choose which files are used to build \n',...
           'the desired trajectories "hd(Phi)", defined in the file "OptionDesiredTrajectory.m"'],OptionContVar);
end
    
switch Case
    case 'position'    
        switch OptionContVar
            case 1
                hd = hd_Polyn(gait_parameters,t);
            otherwise
                error(['The value of the variable "OptionContVar" is %d. However this option is no \n',...
                       'valid in the file "OptionDesiredTrajectory.m"'],OptionContVar);       
        end
    %  *    *     *     *      *    *     *     *     *    *     *     *        
    case 'velocity'        
        switch OptionContVar
            case 1
                Phip = [qfp; 1];
                phiDim = length(Phip) - 6; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix Jd
                dhd_Phi(:,1) = zeros(24,1); % partial derivative of "hd" w.r.t. x
                dhd_Phi(:,2) = zeros(24,1); % partial derivative of "hd" w.r.t. y
                dhd_Phi(:,3) = zeros(24,1); % partial derivative of "hd" w.r.t. z
                dhd_Phi(:,4) = zeros(24,1); % partial derivative of "hd" w.r.t. Roll
                dhd_Phi(:,5) = zeros(24,1); % partial derivative of "hd" w.r.t. Pitch
                dhd_Phi(:,6) = zeros(24,1); % partial derivative of "hd" w.r.t. Yaw
                dhd_Phi(:,7) = hpd_Polyn_t(gait_parameters,t); % partial derivative of "hd" w.r.t. phi1 = t
        end        
    %  *    *     *     *      *    *     *     *     *    *     *     *        
    case 'acceleration'
        switch OptionContVar
            case 1
                Phip = [qfp; 1];
                phiDim = length(Phip) - 6; %dimension of vector "phi", where Phi = [qf^T phi^T]^T
                % terms for building matrix "Jdp"...
                dPhi_dhd_x = zeros(24,6+phiDim);
                dPhi_dhd_y = zeros(24,6+phiDim);
                dPhi_dhd_z = zeros(24,6+phiDim);
                dPhi_dhd_roll = zeros(24,6+phiDim);
                dPhi_dhd_pitch = zeros(24,6+phiDim);
                dPhi_dhd_yaw = zeros(24,6+phiDim);

                dPhi_dhd_t = [zeros(24,1), zeros(24,1), zeros(24,1), zeros(24,1),...
                             zeros(24,1), zeros(24,1), hppd_Polyn_t(gait_parameters,t)];
                dt_dhd_Phi = [dPhi_dhd_x*Phip, dPhi_dhd_y*Phip, dPhi_dhd_z*Phip,...
                              dPhi_dhd_roll*Phip, dPhi_dhd_pitch*Phip, dPhi_dhd_yaw*Phip, dPhi_dhd_t*Phip];           
        end           
    %  *    *     *     *      *    *     *     *     *    *     *     *       
   
   
end