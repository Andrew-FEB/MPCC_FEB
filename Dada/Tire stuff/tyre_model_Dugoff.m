    function [F] = tyre_model_Dugoff(Fz, beta, s, mu, Cx, Cy, sign_Fx)

% author: Gert Heirman

% The Dugoff tyre model is being used.

%  Input
% -------
    % Fz : the normal force on the tyre (N)
    % beta : the slip angle of the tyre (rad)
    % s : thee longitudinal slip of the tyre (/)
    % mu : the friction coefficient tyre <-> road (/)
    % Cx : the longitudinal cornering stiffness of the tyre (N)
    % Cy : the lateral cornering stiffness of the tyre (N/rad)
    % sign_Fx : direction for longitudinal force (-1 or 1)

%  Output
% --------
    % F : the tyre forces
    % F(1) : F_x (force in x-direction) (N)
    % F(2) : F_y (force in y-direction) (N)
    % F(3) : F_z (force in z-direction) (N)
    % F(4) : M_z (moment in z-direction) (N)
    % These variables are defined in a frame attached to the wheel. 
    % This frame follows the wheel steering angle, but doesn't follow
    % the other rotations of the wheel.

if s==1 % this if is necessary, because otherwise Matlab will crash on a 'zero divided by zero'-limit problem
    F(1) = -Cx*mu*Fz/sqrt(Cx^2+(Cy*tan(beta))^2);
    F(2) = -Cy*tan(beta)*mu*Fz/sqrt(Cx^2+(Cy*tan(beta))^2);
    F(3) = Fz;
    F(4) = 0; % No aligning moment assumed
elseif (s==0 && tan(beta)==0) % this if is neccesary, otherwise a divide by zero error
    F(1) = 0;
    F(2) = 0;
    F(3) = Fz;
    F(4) = 0; % No aligning moment assumed
else
    % Auxiliary variable lambda for the Dugoff tyre model
    lambda = mu*Fz*(1-s)/(2*sqrt((Cx*s)^2+(Cy*tan(beta))^2));

    % Auxiliary function f(lambda) for the Dugoff tyre model
    if lambda < 1 % saturation effect
        f_lambda = (2-lambda)*lambda;
    else % in linear part of tire behaviour: no saturation effect
        f_lambda = 1;
    end

    F = zeros(4,1);

    if abs(beta)<= pi/2% normal case
        F(1) = -Cx*s*f_lambda/(1-s);
        F(2) = -Cy*tan(beta)*f_lambda/(1-s); 
        F(3) = Fz;
        F(4) = 0; % No aligning moment assumed
    else %'extreme' case
        F(1) = -Cx*s*f_lambda/(1-s);
        F(2) = Cy*tan(beta)*f_lambda/(1-s); 
        F(3) = Fz;
        F(4) = 0; % No aligning moment assumed
    end
end

if Fz <= 0
    F(1) = 0;
    F(2) = 0;
    F(3) = Fz;
    F(4) = 0;
    disp('Warning: the tyre loses contact with the road')
end

if sign_Fx ~=0
    F(1) = -sign_Fx*F(1);
end



