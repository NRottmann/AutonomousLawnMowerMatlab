function [u] = pdControl(r,dr,rT,drT,phi,dphi)
% This a simple pd controller for the lawn mower
%
% Equation
%   Fdes = -Kp*ep - Kv*ev
%   v = u1 = Fdes * xB
%   xB = [cos(phi) sin(phi)]^T
%   ep = r - rT
%   ev = dr - drT
%
%   w = u2 = -Kr*er - Kw*ew
%   er = phi - phi_des
%   ew = dphi - dphi_des
%   phi_des = atan2(y_ep, x_ep)
%   
% Syntax:
%           [u] = pdControl(r,dr,rT,drT,phi)
%
% Input:
%   r:      Actual estimated position of the vehicle, [x; y];
%   dr:     Velocity of the vehicle, [dx; dy];
%   rT:     Desired position for the vehicle, [xT; yT];
%   drT:    Desired Velocity for the vehicle, [dxT; dyT];
%   phi:    Actual estimated orientation of the vehicle
%   dphi:   Angular velocity of the vehicle
%
% Output:
%   u:      Control inputs for motor, [v; w]
%
% Date:     01.10.2018
% Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)

% Check for correct dimensions
if (size(rT) ~= [2 1])
    error('Size of input rT is not correct!')
end
if (size(r) ~= [2 1])
    error('Size of input r is not correct!')
end
if (size(drT) ~= [2 1])
    error('Size of input drT is not correct!')
end
if (size(dr) ~= [2 1])
    error('Size of input dr is not correct!')
end
if (~isscalar(phi))
    error('phi is not a scalar!')
end
if (~isscalar(dphi))
    error('dphi is not a scalar!')
end

% Control Parameters
out = get_config('PDControl');
Kp = out.Kp;
Kv = out.Kv;
Kr = out.Kr;
Kw = out.Kw;

% Calculate errors
ep = r - rT;
ev = dr - drT;

% Evaluate angle error, therefore transform both angles to [0 2pi]
phi = phi - floor(phi/(2*pi))*2*pi;     % phi is allocated, thus we project it onto [0 2pi]
phi_des = atan2(-ep(2), -ep(1));        % phi_des from atan2 from [-pi pi], thus projection onto [0 2pi] required
if phi_des < 0
    phi_des = 2*pi + phi_des;
end

% Calculate error and pick shorter direction to turn to
er = phi - phi_des;
if abs(er) > pi         % other direction is shorter
    er = -er;
end

% Control algorithm
v_des = -Kp*ep - Kv*ev;
x_B = [cos(phi); sin(phi)];
u1 = v_des' * x_B;

ew = dphi;
u2 = -Kr*er - Kw*ew;

% Ensure that we do not drive backwards
if u1 < 0
    u1 = 0;
end

% Publish commands
u = [u1; u2];
  


end

