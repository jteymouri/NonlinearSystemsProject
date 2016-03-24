%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rotation.m
%
% Description: Function call to show on the graph the position of the
% agents and the radius of the agent
%
% Inputs: rotation(N,t,Alpha)
% N = Number of agents
% t = time
% Alpha = 
%
% Outputs: R
% R = Rotation Matrix in form [cos(a) sin(a); -sin(a) cos(a)];
%
% Note: Rotation Matrix gets multipled by the agents position to determine
% where they should be headed. Very important matrix as far as motion
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R= rotation(N,t,Alpha)
    R=[cos(Alpha) sin(Alpha);-sin(Alpha) cos(Alpha)];

end