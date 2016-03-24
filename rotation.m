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
offset=0; %Not Needed. Makes agents go in or out of circle

%Go toward consensus or what? Stay in place, go toward circle state
if(t<.1)
    R=[cos((pi+offset)/N) sin(((pi+offset)/N));-sin((pi+offset)/N) cos(((pi+offset)/N))];
end

%Archimeddes Spiral. Use for Archimedes Spiral of pattern
if(t>=.1)

    R=[cos(Alpha) sin(Alpha);-sin(Alpha) cos(Alpha)];
end


end