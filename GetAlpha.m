%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GetAlpha.m
%
% Description: Function call to determine the alpha for the Archimedes
% spiral. Using this alpha in the rotation matrix will produce spiralic
% motion along previously covered paths
%
% Inputs: GetAlpha(N,d,i,x)
% N = Number of agents
% d = diameter of each agents sensors, or otherwise covered area
%
% Outputs: Alpha
% Alpha = Constant number for the Archimedes spiral. Used in rotation mat
%
% Note: d is very important here. In the static case we get a perfect
% Archimedes spiral, but we may be altering this number dynamically to
% change the motion of the spirals for specific goals (To be determined).
%
% Note 2: Each agent will have a different alpha value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[Alpha]=GetAlpha(N,d,i,X)

if (i<N) %for N-1 Agents    
    Points = [X(1,i+1),X(2,i+1);X(1,i),X(2,i)];
end

if (i==N) %for the Nth Agent
    Points = [X(1,1),X(2,1);X(1,i),X(2,i)]; %back to N1
end

Zed = pdist(Points,'euclidean'); %Euclidean distance between all X,Y

%Archimedes Spiral
G = ((2*d)/Zed)*(sin(pi/N)/(pi/N));
Alpha = (pi/N) + atan(G); %solves for Alpha

end