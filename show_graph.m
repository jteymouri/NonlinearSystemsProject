%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% show_graph.m
%
% Description: Function call to show on the graph the position of the
% agents and the radius of the agent
%
% Inputs: show_graph(X,N,d,t,Radius)
% X = Position Vector of each agent
% N = Number of agents
% AxisL = Axis Length total
% t = time
% Radius = Radius length for ball graph
%
% Outputs: None, plots graph on figure
%
% Note: rectangle function if called to plot the circles around each agent.
% The rectangle function is used with high curvature and plots starting
% from the bottom left corner. bleft is a variable that converts the
% position to the bottom left for that proper plotting of the rectangle
% function. Curvature = [1 1] for a circle. 
%
% Note2: This program assums a .1 diameter (.05 Radius) ball around the
% agent. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function[]=show_graph(X,N,AxisL,~,Radius)
figure(1);
hold on;

for i=1:N;
    plot(X(1,i),X(2,i),'.','color','r','MarkerSize',15);
%     bleft = [X(1,i)-Radius X(2,i)-Radius Radius Radius];
%     rectangle('Position',bleft,'Curvature',[1 1],'FaceColor',[0 0 0])
end;

axis([0,AxisL,0,AxisL]);
drawnow;
end