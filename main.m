% Initialize Matlab API with the desired scenario
% Choose from the following
% 1. formation+assignment
% 2. coverage
% 3. leader-follower
% 4. consensus
%r = robotariumMatlabAPI('formation+assignment');
r = robotariumMatlabAPI();  %run it with no scenario
r.initializeSubscribers();
r.setSimulationMode(true);

% Get list of available robots
robots = r.getAvailableRobots();
N      = length(robots);
spiral=0;  %start with formation control (set to 1 to skip this)

for t = 1:10000
  % Compute updated velocities
  V = zeros(2, N);
  for i = 1:N
    % ========================================================
    % START OF THE USER-CUSTOMIZABLE PORTION OF THE CODE
    %   IMPLEMENT THE CONTROLLER FUNCTION
    V(:, i) = controller(r, robots(i),spiral);
    % END OF THE USER-CUSTOMIZABLE PORTION OF THE CODE
    % ========================================================
  end
  %--------------------------------------
  X=zeros(3,N);
  spacing=zeros(N); %initialize spacing matrix
  threshold=.17;    %set formation threshold to activate spiraling
  for i=1:N
     X(:,i)=r.getState(i);
  end
  for j=1:N
      for k=j:N
            spacing(j,k)=norm(X(1:2,j)-X(1:2,k));
            spacing(k,j)=spacing(j,k);
      end
  end
  if abs(norm(X(1:2,:)))<threshold && abs(norm(spacing)-.7464)<.47*threshold
   spiral = 1;  %formation reached, now execute cyclic pursuit
  end
  %--------------------------------------
  
  % Send velocities to robots
  r.setVelocities(V);
  
  % Update dynamics and draw
  r.updateDynamics();
  r.draw;
  
  pause(0.05);
end