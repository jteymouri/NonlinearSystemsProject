classdef robotariumMatlabAPI < handle
  %robotariumMatlabAPI Summary of this class goes here
  %   Detailed explanation goes here
  
  properties (GetAccess = private, SetAccess = private)
    % Robot states
    N
    ids
    robotPoses
    targetPoses
    velocities
    
    % Robot parameters
    velocitiesMax
    motionNoise
    
    % Arena boundaries
    boundaries
    
    % Visualization parameters
    handles
    lineHandles
    deltaDiskHandles
    
    % Flags
    simulationMode
    initialized
    
    % Scenario parameters
    scenario
  end
  
  methods
    % -----------------------------------------------------------------
    % Constructor
    % -----------------------------------------------------------------
    function r = robotariumMatlabAPI(scenario)
      % Instantiate a robotariumMatlabAPI object
      %
      % SYNTAX:
      %   r = robotariumMatlabAPIObj(scenario)
      %
      % INPUTS:
      %   scenario - (1 x 1 string)
      %       A string describing one of the following scenarios
      %
      %       1. formation+assignment
      %       2. coverage
      %       3. leader-follower
      %       4. consensus
      %       5. no input - default scenario with randomly initialized
      %                     positions
      %
      % OUTPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       A Robotarium Matlab API object initialized according to
      %       chosen scenario
      %
      % NOTES:
      %----------------------------------------------------------------
      % Set arena parameters
      r.boundaries = [-0.6, 0.6, -0.35, 0.35];
      
      % Set static robot parameters
      r.velocitiesMax = [0.1; 2 * pi * 2];
      r.motionNoise   = [0.01; 2 * pi/180];
      
      % Initialize N robot
      r.N           = 6;
      r.ids         = 1:r.N;
      
      % Initialize velocities
      r.velocities = zeros(2, r.N);
      
      % Initialize target poses
      r.targetPoses = zeros(3, r.N);
      
      % Initialize handles structure
      r.handles           = ones(1, r.N) * -1;
      r.lineHandles       = ones(1, r.N) * -1;
      r.deltaDiskHandles  = ones(1, r.N^2) * -1;
      
      % Initialize simulation mode
      r.simulationMode = true;
      
      % Set initialization flag
      r.initialized = false;
      
      % Initialize poses according to scenario
      if(nargin > 0)
        % ----------------------------------------------------------------
        %     FORMATION CONTROL
        % ----------------------------------------------------------------
        if(strcmp(scenario, 'formation+assignment'))
          % Set formation matrix for hexagon-shaped formation
          %   (pairwise distances between agents)
          x = 0.1;
          h = sqrt(x^2 - (x/2)^2);
          y = 2 * h + x;
          z = sqrt(2*x^2);
          w = sqrt((h+x)^2 + (x/2)^2);
          F = [0, x, w, y, w, x; ...
               x, 0, x, w, z, x; ...
               w, x, 0, x, x, z; ...
               y, w, x, 0, x, w; ...
               w, z, x, x, 0, x; ...
               x, x, z, w, x, 0];
          r.scenario = struct('name', scenario, 'formation', F, 'maxDistance', 0.3);
          
          % Set agent poses
          r.robotPoses = getRandomPoseCircle(r, r.N, 0, 0, 0.15);
        % ----------------------------------------------------------------
        %     COVERAGE CONTROL
        % ----------------------------------------------------------------
        elseif(strcmp(scenario, 'coverage'))
          % TODO: add interesting density function here
          % Set density function
          r.scenario = struct('name', scenario, 'maxDistance', 0.3, ...
                              'densityFcn', @(x,y) 1);
          
          % Set agent poses
          r.robotPoses = r.getRandomPoseSquare(r.N, r.boundaries);
        % ----------------------------------------------------------------
        %     LEADER-FOLLOWER CONTROL
        % ----------------------------------------------------------------
        elseif(strcmp(scenario, 'leader-follower'))
          % Set leader waypoints and follower formations
          leaderWaypoints = [ 0.30, 0.30, -0.30, -0.30; ...
                             -0.15, 0.15,  0.15, -0.15; ...
                              0.00, pi/2,  pi  , 3*pi/2];
          
          % First formation is house-shaped
          F_1 = 0.1*[0 -0.5  0.5 -0.5  0.5; ...
                     0 -0.5 -0.5 -1.0 -1.0];
          F1 = zeros(5,5);
          for i = 1:5
            for j = 1:5
              F1(i,j) = norm(F_1(:,i) - F_1(:,j));
            end
          end
          
          % Second formation is diamond-shaped
          F_2 = 0.1*[0  0.0 -0.5  0.5  0.0; ...
                     0 -0.5 -1.0 -1.0 -1.5];
          F2 = zeros(5,5);
          for i = 1:5
            for j = 1:5
              F2(i,j) = norm(F_2(:,i) - F_2(:,j));
            end
          end
          formations = {{F1, F2}};
          r.scenario = struct('name', scenario, 'leaderWaypoints', leaderWaypoints, ...
                              'formations', formations, 'currentWaypoint', 1, ...
                              'currentFormation', 1, 'maxDistance', 0.3);
          
          % Set agent poses
          r.robotPoses  = r.getRandomPoseSquare(r.N, r.boundaries);
          
          % Set first target position
          r.setTargetPose(1, r.scenario.leaderWaypoints(:, r.scenario.currentWaypoint));
        % ----------------------------------------------------------------
        %     CONSENSUS WITH CONNECTIVITY MAINTENANCE
        % ----------------------------------------------------------------
        elseif(strcmp(scenario, 'consensus'))
          % Initialize topology (complete graph)
          topology = ones(r.N, r.N) - eye(r.N);
          
          % Set max distance between agents before they disconnect
          r.scenario = struct('name', scenario, 'maxDistance', 0.3, 'topology', topology);
          
          % Set agent poses
          r.robotPoses = zeros(3, r.N);
          r.robotPoses(:, 1) = r.getRandomPoseSquare(1, r.boundaries);
          
          % Agent poses are initialized such that the delta-disk graph with
          % delta = r.scenario.maxDistance is initially connected
          i = 2;
          while(i < r.N)
            parent = r.robotPoses(:, randi(i-1));
            pose   = r.getRandomPoseCircle(1, parent(1), parent(2), r.scenario.maxDistance);
            
            if(r.isWithinBoundaries(pose))
              r.robotPoses(:, i) = pose;
              i = i + 1;
            end
          end
        else
          errorMsg = ['Scenario ', scenario, ' does not exist.\n', ...
                      'Select one of the following scenarios: \n', ...
                      '1. formation+assignment \n', ...
                      '2. coverage \n', ...
                      '3. leader-follower \n', ...
                      '4. consensus'];
          
          error('error:selectionError', errorMsg);
        end
      else
        % Set max distance between agents before they disconnect
        r.scenario = struct('name', 'null');
          
        % Initialize randomly across the arena
        r.robotPoses = r.getRandomPoseSquare(r.N, r.boundaries);
      end
    end
    function initializeSubscribers(r)
      r.initialized = true;
    end
    
    % ------------------------------------------------
    % Individual robot GET functions
    % ------------------------------------------------
    function p = getState(r, id)
      % Get the current pose (x/y/theta) of the robot with the given id
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getState(id)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      % OUTPUTS:
      %   p - (3 x 1 number)
      %       x,y,theta-components of current pose.
      %
      % NOTES:
      %----------------------------------------------------------------
      if(id > 0 && id <= r.N)
        p = r.robotPoses(:,id);
      else
        error(['Trying to get the state of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function p = getTargetPose(r, id)
      % Get the current target pose (x/y/theta) of the robot with the
      % given id
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getTargetPose(id)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      % OUTPUTS:
      %   p - (3 x 1 number)
      %       x,y,theta-components of current target pose.
      %
      % NOTES:
      %----------------------------------------------------------------
      if(id > 0 && id <= r.N)
        p = r.targetPoses(:,id);
      else
        error(['Trying to get the target pose of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function v = getVelocity(r, id)
      % Get the current linear and angular velocities of the robot with
      % the given id
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getVelocity(id)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot used for wireless communication
      %
      % OUTPUTS:
      %   v - (2 x 1 number)
      %       Linear and angular velocities.
      %
      % NOTES:
      %----------------------------------------------------------------
      if(id > 0 && id <= r.N)
        v = r.velocities(:,id);
      else
        error(['Trying to get the velocity of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function [N, X] = getNeighbors(r, id, maxDistance)
      % Get the set of neighbors within distance 'maxDistance'
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getNeighbors(id, maxDistance)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      %   maxDistance - (1 x 1 number)
      %       All robots within distance 'maxDistance' to the current robot 
      %       are considered to be in the neighborhood
      %
      % OUTPUTS:
      %   N - (1 x M number)
      %       Vector of ids of all robots in the neighborhood
      %
      %   X - (3 x M number)
      %       Vector of poses of all robots in the neighborhood
      %
      % NOTES:
      %----------------------------------------------------------------
      N = [];
      X = [];
      
      % Check maxDistance according to scenario
      if(isfield(r.scenario, 'maxDistance'))
        if(maxDistance < r.scenario.maxDistance)
          maxDist = maxDistance;
        else
          maxDist = r.scenario.maxDistance;
        end
      else
        maxDist = maxDistance;
      end
      
      if(id > 0 && id <= r.N)
        for i = 1:r.N
          d = norm(r.robotPoses(1:2,i)-r.robotPoses(1:2,id));
          if (d < maxDist && (i ~=id))
            N = [N i];
            X = [X, r.robotPoses(:, i)];
          end
        end
      else
        error(['Trying to get the neighbors of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function [N, X] = getNeighborsFromTopology(r , id)
      % Get the set of neighbors according to the current graph topology
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getNeighborsFromTopology(id)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      % OUTPUTS:
      %   N - (1 x M number)
      %       Vector of ids of all robots in the neighborhood specified
      %       by the topology
      % 
      %   X - (3 x M number)
      %       Vector of poses of all robots in the neighborhood specified
      %       by the topology
      %
      % NOTES:
      %----------------------------------------------------------------
      if(isfield(r.scenario, 'topology'))
        N = [];
        X = [];
        if(id > 0 && id <= r.N)
          for i = 1:r.N
            if (r.scenario.topology(id,i) == 1)
              N = [N i];
              X = [X r.robotPoses(:, i)];
            end
          end
        else
          error(['Trying to get the neighbors of robot ', num2str(id), '. Only N = 6 robots available.']);
        end
      else
        error(['No topology specified for scenario ', r.scenario.name]);
      end
    end
    
    % ------------------------------------------------
    % Individual robot SET functions
    % ------------------------------------------------
    function setVelocity(r, id, v)
      % Set linear and rotational velocities of a robot
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setVelocity(id, v)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      %   v - (2 x 1 number)
      %       Linear and angular velocity.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      if(id > 0 && id <= r.N)
        if(size(v, 1) == 2 && size(v, 2) == 1)
          r.velocities(:,id) = r.saturateVelocities(v);
        else
          error('Velocities vector needs to be of size 2x1');
        end
      else
        error(['Trying to set the velocities of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function setTargetPose(r, id, pose)
      % Set the target pose for the robot with the given id
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setTargetPose(id, pose)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      %   pose - (3 x 1 number)
      %       x,y,theta components of target pose.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      if(id > 0 && id <= r.N)
        if(size(pose, 1) == 3 && size(pose, 2) == 1)
          if(r.isWithinBoundaries(pose))
            r.targetPoses(:,id) = pose;
          else
            error(['Target pose vector needs to be within boundaries of ', num2str(r.boundaries)]);
          end
        else
          error('Target pose vector needs to be of size 3x1');
        end
      else
        error(['Trying to set the target position of robot ', num2str(id), '. Only N = 6 robots available.']);
      end
    end
    function setSimulationMode(r, simulationMode)
      % Switch between simulation and hardware mode
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setSimulationMode(simulationMode)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   simulationMode - (1 x 1 number)
      %       Boolean determining simulation mode (1) or hardware mode (0)
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      r.simulationMode = simulationMode;
    end
    
    % ------------------------------------------------
    % Group robot GET functions
    % ------------------------------------------------
    function N = getAvailableRobots(r)
      % Get a list of all available robots, i.e. robots that are
      % registered with the backend and have been matched to markers
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getAvailableRobots()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS:
      %   N - (1 x N number)
      %       Vector of ids of active robots
      %
      % NOTES:
      %----------------------------------------------------------------
      N = r.ids;
    end
    function [I, V] = getVelocities(r)
      % Get the velocities of all available robots, i.e. robots that
      % are registered with the backend and have been matched to
      % markers
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getVelocities()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS:
      %   I - (1 x N number)
      %       Vector of ids of active robots
      %
      %   V - (2 x N number)
      %       Vector of velocities of active robots
      %
      % NOTES:
      %----------------------------------------------------------------
      I = r.ids;
      V = r.velocities;
    end
    function [I, P] = getTargetPoses(r)
      % Get the target poses of all available robots, i.e. robots
      % that are registered with the backend and have been matched to
      % markers
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getTargetPoses()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS:
      %   I - (1 x N number) [zeros(1,N)]
      %       Vector of ids of active robots
      %
      %   P - (3 x N number) [zeros(3,N)]
      %       Vector of x/y/theta target poses of active robots
      %
      % NOTES:
      %----------------------------------------------------------------
      I = r.ids;
      P = r.targetPoses;
    end
    
        % ------------------------------------------------
    % Group robot SET functions
    % ------------------------------------------------
    function setTargetPoses(r, poses)
      % Set x/y/theta target poses of all available robots
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setTargetPoses(poses)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   poses - (3 x N number)
      %       x/y/theta target poses of robots.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      if(size(poses, 1) == 3 && size(poses, 2) == r.N)
        for i = 1:size(poses, 2)
          if(~r.isWithinBoundaries(poses(:, i)))
            error(['All target pose vectors need to be within boundaries of ', num2str(r.boundaries)]);
          end
        end
        
        r.targetPoses = poses;
      else
        error('Target poses vector needs to be of size 3 x N with N = 6');
      end
    end
    function setVelocities(r, velocities)
      % Set linear and rotational velocities of all available robots.
      % Velocities are saturated according to actuator limits on the
      % robots.
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setVelocities(velocities)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   velocities - (2 x N number)
      %       Linear and rotational velocities
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      if(size(velocities, 1) == 2 && size(velocities, 2) == r.N)
        V = zeros(2, r.N);
        for i = 1:r.N
          V(:,i) = r.saturateVelocities(velocities(:,i));
        end
        r.velocities = V;
      else
        error('Velocities vector needs to be of size 2xN with N = 6');
      end
    end
    function stopAllRobots(r)
      % Stop all robots, i.e. set their velocities to 0
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.stopAllRobots()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      r.velocities = zeros(2, r.N);
    end
    
    % ------------------------------------------------
    % Scenario-related GET functions
    % ------------------------------------------------
    function F = getFormation(r)
      % Get the current formation specified as an N x N matrix of pairwise
      % interagent distances
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getFormation()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS:
      %   F - (N x N number)
      %       The N x N matrix of pairwise interagent distances specifying
      %       the formation - for the scenarios for which a formation is
      %       specified
      %
      % NOTES:
      %----------------------------------------------------------------
      if(strcmp(r.scenario.name, 'leader-follower'))
        F = r.scenario.formations{r.scenario.currentFormation};
      elseif(strcmp(r.scenario.name, 'formation+assignment'))
        F = r.scenario.formation;
      else
        error(['No formation given for scenario ', r.scenario.name]);
      end
    end
    function d = getDensity(r, x, y)
      % Compute the value of the current density function at point x/y
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setTargetPoses(poses)
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   x - (1 x 1 number)
      %       X-coordinate of desired point
      %
      %   y - (1 x 1 number)
      %       Y-coordinate of desired point
      %
      % OUTPUTS:
      %   d - (1 x 1 number)
      %       Density computed according to current density function at
      %       point x/y
      %
      % NOTES:
      %----------------------------------------------------------------
      if (strcmp(r.scenario.name, 'coverage'))
        d=r.scenario.densityFcn(x,y);
      else
        error('No density function for current scenario');
      end
    end
    
    % ------------------------------------------------
    % Scenario-related SET functions
    % ------------------------------------------------
    function setTopology(r, adjMatrix)
      % Sets the underlying graph topology
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setTopology(adjmatrix)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   adjmatrix - (N x N number)
      %       An N x N adjacency matrix
      %
      % OUTPUTS: none
      %
      % NOTES:
      %---------------------------------------------------------------
      if( size(adjMatrix,1) == r.N && size(adjMatrix, 2) == r.N)
        if(isfield(r.scenario, 'topology'))
          r.scenario.topology = adjMatrix;
        else
          error(['No topology specifiable for scenario', r.scenario.name]);
        end
      else
        error(['Adjacency matrix needs to be of size ', num2str(r.N),' x ', num2str(r.N), '.']);
      end
    end
    function setDensity(r, n)
      % Sets a predefined density function for the coverage control scenario
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setDensityFunction(densityFcnHandle)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   n - (1 x 1 number)
      %       Index of the desired density function. Needs to be in the
      %       range of 1 to 4.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %---------------------------------------------------------------
      if (strcmp(r.scenario.name, 'coverage'))
        e = exp(1);
        if (n == 1)
          r.setDensityFunction(@(x,y) 1);
        elseif (n == 2)
          r.setDensityFunction(@(x,y) e^(-0.5*(x^2 + y^2)));
        elseif(n==3)
          r.setDensityFunction(@(x,y) e^(-0.5*((x-0.4)^2 + (y)^2)) ...
            + e^(-0.5*((x+0.4)^2 + (y)^2)));
        elseif(n==4)
          r.setDensityFunction(@(x,y) e^(-0.5*((x-0.4)^2 + (y-0.1)^2)) ...
            + e^(-0.5*((x+0.4)^2 + (y+0.1)^2)) ...
            + e^(-0.5*((x+0.4)^2 + (y-0.1)^2))...
            + e^(-0.5*((x+0.4)^2 + (y-0.1)^2)));
        else
          error('Call setDensity with any value between 1 and 4');
        end
      else
        error('No density function to set in this scenario');
      end
    end
    function setDensityFunction(r, densityFcnHandle)
      % Sets a density function for the coverage control scenario
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setDensityFunction(densityFcnHandle)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   densityFcnHandle - (1 x 1 function handle)
      %       A function handle to a density function with input parameters
      %       x and y, i.e. the density function has to be of the form 
      %       phi = densityFcnHandle(x, y)
      %
      % OUTPUTS: none
      %
      % NOTES:
      %---------------------------------------------------------------
      if(strcmp(r.scenario.name, 'coverage'))
        if(isa(densityFcnHandle,'function_handle'))
          if(nargin(densityFcnHandle) == 2)
            r.scenario.densityFcn = densityFcnHandle;
          else
            error('Provide a function handle with two input parameters!');
          end
        else
          error('Provided input is not a function handle');
        end
      else
        error('A function handle can only be provided for the coverage scneario');
      end
    end
    function setDeltaDiskDistance(r, maxDist)
      % Sets the maximum distance up to which two agents are considered
      % adjacent and connected
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.setDeltaDiskDistance(maxDist)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   maxDist - (1 x 1 number)
      %       Maximum distance for the delta disk graph
      %
      % OUTPUTS: none
      %
      % NOTES:
      %---------------------------------------------------------------
      if( isscalar(maxDist))
        if(isfield(r.scenario, 'maxDistance'))
          r.scenario.maxDistance = maxDist;
        else
          error(['No maximum distance specifiable for scenario', r.scenario.name]);
        end
      else
        error(['Maximum distance needs to be scalar.']);
      end
    end
    
    % ------------------------------------------------
    % UPDATE functions
    % ------------------------------------------------
    function updateDynamics(r)
      % Update the dynamics of all robots. In case there is a leader agent,
      % this function updates the waypoint following logic of the leader.
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.updateDynamics()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      dt = 0.05;
      
      if(strcmp(r.scenario.name, 'leader-follower'))
        % Update leader dynamics (waypoint following) through position control
        % Overwrite any user-computed velocities for agent 1
        k  = 50.0;
        l  = 0.05;
        vx = k * (r.targetPoses(1, 1) - r.robotPoses(1, 1));
        vy = k * (r.targetPoses(2, 1) - r.robotPoses(2, 1));
        
        v  = ( cos(-r.robotPoses(3, 1)) * vx - sin(-r.robotPoses(3, 1)) * vy );
        w  = ( sin(-r.robotPoses(3, 1)) * vx + cos(-r.robotPoses(3, 1)) * vy ) / l;
        
        r.setVelocity(1, 0.8 .* r.saturateVelocities([v; w]));
        
        % Waypoint switching for leader agent
        d = norm(r.robotPoses(1:2, 1) - r.targetPoses(1:2, 1));
        if(d < 0.08)
          r.scenario.currentWaypoint = r.scenario.currentWaypoint + 1;
          if(r.scenario.currentWaypoint > 4)
            % Reset waypoint
            r.scenario.currentWaypoint = 1;
            
            % Toggle target formation
            r.scenario.currentFormation = r.scenario.currentFormation + 1;
            if(r.scenario.currentFormation > length(r.scenario.formations))
              r.scenario.currentFormation = 1;
            end
          end
          
          r.setTargetPose(1, r.scenario.leaderWaypoints(:, r.scenario.currentWaypoint));
          
          disp(['Current formation: ', num2str(r.scenario.currentFormation)]);
          disp(['Current waypoint: ', num2str(r.scenario.leaderWaypoints(:, r.scenario.currentWaypoint)')]);
        end
      end
      
      % Update dynamics of all agents
      for i = 1:r.N
        x   = r.robotPoses(1, i);
        y   = r.robotPoses(2, i);
        th  = r.robotPoses(3, i);
        v   = r.velocities(1, i);
        w   = r.velocities(2, i);
        
        % Update using unicycle dynamics
        r.robotPoses(1, i) = x  + (v * cos(th) + rand() * r.motionNoise(1)) * dt;
        r.robotPoses(2, i) = y  + (v * sin(th) + rand() * r.motionNoise(1)) * dt;
        r.robotPoses(3, i) = th + (w           + rand() * r.motionNoise(2)) * dt;
      end
    end
    
    % ------------------------------------------------
    % DRAW functions
    % ------------------------------------------------
    function draw(r)
      % Render all robots as circular markers with direction arrow. If the
      % scenario reqires a delta-disk graph, visualize the connectivity
      % graph as well.
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.draw()
      %
      % INPUTS:
      %   r - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      % OUTPUTS: none
      %
      % NOTES:
      %----------------------------------------------------------------
      % Set plotting parameters
      arrowLength = 0.03;
      maxHeadSize = 5;
      lineWidth   = 2;
      
      for i = 1:r.N
        if(ishandle(r.handles(i)))
          % Update robot handle
          set(r.handles(i), 'XData', r.robotPoses(1, i));
          set(r.handles(i), 'YData', r.robotPoses(2, i));
        else
          % Create new handle
          hold on;
          r.handles(i) = plot(r.robotPoses(1, i), r.robotPoses(2, i), 'ro', 'MarkerSize', 10);
        end

        if(ishandle(r.lineHandles(i)))
          x  = r.robotPoses(1, i);
          y  = r.robotPoses(2, i);
          th = r.robotPoses(3, i);
          
          set(r.lineHandles(i), 'XData', x);
          set(r.lineHandles(i), 'YData', y);
          set(r.lineHandles(i), 'UData', cos(th) * arrowLength);
          set(r.lineHandles(i), 'VData', sin(th) * arrowLength);
        else
          x  = r.robotPoses(1, i);
          y  = r.robotPoses(2, i);
          th = r.robotPoses(3, i);
          
          r.lineHandles(i) = quiver(x, y, cos(th) * arrowLength, sin(th) * arrowLength, ...
                                    0, 'MaxHeadSize', maxHeadSize, 'LineWidth', lineWidth);
        end
      end
      
      % Draw delta disk graph if specified by scenario
      if(isfield(r.scenario, 'topology'))
        L = r.scenario.topology;
        edge = 1;
        for i = 1:r.N
          for j = i:r.N
            if(L(i, j) == 1)
              r.updateEdge(edge, i, j);
              edge = edge + 1;
            end
          end
        end
      elseif(isfield(r.scenario, 'maxDistance'))
        edge = 1;
        for i = 1:r.N
          for j = i:r.N
            d = norm(r.robotPoses(1:2, i) - r.robotPoses(1:2, j));
            
            if(d < r.scenario.maxDistance)
              r.updateEdge(edge, i, j);
              edge = edge + 1;
            end
          end
        end
        
        % Delete remaining unused edges
        for i = edge-1:length(r.deltaDiskHandles)
          if(ishandle(r.deltaDiskHandles(i)))
            delete(r.deltaDiskHandles(i));
          end
        end
      end
      
      % Set axis parameters
      axis(r.boundaries);
      set(gca, 'PlotBoxAspectRatio', [1,1,1], 'DataAspectRatio', [1,1,1]);
    end
    function updateEdge(r, handleIndex, i, j)
      if(ishandle(r.deltaDiskHandles(handleIndex)))
        % Update edge handle
        set(r.deltaDiskHandles(handleIndex), 'XData', [r.robotPoses(1,i), r.robotPoses(1,j)]);
        set(r.deltaDiskHandles(handleIndex), 'YData', [r.robotPoses(2,i), r.robotPoses(2,j)]);
      else
        % Instantiate new edge
        r.deltaDiskHandles(handleIndex) = line([r.robotPoses(1,i), r.robotPoses(1,j)], ...
                                               [r.robotPoses(2,i), r.robotPoses(2,j)]);
      end
    end
    
    % ------------------------------------------------
    % UTILITIY functions
    % ------------------------------------------------
    function v = saturateVelocities(r, vRaw)
      % Saturates linear and angular velocities of each robots according to
      % actuator limits specified in r.velocitiesMax
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.saturateVelocities(vRaw)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   vRaw - (2 x 1 number)
      %       The raw input velocities as computed by the controller
      %
      % OUTPUTS:
      %   v - (2 x 1 number)
      %       The saturated velocities
      %
      % NOTES:
      %---------------------------------------------------------------
      scaleV = 1;
      scaleW = 1;
      
      if(abs(vRaw(1)) > abs(r.velocitiesMax(1)))
        scaleV = abs(r.velocitiesMax(1) / vRaw(1));
      end
      
      v = vRaw * scaleV;
      
      if(abs(v(2)) > abs(r.velocitiesMax(2)))
        scaleW = abs(r.velocitiesMax(2) / vRaw(2));
      end
      
      v = v * scaleW;
    end
    function [T, B, L, R] = getDistanceToBoundaries(r , id)
      % Computes the distance to each of the four boundaries of the arena. 
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getDistanceToBoundaries(id)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   id - (1 x 1 number)
      %       Id of target robot
      %
      % OUTPUTS:
      %   T - (1 x 1 number)
      %       The distance to the top boundary of the arena
      %   B - (1 x 1 number)
      %       The distance to the bottom boundary of the arena
      %   L - (1 x 1 number)
      %       The distance to the left boundary of the arena
      %   R - (1 x 1 number)
      %       The distance to the right boundary of the arena
      %
      % NOTES: - r.boundaries contains [xMin, xMax, yMin, yMax]
      %---------------------------------------------------------------
      idState = r.robotPoses(:,id);
      L = abs(idState(1,:) - r.boundaries(1));
      R = abs(idState(1,:) - r.boundaries(2));
      T = abs(idState(2,:) - r.boundaries(4));
      B = abs(idState(2,:) - r.boundaries(3));
    end
    function b = isWithinBoundaries(r, pose)
      % Determines whether a pose is within the boundaries of the arena.
      % Mostly used to instantiate initial random positions of agents for
      % the simulation
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.isWithinBoundaries(pose)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   pose - (3 x 1 number)
      %       Pose to be checked
      %
      % OUTPUTS:
      %   b - (1 x 1 number)
      %       Boolean value determining whether pose is within (1)
      %       boundaries or not (0)
      %
      % NOTES: - r.boundaries contains [xMin, xMax, yMin, yMax]
      %---------------------------------------------------------------
      if(pose(1) >= r.boundaries(1) && pose(1) <= r.boundaries(2) &&...
          pose(2) >= r.boundaries(3) && pose(2) <= r.boundaries(4))
        b = true;
      else
        b = false;
      end
    end
    function P = getRandomPoseSquare(r, N, limits)
      % Generate N random poses within a square defined by limits, which
      % is a vector [xMin, xMax, yMin, yMax]
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getRandomPoseSquare(N, limits)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   N - (1 x 1 number)
      %       Number of poses to be generated within the given limits
      %
      %   limits - (4 x 1 or 1 x 4 number)
      %       A vector containing [xMin, xMax, yMin, yMax] limits in which
      %       the poses need to be generated
      %
      % OUTPUTS:
      %   P - (3 x N number)
      %       Generated poses
      %
      % NOTES:
      %---------------------------------------------------------------
      dx = limits(2) - limits(1);
      dy = limits(4)- limits(3);
      
      P = zeros(3, N);
      P(1,:) = limits(1) + dx * rand(1,N);
      P(2,:) = limits(3) + dy * rand(1,N);
      P(3,:) = rand(1,N) * 2 * pi;
    end
    function P = getRandomPoseCircle(r, N, x, y, radius)
      % Generate N random poses within a circle of radius r centered at x/r
      %
      % SYNTAX:
      %   robotariumMatlabAPIObj.getRandomPoseCircle(N, x, y, radius)
      %
      % INPUTS:
      %   robotariumMatlabAPIObj - (1 x 1 robotariumMatlabAPIObj)
      %       An instance of the "robotariumMatlabAPIObj" class.
      %
      %   N - (1 x 1 number)
      %       Number of poses to be generated within the circle
      %
      %   x - (1 x 1 number)
      %       X-coordinate of center of circle
      %
      %   y - (1 x 1 number)
      %       Y-coordinate of center of circle
      %   
      %   radius - (1 x 1 number)
      %       Radius of the circle
      %
      % OUTPUTS:
      %   P - (3 x N number)
      %       Generated poses
      %
      % NOTES:
      %---------------------------------------------------------------
      angles  = rand(1, N) * 2 * pi;
      radii   = rand(1, N) * radius;
      
      P = zeros(3, N);
      for i = 1:N
        P(1,i) = x + radii(i) * cos(angles(i));
        P(2,i) = y + radii(i) * sin(angles(i));
        P(3,i) = rand() * 2 * pi;
      end
    end
  end
end