%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% main1.m
% Description: Main script to run spiraling robots
%
% Function Calls
% GetAlpha()   : Gets the alpha for the Archimedes Spiral Rotation Matrix.
%                Used in the rotation matrix calculation.
% showgraph()  : Simply plots the position of agents and the covered area
% rotation()   : Uses positions and gives back Rotation Matrix, Used to
%                update positions. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% To Do list
% -Fix the Coverage determination, Radius Shadow (Smaller blocks or what?)
% -Something wrong with Consensus Equation. Fix it
% -Gravitate to the normed vectors. How to get them to dynamically update
% -Add in drift/Noise
% -Adaptive controller to control noise/drift
% -Functionality? Update Z function?, Put in Show Graph?
% -Show jonathan the error over time (discrete time error or what!)
% -Potential Field Visualization idea, lyapunov set, gradient potential


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Block 1: Variable Initialization and Settings
clear;
RandStart = 1; %Determine is random start or not
N=6; %Number of agents to start (N robots)
dt=0.01; % numerical steplength
t=0; %Start time
Tf=50; %Final time
iter=1; %Iteration counter
DX=zeros(2,N);  %PreAllocation: Here is where we store the derivatives
X = zeros(2,N); %PreAllocation: Here is where we store the positions
Alpha = 0;
BlockSize = 0.05;
AxisLength=10; % Total Length of Axis
deltadisk = 0.3;
di = .03; %d for archimedes sprial
RadiusShadow = .08; %Length of Radius of area coverage ball [used in Showgraph()]
Consensus = 1; %Do you want consensus to center first?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Block 2: The Following Creates the meshgrid for the plot
hold on; %hold figure on for graphing
x = 0:BlockSize:AxisLength;
[X1,Y1] = meshgrid(0:BlockSize:AxisLength, 0:BlockSize:AxisLength); %Create Normal spaced vectors for axis
% PDF1 = normpdf(X1,AxisLength/2,AxisLength/4); %normal dis
% PDF2 = normpdf(Y1,AxisLength/2,AxisLength/4); %normal dis
% Z = PDF1.*PDF2; %z values
% Z = -1*(Z./max(max(Z)));
Z = zeros(size(X1));
for i=1:length(X1(1,:))
    for j=1:length(Y1(:,1))
        Z(j,i) = -1*gaussian(X1(1,i),Y1(j,1),AxisLength/2,AxisLength/2,sqrt(AxisLength/4),sqrt(AxisLength/4));
    end
end
%Z = Z./min(min(Z));

TrackingColors = Z;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Block 3: Initialize Positions and Create Agents
% Generate the initial conditions randomly
RadiusSpread=0.3; %Radius of how spread apart wanted 

if RandStart == 1
    X=AxisLength.*rand(2,N); %Create initial positions (x,y), *axislength
end

% Initial positions are not random
if RandStart == 0
   Theta = 2*pi/N;
   Center = [AxisLength/2 AxisLength/2]; %This is used to determine the intial positions
   for j = 1:N
       %Something about this isn't right - Check back later
       X(1,j) = Center(1)+RadiusSpread*sin(-Theta*j);
       X(2,j) = Center(2)+RadiusSpread*cos(-Theta*j);
   end
end
%----------------------------new code--------------------------------------
%using the non-random initial positions above as the target formation given
%that the actual starting positions are random
%goal is to have agents spiral and converge to this initial formation
Theta = 2*pi/N;
Center=[AxisLength/2 AxisLength/2];
start=zeros(2,N);           %initialize starting formation matrix
for j = 1:N
    start(1,j) = Center(1)+RadiusSpread*sin(-Theta*j);
    start(2,j) = Center(2)+RadiusSpread*cos(-Theta*j);
end

%--------------------------end of new code---------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Block 4: Consensus
while Consensus == 1;
        clf;
        DX=zeros(2,N+1);
        %Create virtual node at Center Position
        X(:,N+1) = [AxisLength/2; AxisLength/2]; %Center Position

%         for i = 1:N
%             for j = 1:N+1
%                 if (i~=j)
%                     
%                     if (j == N+1)
%                         DX(:,i) =DX(:,i)-((X(:,i)-X(:,j))*(norm(X(:,i)-X(:,j))-0.01)); %What is 0.03?
%                     else
%                         DX(:,i) =DX(:,i)-((X(:,i)-X(:,j))*(norm(X(:,i)-X(:,j))-0.01)); %What is 0.07?
%                     end
%                 end
%             end
%         end
%--------------------new code----------------------------------------------
reldistance=zeros(2,N+1);         %initialize relative distance matrix
relmag=zeros(1,N+1);              %magnitude of relative distance
mindistance=.2;                   %minimum distance needed to avoid collision
for i=1:N
    collision=0;                %initialize collision boolean to false
    for j=1:N+1
        if(i~=j)
            %this is just a safegaurd to ensure agents dont collide
            reldistance(:,j)=X(:,j)-X(:,i);
            relmag(1,j)=norm(reldistance(:,j));
            if(relmag(1,j)<mindistance)
                collision=1;   %indicates agents are too close
                DX(:,i)=DX(:,i)-reldistance(:,j)/relmag(1,j);
            end
        end
    end
    if(collision==0)
        Alpha = GetAlpha(N,di,i,X,Consensus);
        R=rotation(N,t,Alpha);
        DX(:,i)=DX(:,i) + R*3*((start(:,i)-X(:,i)));  %/(sqrt((start(1,i)-X(1,i))^2 + (start(2,i)-X(2,i))^2)); 
    end
end    
if abs(norm(X(:,1:N)-start))<BlockSize
   Consensus = 0; 
end
%-----------------end of new code------------------------------------------
        
        for k=1:N;
            X(:,k)=X(:,k)+(dt).*DX(:,k); %Why 0.1? Why not dt?
        end
        
        plot(X(1,:),X(2,:),'o')
        axis([0,AxisLength,0,AxisLength]);
        drawnow;
        if norm(X-repmat([AxisLength/2;AxisLength/2],1,N+1))<BlockSize
            Consensus = 0;
        end
        %Update Time here? - Depends what you want to restrict the total
        %time of
end
X(:,N+1) = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Block 5: Main While Loop to update positions
while (t<Tf)&&(Consensus==0);
        %% Determine change to TrackingColor value and update
    %_% Fix this based on RadiusShadow
    
    for k=1:N
        dx = (X1(1,:)+repmat(BlockSize/2,1,length(X1))) - repmat(X(1,k),1,length(X1));
        dy = (Y1(:,1)+repmat(BlockSize/2,length(Y1),1)) - repmat(X(2,k),length(Y1),1);
        [~,centerix] = min(abs(dx));
        [~,centeriy] = min(abs(dy));
         [indexX] = find(abs(dx)<=deltadisk);
         [indexY] = find(abs(dy)<=deltadisk);
         for i=1:length(indexX)
             for j=1:length(indexY)
                 if dx(indexX(i))^2+dy(indexY(j))^2 < deltadisk^2
                    TrackingColors(indexY(j),indexX(i)) = 0;
                 end
             end
         end
    end
    
%     di = di/(-1*Z(centeriy,centerix));
%     if di >= deltadisk
%         di = deltadisk;
%         break;
%         display('Max di reached');
%     end
    
    %% Update Rotation and Dx
    %Agents 1-(N-1): Rotation and Dx
%     if mod(iter,1000) == 0
%         di = 1+Z(indexY,indexX);
%         if di>= 0.5
%             di=0.5;
%         end
%     end
    
    for i=1:(N-1)
        Alpha = GetAlpha(N,di,i,X,Consensus);
        R=rotation(N,t,Alpha);
        DX(:,i)=R*((X(:,i+1)-X(:,i)));
    end;
    
    %Agent N: Rotation and Dx
    %Separated because wrapping around from agent N back to agent 1
    
        Alpha = GetAlpha(N,di,N,X,Consensus);
        R=rotation(N,t,Alpha);
        DX(:,N)=R*((X(:,1)-X(:,N))); 
    
    
    %% For All Agents, Update Position
    for i=1:N;
        X(:,i)=X(:,i)+dt.*DX(:,i);
    end;
    
    t=t+dt; %% Update time
    
    %% Plot the solution every X iterations. Skips some iterations for speed
    if (mod(iter,100)==0)
        show_graph(X,N,AxisLength,t,RadiusShadow); %plot graph
        pcolor(X1,Y1,TrackingColors);  %replot Z map with updates
    end;
    
   
    iter=iter+1; %counter of # of iterations of loop
    
end;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%END of File
