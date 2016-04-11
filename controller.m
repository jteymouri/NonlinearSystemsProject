function [ v ] = controller( r, id, spiral )

  %initialize variables
  L=0.05;                   %length use for M matrix below
  v=[0;0];                  %velocity vector for xi
  collisionFlag=false;      %initialize collision flag
  xDot = [0; 0]; xmove=[0;0];
  radius=.1;                %radius of circle for cyclic pursuit
  N=6;                      %number of agents
  gain=15;                  %velocity gain
  angle=pi/N;               %angle spacing between agents
  
  %collision avoidance thresholds
  Threshold = .06;
  Threshold1 = .05;
  close = .07;
  
  %initialize formation matrix of pairwise distances
  d=zeros(N,1);
  for i=1:N
     d(i)=sin((i-1)*angle);
  end
  form=(2*radius).*[d(1) d(2) d(3) d(4) d(5) d(6);...
        d(2) d(1) d(2) d(3) d(4) d(5);...
        d(3) d(2) d(1) d(2) d(3) d(4);...
        d(4) d(3) d(2) d(1) d(2) d(3);...
        d(5) d(4) d(3) d(2) d(1) d(2);...
        d(6) d(5) d(4) d(3) d(2) d(1)];
  
  xi = r.getState(id);      %get current node state
  M = [cos(xi(3)), sin(xi(3));-sin(xi(3))/L, cos(xi(3))/L];
    
%MAIN LOOP-
  if spiral==0
  %get in circular formation from initial random positions
        for i=1:N
            Xi=r.getState(i);
            if i~=id && norm(Xi(1:2)-xi(1:2)) < close
            %if neighbors start to get close, increase gain on their
            %formation control (proactive collision avoidance)
                scale=3;
            elseif abs(id-i)==1 || abs(id-i)==5
                scale=2; %pay more attention to immediate neighbors
            else
                scale=1;
            end
            if i~=id && norm(Xi(1:2)-xi(1:2)) < Threshold
            %collision avoidance routine  
                display(['too close: ', num2str(id)]);
                %xmove=xmove + 2*scale.*(Xi(1:2)-xi(1:2))*(norm(Xi(1:2)-xi(1:2))-form(id,i));
                xDot=xDot + 2*scale.*(Xi(1:2)-xi(1:2))*(norm(Xi(1:2)-xi(1:2))-form(id,i));
                if norm(Xi(1:2)-xi(1:2)) < Threshold1
                    collisionFlag=true;
                    display(['collision: ',num2str(id)]);
                    xDot=2*scale.*(xi(1:2)-Xi(1:2)); %last ditch collision avoidance
                end
            else
            %no collisions, execute formation
                xDot=xDot+scale.*(Xi(1:2)-xi(1:2))*(norm(Xi(1:2)-xi(1:2))-form(id,i));
            end
        end
        if collisionFlag~=true 
            xDot=xDot-.01.*xi(1:2)/norm(xi(1:2)); %center agents at origin
        end
        v=gain.*M*xDot;
  elseif spiral==1
  %run a circle
      if id<N
            Xi=r.getState(id+1);
            xDot=xDot + (Xi(1:2)-xi(1:2));  %cyclic pursuit
      else
            Xi=r.getState(1);
            xDot=xDot + (Xi(1:2)-xi(1:2)); 
      end
      xDot=xDot - (xi(1:2))*(norm(xi(1:2)));  %stay centered  
      v=gain.*M*xDot;
  
%STILL TO DO: MAKE SPIRAL CODE  
%   elseif spiral==2
%       %start spiral code here
%       for i=1:N
%          X(:,i)=r.getState(i);
%       end
%       Alpha = GetAlpha(N,.03,id,X);
%       M1 = [cos(Alpha), sin(Alpha);-sin(Alpha)/L, cos(Alpha)/L];
%       v=M1*...
  end
  
end