close all
clear all
clc

%Python dependencies for navigation fields (has to pip install shapely, tripy packages)
py.importlib.import_module('polygeom_lib');
py.importlib.import_module('reactive_planner_lib');


global WantMovie frameNum frameRate Nframes timeBtwFrames flowTime jumpTime maxStep
global independentNav softSpring rescalingFactors ang agentNum orderNum offsetNum stateDim lambda gamma adjMat R varrho springGain alpha mu omega xStar p m M varrhoStar
%% Recorded Output parameters
WantMovie=1;
frameRate = 30;         % Frame rate for playing the video
Nframes = 600;          % Total number of frames in the video
flowTime = 30;  % Define simulation flow time
jumpTime = 1;   % Define the total number of jumps
maxStep = 1/(2*frameRate);  % Maximum step size for numerical solver
timeBtwFrames = flowTime/Nframes;

%% Simulation parameters and Initial State

% basic PnP properties
independentNav=0; % Does everyone navigate independently? (1 only needed for a single demo, otherwise make sure this is set to 0)
softSpring=0;   % Does the tension vanish in the safe zone? 
rescalingFactors=1; % Are the PnP rescaling factors present?

% workspace geometry
p=10;           % Obstacle shape parameter (p=1 gives a circle)
lambda = 4;     % Selected greater than zero, impacts target location

% MAS geometry
agentNum = 14;  % Number of agents in MAS: NO LESS THAN 3!! (indexing issues)
orderNum = 14;  % Number of position in the initial cycle
offsetNum = 8;  % Agent #1 initial position along the cycle (the rest of the agents are posted following it in counter-clockwise fashion)
stateDim = 2;   % Dimension of state-space (DO NOT CHANGE)
alpha = 1;      % Tension growth exponent equals 1+alpha
gamma = .1;      % Leader Control gain

ang = 2*pi/orderNum;
R = 1.5*ang;    % Communication radius
springGain = .1;        % PnP Control gain


% Initialize agent positions and target location

x0 = zeros(agentNum,stateDim);
for k = 0 : agentNum-1
   %x0(k+1,:) = Trans(1.25*[cos((k+offsetNum+10^(-20))*ang),sin((k+offsetNum+10^(-20))*ang)]); 
   x0(k+1,:) = 1.25*[cos((k+offsetNum+10^(-20))*ang),sin((k+offsetNum+10^(-20))*ang)]; 
end

xStar = (1+lambda)*[1,0]';

% Reshape the initial condition for the hybrid systems solver
x0Alt = reshape(x0',[1 agentNum*stateDim]);


%% Define the Adjacency matrix

% Generate an undirected adjacency matrix for the communication graph and compute the maximum distance along an edge
distMat = zeros(agentNum,agentNum);
R_max = 0;
for i = 1:agentNum
    for j = 1:agentNum
        if (norm(x0(i,:)-x0(j,:)) <= R) && (i ~= j)
            distMat(i,j) = 1;
        end
        
        if (norm(x0(i,:)-x0(j,:)) > R_max) && (distMat(i,j)==1)
           R_max =  norm(x0(i,:)-x0(j,:));
        end
    end
end

circMat = zeros(agentNum,agentNum);
for i = 1:agentNum
    for j = 1:agentNum
        if abs(i-j) == 1 || abs(i-j) == agentNum-1
            circMat(i,j) = 1;
        else
            circMat(i,j) = 0;
        end
    end
end

intMat = circMat;
broken = 6;      % Select which edge to remove from the circle
if broken <= agentNum-1 && broken >= 1
   intMat(broken,broken+1) = 0;
   intMat(broken+1,broken) = 0;
else
    intMat(1,agentNum) = 0;
    intMat(agentNum,1) = 0;
end

adjMat = intMat; % <-------------------------------------- Pick your adjacency matrix

EE = sum(adjMat*ones(agentNum,1))/2; % Number of edges

%Compute a value for the buffer radius
InitMaxDistBelowR=0;
for i = 1:agentNum
    for j = 1:agentNum
        if distMat(i,j)==1 && adjMat(i,j)==1
            InitMaxDistBelowR=max(InitMaxDistBelowR,norm(x0(i,:)-x0(j,:)));
        end
    end
end
varrho = 0.5*(InitMaxDistBelowR+R); % computed buffer radius                             

m = R / varrho;
if softSpring==1
    mu=0;
    omega=springGain;
    varrhoStar=varrho+(R-varrho)^(1+alpha)*EE^(-1/(2+alpha));
else
    mu=springGain;
    omega=((2+alpha)*mu/(2*R^(1+alpha)))*m^(1+alpha)*(EE-m^2)/((m-1)^(2+alpha));
    varrhoStar=varrho;
end


%% Simulation

TSPAN = [0 flowTime];
JSPAN = [0 jumpTime];

% Rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 2;

options = odeset('RelTol',1e-6,'MaxStep',maxStep); 

% Simulate
[t,j,x] = HyEQsolver(@FlowMap,@JumpMap,@FlowSet,@JumpSet,x0Alt,TSPAN,JSPAN,rule,options,'ode45');
%[t,x] = ode45(@FlowMap,TSPAN,x0Alt');
%[t,x] = ode23(@FlowMap,TSPAN,x0Alt');

%% Weed out the chaff
x=permute(reshape(x,[length(t),stateDim,agentNum]),[3,2,1]);

% Remove unnecessary data by sampling time at near-uniform intervals
redundantFrames=[];
frameNum=1;
for step = 2:length(t)
    if floor(t(step-1)/timeBtwFrames)<floor(t(step)/timeBtwFrames) && frameNum<Nframes
        frameNum=frameNum+1;
    else
        redundantFrames=[redundantFrames,step];
    end
end
t(redundantFrames)=[];
j(redundantFrames)=[];
x(:,:,redundantFrames)=[];

%% Plots

figHeight=600;
topViewWidth=100*(5+lambda);
edgeLengthsWidth=800;

combined_fig=figure('Position',[0,0,topViewWidth+edgeLengthsWidth,figHeight]);
%pnp_ax=subplot(1,2,1);
pnp_ax=subplot('Position',[0.05,0.1,0.5,0.8]);
%el_ax=subplot(1,2,2);
el_ax=subplot('Position',[0.6,0.1,0.35,0.8]);

% Size the topView plot
daspect(pnp_ax,[1,1,1]);
xlim(pnp_ax,[-2,xStar(1)+1]);
ylim(pnp_ax,[-2,2]);

% Prepare edgeLengths figure
edgeLength = zeros(length(t),agentNum-1);
for i = 1:agentNum
    for step = 1:length(t)
        if i==agentNum
            edgeLength(step,i) = norm(x(i,:,step)-x(1,:,step))/R;
        else
            edgeLength(step,i) = norm(x(i,:,step)-x(i+1,:,step))/R;
        end
    end
end

hold(el_ax,'on')
for i = 1:agentNum
  plot(el_ax,t,edgeLength(:,i),'LineWidth',0.5*(1+agentNum-i),'DisplayName','$i=$'+string(i));
end
plot(el_ax,varrho/R,'LineWidth',0.125,'Color','b','DisplayName','$d=\varrho/R\qquad$');
eL_plot=plot(el_ax,t(1)*ones(1,agentNum),edgeLength(1,:),'LineWidth',0.5,'Color','b','Marker','o','DisplayName','$t=$'+string(round(t(1),3)));
legend(el_ax,'Interpreter','latex','FontSize',16,'Location','northeastoutside');
if independentNav==1
    ylim(el_ax,[0,5]);
else
    ylim(el_ax,[0,1.2]);
end
xlabel(el_ax,'$t$=Time elapsed','Interpreter','latex','FontSize',16);
ylabel(el_ax,'$d=\Vert x_i - x_{i+1} \Vert/R$','Interpreter','latex','FontSize',16);
hold(el_ax,'off')

% Prepare evolving graphs video
    %pnp_ax.NextPlot='replaceChildren';

% video parameters
pnp_movie(length(t))=struct('cdata',[],'colormap',[]);

% calculate frame (viewport) to be captured
rect=combined_fig.Position;

% PREP OBSTACLE BOUNDARY
radius = 1;
theta = 0:0.01:2*pi+1;
OBS=zeros(length(theta), 2);
for ii=1:length(theta)
    OBS(ii,:)=Trans([radius*cos(theta(ii));radius*sin(theta(ii))]);
end

hold(pnp_ax,'on')
% Plot the target
targetPlot=plot(pnp_ax,xStar(1),xStar(2),'*','DisplayName',"$\quad$ target $x^{\ast}$");

% Plot obstacle boundary
obsPlot=plot(pnp_ax,OBS(:,1),OBS(:,2),'b','DisplayName',"$\quad$ obstacle boundary");

% Plot the graph
masGraph=commGraph(x(:,:,1));
masGraphPlot=plot(pnp_ax,masGraph,'XData',x(:,1,1),'YData',x(:,2,1),'DisplayName',"$\quad$ communication graph $\mathcal{G}$");

% Prepare title,legend,etc...
if independentNav==1
    title(pnp_ax,"Agents navigate independently of each other");
    basic_info="$t=$";
else
    title(pnp_ax,"PnP controller in action");
    basic_info="$\alpha=$"+string(alpha)+", $\mu=$"+string(mu)+", $\omega=$"+string(omega)+", $\varrho^\ast=$"+string(varrhoStar)+", $\gamma=$"+string(gamma)+", $t=$";
end
subtitle(pnp_ax,basic_info+string(round(t(1),3)),'Interpreter','latex');
legend(pnp_ax,'Interpreter','latex','FontSize',16);

hold(pnp_ax,'off')

%
%%%% Capture the video
%

%Capture the first frame and save it as a JPG
combined_fig.Visible='off';
drawnow
pnp_movie(1)=getframe(combined_fig,rect);
imwrite(frame2im(pnp_movie(1)),strcat("pnp_movie_poster-",string(1),".jpg"),'jpeg');
%Capture the rest of the movie
for frame = 2:length(t)
    if mod(frame,10)==0
        frame
    end
    %tG=commGraph(x(:,:,frame);
    subtitle(pnp_ax,basic_info+string(round(t(frame),3)),'Interpreter','latex');
    masGraphPlot.XData=x(:,1,frame);
    masGraphPlot.YData=x(:,2,frame);
    eL_plot.XData=t(frame)*ones(1,agentNum);
    eL_plot.YData=edgeLength(frame,:);
    eL_plot.DisplayName='$t=$'+string(round(t(frame),3));
    drawnow
    pnp_movie(frame)=getframe(combined_fig,rect);
end
%combined_fig.Visible='on';
movie_fig=figure('Position',[0,0,topViewWidth+edgeLengthsWidth,figHeight]);
movie(movie_fig,pnp_movie,1,frameRate)

%Save the movie as MP4
if WantMovie==1
    myWriter=VideoWriter('pnp_movie','MPEG-4');
    myWriter.FrameRate=frameRate;   
    myWriter.Quality=100;
    open(myWriter);
    writeVideo(myWriter,pnp_movie);
    close(myWriter);
end


%% Function and Set Definitions

function cG = commGraph(conf)
    global agentNum R adjMat
    dM = zeros(agentNum,agentNum);
    for i = 1:agentNum
        for j = 1:agentNum
            if (norm(conf(i,:)-conf(j,:)) <= R) && (i ~= j)
                dM(i,j) = adjMat(i,j);
            end
        end
    end
    cG = graph(dM);
end

function v  = FlowSet(x) 
    v = 1;
end

function v  = JumpSet(x) 
    v = 0;
end


function x_dot = FlowMap(x,t)
global independentNav agentNum stateDim adjMat gamma xStar rescalingFactors

x_dot = zeros(size(x));
if independentNav==1
    for i = 1:agentNum
           index_i = (1:stateDim) + stateDim*ones(1,stateDim)*(i-1);
           x_dot(index_i)=gamma*frakN(xStar,x(index_i));
    end
else
    for i = 1:agentNum
       index_i = (1:stateDim) + stateDim*ones(1,stateDim)*(i-1);
       temp = zeros(stateDim,1);
       for j = 1:agentNum
           if adjMat(i,j) == 1
               index_j = (1:stateDim) + stateDim*ones(1,stateDim)*(j-1);
               if rescalingFactors==1            
                   temp = temp + edgeWeight(x(index_j),x(index_i))*frakN(x(index_j),x(index_i)); % Asymmetric PnP weighting
               else               
                   temp = temp + adjQ(x(index_j),x(index_i))*frakN(x(index_j),x(index_i)); % No PnP weighting
               end             
           end
           if j == agentNum
               x_dot(index_i) = temp;
           end 
       end
       V = zeros(agentNum*stateDim,1); 
       i = 1;                                                      % The leader is agent 1 
       index_i = (1:stateDim) + stateDim*ones(1,stateDim)*(i-1);
       V(index_i) = gamma*frakN(xStar,x(index_i))-x_dot(index_i); % Task component for asymmetric leader      
       %V(index_i) = gamma*frakN(xStar,x(index_i)); % Task component for symmetric leader      
       x_dot = x_dot + V;
    end
end
end

function x_plus = JumpMap(x)
    x_plus = x;
end

function boo=adjQ(y,z)
    global R
    if norm(y-z)<=R
        boo=1;
    else
        boo=0;
    end
end

%%%%%%%%%%%% Custom functions %%%%%%%%%%%%

function output = edgeWeight(y,z)
    s = norm(y-z);
    output = (r(s)*s^2)/(frakN(y,z)'*(y-z));
end

function output = r(s)
global R varrho mu omega alpha
    if s <= varrho
        output = mu;
    elseif s <= R && s>= varrho
        output = mu+omega*((s-varrho)^(1+alpha));
    else
        output = 0; %zeroing out results in the controller "breaking" edges of length greater than R.
    end
end

function output = frakNSphere(y,z)
    output = Pi(z,y) - z;
end

function output = frakN(y,z) 
    Y = TransInv(y);
    Z = TransInv(z);
    output = TransDeriv(Z)*frakNSphere(Y,Z);
    %output=frakNSphere(y,z);
end

function output = Pi(z,y)
    if y'*(z/norm(z)) >= (norm(z)+1)/2
        output = y;
    else
        output = y + ((norm(z)+1)/2 - (y'*z)/norm(z))*(z/norm(z));
    end
end

function D = RadialDistortion(input)
    global p
    x = input(1);
    y = input(2);
    r = (x^2+y^2)^(1/2);
    rp= ((x^2)^p+(y^2)^p)^(1/(2*p));
    D = rp/r;
end

function D = RadialDistortionDeriv(input)
    global p
    x = input(1);
    y = input(2);
    r = x^2+y^2;
    rp = (x^2)^p + (y^2)^p ;
    gx = RadialDistortion(input)*(((x^2)^p)/(x*rp)-(x)/(r));
    gy = RadialDistortion(input)*(((y^2)^p)/(y*rp)-(y)/(r));
    D = [gx,gy];
end

function Z = Trans(input)
    Z = RadialDistortion(input)*input; 
end

function Z = TransInv(input)
    global p
    x = input(1);
    y = input(2);
    r = (x^2 + y^2)^(1/2);
    rp= ((x^2)^p+(y^2)^p)^(1/(2*p));
    Z = (r/rp)*input;
end

function DZ = TransDeriv(input)
    x = input(1);
    y = input(2);
    DZ = RadialDistortion(input)*eye(2) + [x x;y y]*diag(RadialDistortionDeriv(input));
end