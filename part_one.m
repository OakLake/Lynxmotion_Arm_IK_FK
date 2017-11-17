%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%               ROBOTICS FUNDAMENTALS
%   Sammy N. M. Hasan                             2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part I
%

close all
clear
clc





%% Anon. Functions

Beta = @(T) atan2(-T(3,1),sqrt((T(3,2))^2+(T(3,3))^2)); % pitch
Alpha = @(T) atan2(T(2,1),T(1,1)); % yaw
Gamma = @(T) atan2(T(3,2),T(3,3));% roll

hyp = @(x,y) sqrt(x^2 + y^2); % hypotenuse

%% A.


% defining the parameters of the denavit hartneberg table
% for the lynxmotion arm

% using proximal method

% lynxmotion arm geometry variables
l1 = 50;    l2 = 120;   l3 = 130;   l4 = 40;    l5 = 20;

% a n-1
a0 = 0;     a1 = 0;     a2 = l2;    a3 = l3;    a4 = 0;     ae = 0;

% alpha n-1
al0 = 0;    al1 = sym('pi')/2;      al2 = 0;    al3 = 0;    al4 = -sym('pi')/2;     ale = 0;
% above sym('pi') is used because using Pi would return odd results in
% further calculations

% dn
d1 = l1;    d2 = 0;     d3 = 0;     d4 = 0;     d5 = l4;    de = l5;

% thetan , g stands for general (as in notation but not variable)
syms th1 th2 th3 th4 th5

th1 = degtorad(15);     th2 = degtorad(60);     th3 = degtorad(-30);        th4 = degtorad(-30);        th5 = 0;

g_th1 = th1;
g_th2 = th2;
g_th3 = th3;
g_th4 = th4 - sym('pi')/2; % - sym('pi')/2 is used to set the reference point with a shift.
g_th5 = th5;
g_the = 0;



% defining the transformation function that takes in DH parameters and
% returns the HT matrix for the specified reference frames (two frames).

DHtoHT_nPre_nNext = @(aN_1,alN_1,dN,thN) [cos(thN) -sin(thN) 0 aN_1;...
                               sin(thN)*cos(alN_1) cos(thN)*cos(alN_1) -sin(alN_1) -dN*sin(alN_1);...
                               sin(thN)*sin(alN_1) cos(thN)*sin(alN_1) cos(alN_1) dN*cos(alN_1);...
                               0 0 0 1];

%% Fidning the HT matrices for each joint




T01 = DHtoHT_nPre_nNext(a0,al0,d1,g_th1);
T12 = DHtoHT_nPre_nNext(a1,al1,d2,g_th2);
T23 = DHtoHT_nPre_nNext(a2,al2,d3,g_th3);
T34 = DHtoHT_nPre_nNext(a3,al3,d4,g_th4);
T45 = DHtoHT_nPre_nNext(a4,al4,d5,g_th5);
T5E = DHtoHT_nPre_nNext(ae,ale,de,g_the);

% Defined by user
% with respect to world reference,
% world ref: X: towards right
%            Y: Upwards

%  TW0 = [1 0 0 0;...
%         0 1 0 0;...
%         0 0 1 0;...
%         0 0 0 1];

% no need for TWO since pitch angle is about Y and y is in the correct
% orientation

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T0E = T05*T5E;
% T0E = T01*T12*T23*T34*T45*T5E;

% T15 = inv(T01)*T0E;


% tested for all theta = 0; results check with sketched drawing and arm
% lengths given. :)

%% plotting the workspace



plotWorkspace = 0;

if plotWorkspace == 1

% variables are :
% th1: 0>180
% th2: 0>180
% th3: -150 > 0
% th4: -90 > 90
% th5: 0 rotation does not matter
%
% itr_th5 = 0;
% itr_the = 0;

% itr_th2 = 0;
% itr_th3 = 0;
% itr_th4 = -90;
%
x = [];
y = [];
z = [];

figure
clf
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on
for itr_th1 = 0:10:180
    for itr_th2 = 0:15:180
        for itr_th3 = -150:15:0
            for itr_th4 = -180:15:0
                T01 = DHtoHT_nPre_nNext(a0,al0,d1,degtorad(itr_th1));
                T12 = DHtoHT_nPre_nNext(a1,al1,d2,degtorad(itr_th2));
                T23 = DHtoHT_nPre_nNext(a2,al2,d3,degtorad(itr_th3));
                T34 = DHtoHT_nPre_nNext(a3,al3,d4,degtorad(itr_th4));
                T45 = DHtoHT_nPre_nNext(a4,al4,d5,degtorad(itr_th5));
                T5E = DHtoHT_nPre_nNext(ae,ale,de,degtorad(itr_the));
                T0E = T01*T12*T23*T34*T45*T5E;
                xtemp = T0E(1,4);
                ytemp = T0E(2,4);
                ztemp = T0E(3,4);
                x(end+1) = xtemp;
                y(end+1) = ytemp;
                z(end+1) = ztemp;


%                  scatter3(xtemp,ytemp,ztemp);
%                  pause(0.1)

            end
        end
    end
end


scatter3(x,y,z);
xlabel('X')
ylabel('Y')
zlabel('Z')

end

%% Path Planning

xeList = [130,130,24,-24,-130,-130];
yeList = [0,24,130,130,24,0];
zeList = [0,100,142,140,100,0];
BLinDegList = [-90,0,30,20,0,-90];

xePathLine = [ linspace(xeList(1),xeList(2),20), linspace(xeList(2),xeList(3),20), linspace(xeList(3),xeList(4),20), linspace(xeList(4),xeList(5),20), linspace(xeList(5),xeList(6),20)];
yePathLine = [ linspace(yeList(1),yeList(2),20), linspace(yeList(2),yeList(3),20), linspace(yeList(3),yeList(4),20), linspace(yeList(4),yeList(5),20), linspace(yeList(5),yeList(6),20)];
zePathLine = [ linspace(zeList(1),zeList(2),20), linspace(zeList(2),zeList(3),20), linspace(zeList(3),zeList(4),20), linspace(zeList(4),zeList(5),20), linspace(zeList(5),zeList(6),20)];
BetaDegPathLine = [ linspace(BLinDegList(1),BLinDegList(2),20),...
    linspace(BLinDegList(2),BLinDegList(3),20),...
    linspace(BLinDegList(3),BLinDegList(4),20),...
    linspace(BLinDegList(4),BLinDegList(5),20),...
    linspace(BLinDegList(5),BLinDegList(6),20)];



%%% Free Path
load('Y_Free_Motion_Path.mat');
xeFreePath = @(step) 2*10^(-18)*step^4 + 0.0007*step^3 - 0.1027*step^2 + 0.8244*step + 132.95;
yeFreePath = @(step) yeFreeMP(step); %% <<-- Generated with cftool from yePathLine values, output was saved to a model
zeFreePath = @(step) -0.0587*step^2 + 5.9227*step - 4.1244;


%%% Free Path Joint space interpolation
th1FP = [0,10,80,100,170,180];
th2FP = [67,110,109,113,110,67];
th3FP = [-117,-139,-132	,-132,-139,-117];
th4FP = [-40,29,53,38,29,-40];

th1FreeMP = [ linspace(th1FP(1),th1FP(2),10), linspace(th1FP(2),th1FP(3),10), linspace(th1FP(3),th1FP(4),10), linspace(th1FP(4),th1FP(5),10), linspace(th1FP(5),th1FP(6),10) ];
th2FreeMP = [ linspace(th2FP(1),th2FP(2),10), linspace(th2FP(2),th2FP(3),10), linspace(th2FP(3),th2FP(4),10), linspace(th2FP(4),th2FP(5),10), linspace(th2FP(5),th2FP(6),10) ];
th3FreeMP = [ linspace(th3FP(1),th3FP(2),10), linspace(th3FP(2),th3FP(3),10), linspace(th3FP(3),th3FP(4),10), linspace(th3FP(4),th3FP(5),10), linspace(th3FP(5),th3FP(6),10) ];
th4FreeMP = [ linspace(th4FP(1),th4FP(2),10), linspace(th4FP(2),th4FP(3),10), linspace(th4FP(3),th4FP(4),10), linspace(th4FP(4),th4FP(5),10), linspace(th4FP(5),th4FP(6),10) ];

%% Obstacle Avoidance:

% defining the obstacle as a cylinder with z from -inf -> inf,
% thus only defining the x,y coordinated in space

obs_diameter = 23;
obs_xc = 80;
obs_yc = 65;
obstacleX = zeros(obs_diameter);
obstacleY = zeros(obs_diameter);

% populate coordinates of obstacle
indexX = 1;
indexY = 1;
for x = -11:11
    for yy = -11:11
        y = -(yy);
        r = sqrt((x)^2+(y)^2); % w.r.t obstacle centre at (75,75)
        if r <= 11 % radius
            obstacleX(indexX) = 80+x;
            obstacleY(indexY) = 65+y;
        end
        indexX = indexX + 1;
        indexY = indexY + 1;
    end
end
% %%
% for ix = 1:23
%     for iy = 1:23
%         xx = obstacleX(ix,iy);
%         yy = obstacleY(ix,iy);
%         scatter(xx,yy)
%         hold on
%     end
% end
% %%
% Now to modify the old path. xePathLine & yePathLine
obsPathX = [];
obsPathY = [];

xpAgg = [];
ypAgg = [];
for p = 1:numel(xePathLine)
    xp = xePathLine(p);
    yp = yePathLine(p);

    %anyX = any(abs(74.5-obstacleX)<1);
    %anyY = any(abs(74.5-obstacleX)<1);
    % round(v)

    membershipX = ismember(round(xp),obstacleX);
    membershipY = ismember(round(yp),obstacleY);

    % if both values are in the cylinder, then update to a new point
    if membershipX == 1 && membershipY == 1
        obstructed = 1;
        coss = xp/hyp(xp,yp);
        sinn = yp/hyp(xp,yp);

        while obstructed
            xp = xp - 1;%*coss;
            yp = yp - 1;%*sinn;

            xpAgg(end+1) = xp;
            ypAgg(end+1) = yp;

            membershipX = ismember(round(xp),obstacleX);
            membershipY = ismember(round(yp),obstacleY);

            if membershipX == 0 || membershipY == 0
                disp('xpBreak: '),disp(xp)
                disp('ypBreak: '),disp(yp)
                break
            end

        end

    end


    obsPathX(end+1) = xp;
    obsPathY(end+1) = yp;

end



%% Inverse Kinematics

% Note, Theta 5 has been ignored since it does not change the coordinates
% of EE but only orientation, which is not required in this application

% initialise for safety
xe = -130;
ye = 0;
ze = 0;
beta = degtorad(-90);

clc

% define the coordinates, either constants or variables defined by a
% function


% xe = T0E(1,4);
% ye = T0E(2,4);
% ze = T0E(3,4);



% iTheta is the angle
% iTheta = 0;

% xe = (300-iTheta*2)*cos(iTheta);
% ye = (300-iTheta*2)*sin(iTheta);
% ze = (300-iTheta*5)*sin(iTheta);

signz = 1;
signx = 1;
signy = 1;

xfeAgg = [];
yfeAgg = [];
zfeAgg = [];

% betaPath is the pitch angle of the EE, created as "linspace"
% betaPath = 0:pi/800:pi/2;
% bIndex = 0; % initialise the index for accessing betaPath


[m,n] = size(xePathLine);


% use these to toggle between different path modes
chooseLinePath = 1; %make n = 100 down below
chooseFreePath = 0; %make n = 100 down below
jointSpaceFreeMotion = 0; %make n = 50 down below
obstacleAvoidancePath = 1; %make n = 100 down below
%                               ..
%                               ..
for iTheta = 1:100 % < ............  n is here
%specify end effector location
% update EE coordinates for next iTheta (next step). "iTheta:inverseTheta"

if chooseLinePath == 1
    xe = xePathLine(iTheta);
    ye = yePathLine(iTheta);
    ze = zePathLine(iTheta);
end

if chooseFreePath == 1
    xe = xeFreePath(iTheta);
    ye = yeFreePath(iTheta);
    ze = zeFreePath(iTheta);
end

if obstacleAvoidancePath == 1
    xe = obsPathX(iTheta);
    ye = obsPathY(iTheta);
    ze = zeFreePath(iTheta);
end

% xe = [130,130,24,-24,-130,-130];
% ye = [0,24,130,130,24,0];
% ze = [0,100,142,140,100,0];
% BLinDeg = [-90,0,30,20,0,-90];

% xe = -130;
% ye = 0;
% ze = 0;
% beta = degtorad(-90);

beta = degtorad(BetaDegPathLine(iTheta));


% specify pitch angle
%beta = sym('pi')/2 - abs(Beta(T0E));%*cos(Gamma(T0E));

% increment betaPath access index to get next pitch angle for EE
% bIndex = bIndex+1;
%beta = ;%degtorad(-90);%betaPath(bIndex);


% re: euclidean distance of EE in the X,Y plane from the base of the robot
% or {0}

if jointSpaceFreeMotion == 0
re = sqrt(xe^2+ye^2);

%r4 : euclidean distance of {4}, extracted from re and the pitch angle of
%the EE, which defines the angle theta4
r4 = re - abs((l4+l5)*cos(beta));

% sign: defines in what orientation/quadrant the end efector is with respect to {4}
% this has been done after empirical testing. and due to the shift in z
% coordinates w.r.t the base of the robot.
sign = cos(Gamma(T0E));

% z4: z coordinate of {4}
z4 = ze - ((l4+l5)*sin(beta)) - l1;  % was after first - sign*abs


% IK as defined in the report.
%
IK_cos_3 = (r4^2 + z4^2 - l2^2 - l3^2) / (2*l2*l3);
IK_sin_3 = [real(sqrt(1-IK_cos_3^2)) , -real(sqrt(1-IK_cos_3^2))];


IK_th1 = atan2(ye,xe);

IK_th3_A = atan2(IK_sin_3(1),IK_cos_3);
IK_th3_B = atan2(IK_sin_3(2),IK_cos_3);

IK_th2_A = atan2(z4,r4) + atan2(abs(l3*sin(IK_th3_A)),l2+l3*cos(IK_th3_A));
IK_th2_B = atan2(z4,r4) + atan2(abs(l3*sin(IK_th3_B)),l2+l3*cos(IK_th3_B));

IK_th4_A = sign*beta - IK_th3_A - IK_th2_A ;
IK_th4_B = sign*beta - IK_th3_B - IK_th2_B ;

IK_th1_deg = vpa(radtodeg(IK_th1));
IK_th2_deg_A = vpa(radtodeg(IK_th2_A));
IK_th2_deg_B = vpa(radtodeg(IK_th2_B));

IK_th3_deg_A = vpa(radtodeg(IK_th3_A));
IK_th3_deg_B = vpa(radtodeg(IK_th3_B));

IK_th4_deg_A = vpa(radtodeg(IK_th4_A));
IK_th4_deg_B = vpa(radtodeg(IK_th4_B));

% IK works for extreme limits (simple reaching boundary) tests

disp('IK_th1_deg'), disp(IK_th1_deg)
disp('IK_th2_deg_AB'), disp([IK_th2_deg_A,IK_th2_deg_B])
disp('IK_th3_deg_AB'), disp([IK_th3_deg_A,IK_th3_deg_B])
disp('IK_th4_degAB'), disp([IK_th4_deg_A,IK_th4_deg_B])

end
% Getting the location of each joint from the inverskinematic equations in
% order to plot the robot.


if jointSpaceFreeMotion == 1

   IK_th1   = degtorad(th1FreeMP(iTheta));
   IK_th2_A = degtorad(th2FreeMP(iTheta));
   IK_th3_B = degtorad(th3FreeMP(iTheta));
   IK_th4_B = degtorad(th4FreeMP(iTheta));

  pause(0.1)
end

rf2 = l2*cos(IK_th2_A);
xf2 = rf2*cos(IK_th1);
yf2 = rf2*sin(IK_th1);
zf2 = l2*sin(IK_th2_A)+l1;

rf3 = rf2 + l3*cos(IK_th2_A+IK_th3_B);
xf3 = rf3*cos(IK_th1);
yf3 = rf3*sin(IK_th1);
zf3 = zf2 + l3*sin(IK_th2_A+IK_th3_B);

rf4 = rf3 + l4*cos(IK_th2_A+IK_th3_B+IK_th4_B);
xf4 = rf4*cos(IK_th1);
yf4 = rf4*sin(IK_th1);
zf4 = zf3 + l4*sin(IK_th2_A+IK_th3_B+IK_th4_B);

rfe = rf4 + l5*cos(IK_th2_A+IK_th3_B+IK_th4_B);
xfe = rfe*cos(IK_th1);
yfe = rfe*sin(IK_th1);
zfe = zf4 + l5*sin(IK_th2_A+IK_th3_B+IK_th4_B);

% the Agg_regate stores all values of the end effetors location in order to
% plot the path which the EE has traversed
xfeAgg(end+1) = xfe;
yfeAgg(end+1) = yfe;
zfeAgg(end+1) = zfe;


% plotting
plot3([0,0,xf2,xf3,xf4,xfe],[0,0,yf2,yf3,yf4,yfe],[0,l1,zf2,zf3,zf4,zfe])
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on
grid on
% circle(0,0,80)
% circle(0,0,300)
plot3(xfeAgg,yfeAgg,zfeAgg);
scatter3([0,0,xf2,xf3,xf4,xfe],[0,0,yf2,yf3,yf4,yfe],[0,l1,zf2,zf3,zf4,zfe])



% the circles plotted are merely to visualise the movement of the robot in
% the x,y plane to better understand and avoid problems with 3D visuals.
circle(0,0,double(rf2))
circle(0,0,double(rf3))
circle(0,0,double(rf4))

circle(0,0,double(rfe))


% setting the limit of the axes, so the figure is not jumpy with every plot
xlim([-320 320])
ylim([-320 320])
zlim([-50 320])


% hold off;

% pause in order to visualise
pause(0.1)

% xe = 300*cos(iTheta);
% ye = 300*sin(iTheta);
% ze = (100-iTheta*5)*sin(iTheta) + 100;
end

% implementation testing.
% angles given w.r.t DH frames
% ALPHA0E = Alpha(T0E);
% GAMMA0E =Gamma(T0E);
%
% disp('Alpha 0E'), disp(ALPHA0E)
% disp('Gamma 0E'), disp(GAMMA0E)
