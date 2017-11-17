%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%               ROBOTICS FUNDAMENTALS
%   Sammy N. M. Hasan                             2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc

%%
%Functions

dist = @(x1,y1,x2,y2) sqrt((x2-x1)^2+(y2-y1)^2);

% Part II
%% Defining the variables

exA  = exist('a');
exXc = exist('xc');
exYc = exist('yc');

% check if values have been already defiend
if ~exA
    a = 0;
end

if ~exXc
    xc = 290*cosd(30);
end

if ~exYc
    yc = 290*sind(30)+60; % +60 is due to an artifact
end



lp = 130;   Sa = 170;   l = 130;

Xpb1 = 0;       Xpb2 = 2*290*cosd(30);      Xpb3 = 290*cosd(30);
Ypb1 = 0;       Ypb2 = 0;                   Ypb3 = 290*cosd(30) + 251;


% xws90 = [];
% yws90 = [];
% for xc = 0:500
%     for yc = 0:500
%% Calculating the PBi positions


Xpp1 = xc + lp*cosd(a + 210);       Xpp2 = xc + lp*cosd(a + 330);       Xpp3 = xc + lp*cosd(a + 90);
Ypp1 = yc + lp*sind(a + 210);       Ypp2 = yc + lp*sind(a + 330);       Ypp3 = yc + lp*sind(a + 90);


Xt2 = Xpp2 - Xpb2;      Xt3 = Xpp3 - Xpb3;
Yt2 = Ypp2 - Ypb2;      Yt3 = Ypp3 - Ypb3;


%% Inverse Kinematics

%%%%%%%%%%%%%%%%% Angles 1
c_psi_1 = (Xpp1^2 + Ypp1^2 - Sa^2 - l^2)/(2*Sa*l);
s_psi_1 = real(sqrt(1-c_psi_1^2));

psi1_A = atan2(s_psi_1,c_psi_1);
psi1_B = atan2(-s_psi_1,c_psi_1);

th1_A = atan2(Ypp1,Xpp1) + atan2(l*s_psi_1,Sa+l*c_psi_1);
th1_B = atan2(Ypp1,Xpp1) + atan2(-l*s_psi_1,Sa+l*c_psi_1);

%%%%%%%%%%%%%%%%% Angles 2
% http://www.ohio.edu/people/williar4/html../PDF/M97Paral.pdf
E = 2*(Xpp2 - Xpb2)*Sa;
F = 2*(Ypp2 - Ypb2)*Sa;
G = l^2 - Sa^2 - (Xpp2 - Xpb2)^2 - (Ypp2 - Ypb2)^2;

SQRT = real(sqrt(E^2+F^2-G^2));

th2_A = 2*atan2(-F + SQRT,G-E);
th2_B = 2*atan2(-F - SQRT,G-E);

psi2_A = atan2(Ypp2 - Ypb2 - Sa*sin(th2_A),Xpp2 - Xpb2 - Sa*cos(th2_A)) - th2_A;
psi2_B = atan2(Ypp2 - Ypb2 - Sa*sin(th2_B),Xpp2 - Xpb2 - Sa*cos(th2_B)) - th2_B;


% %Alternative
% c_psi_2 = (Xt2^2 + Yt2^2 - Sa^2 - l^2)/(2*Sa*l);
% s_psi_2 = real(sqrt(1-c_psi_2^2));
%
% psi2_A = atan2(s_psi_2,c_psi_2);
% psi2_B = atan2(-s_psi_2,c_psi_2);
%
% th2_A = atan2(Ypp2,Xpp2) + atan2(l*s_psi_2,Sa+l*c_psi_2);
% th2_B = atan2(Ypp2,Xpp2) + atan2(-l*s_psi_2,Sa+l*c_psi_2);

%%%%%%%%%%%%%%%%% Angles 3

c_psi_3 = (Xt3^2 + Yt3^2 - Sa^2 - l^2)/(2*Sa*l);
s_psi_3 = real(sqrt(1-c_psi_3^2));

psi3_A = atan2(s_psi_3,c_psi_3);
psi3_B = atan2(-s_psi_3,c_psi_3);

th3_A = atan2(Yt3,Xt3) + atan2(l*s_psi_3,Sa+l*c_psi_3);
th3_B = atan2(Yt3,Xt3) + atan2(-l*s_psi_3,Sa+l*c_psi_3);


%%%%%%%%%%%%%%%%%
%DEGREES
deg_psi1_A = radtodeg(psi1_A)
deg_psi1_B =radtodeg(psi1_B)
deg_th1_A = radtodeg(th1_A)
deg_th1_B = radtodeg(th1_B)

deg_psi2_A = radtodeg(psi2_A)
deg_psi2_B =radtodeg(psi2_B)
deg_th2_A = radtodeg(th2_A)
deg_th2_B = radtodeg(th2_B)

deg_psi3_A = radtodeg(psi2_A)
deg_psi3_B =radtodeg(psi2_B)
deg_th3_A = radtodeg(th2_A)
deg_th3_B = radtodeg(th2_B)


len_11 = dist(Xpb1,Ypb1,Sa*cos(th1_A),Sa*sin(th1_A));
len_12 = dist(Sa*cos(th1_A),Sa*sin(th1_A),Xpp1,Ypp1);

len_21 = dist(Xpb2,Ypb2,Xpb2+Sa*cos(th2_A),Ypb2+Sa*sin(th2_A));
len_22 = dist(Xpb2+Sa*cos(th2_A),Ypb2+Sa*sin(th2_A),Xpp2,Ypp2);

len_31 = dist(Xpb3,Ypb3,Xpb3+Sa*cos(th3_A),Ypb3+Sa*sin(th3_A));
len_32 = dist(Xpb3+Sa*cos(th3_A),Ypb3+Sa*sin(th3_A),Xpp3,Ypp3);

if (len_11 > 170.01) || (len_21 > 170.01) || (len_31 > 170.01) || (len_12 > 130.01) || (len_22 > 130.01) || (len_32 > 130.01)

    disp('Boundary has been breached, report to second Foundation')
    xc = 290*cosd(30);
    yc = 290*sind(30);
    part2


% else
%     xws90(end+1) = xc;
%     yws90(end+1) = yc;
end



plot([0,Sa*cos(th1_A),Xpp1,xc],[0,Sa*sin(th1_A),Ypp1,yc])
hold on
scatter([0,Sa*cos(th1_A),Xpp1,xc],[0,Sa*sin(th1_A),Ypp1,yc])
%
plot([502,Xpb2+Sa*cos(th2_A),Xpp2,xc],[0,Ypb2+Sa*sin(th2_A),Ypp2,yc])
% hold on
scatter([Xpb2,Xpb2+Sa*cos(th2_A),Xpp2,xc],[0,Ypb2+Sa*sin(th2_A),Ypp2,yc])


plot([Xpb3,Xpb3+Sa*cos(th3_A),Xpp3,xc],[Ypb3,Ypb3+Sa*sin(th3_A),Ypp3,yc])
% hold on
scatter([Xpb3,Xpb3+Sa*cos(th3_A),Xpp3,xc],[Ypb3,Ypb3+Sa*sin(th3_A),Ypp3,yc])

plot([Xpp1,Xpp2,Xpp3,Xpp1],[Ypp1,Ypp2,Ypp3,Ypp1],':k')
plot([Xpb1,Xpb2,Xpb3,Xpb1],[Ypb1,Ypb2,Ypb3,Ypb1],':g')

circle(xc,yc,lp)
axis([-200,600,-200,600])
axis equal;
% end
% end

[xc,yc] = ginput(1)
clf
part2
clc
