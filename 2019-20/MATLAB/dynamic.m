% Script to find the most efficient speed profile for a given track data
% in the form of x,y and z coordinates from a fixed origin.
%
% One can vary the resolution of the profile. Higher the resolution slower
% the speed and more erratic the speed profile. Lower the resolution faster
% the speed, smoother the speed profile but more energy consumed. Optimum
% resolution is around 50-200.
%
% One can vary the number of times fmincon solves the solution in options.
% Fmincon automatically stops when speed profile is the best; however, it
% may be the case that fmincon takes too many iterations and takes to long,
% and hence, one can restrict the number of evaluations.
%
% Author: Aayush Chowdhry
trackData = table2array(readtable('td.xls'));

% Process the track data to get the x,y, & z values at various distances 
% covered by the vehicle as it goes around the track

x=trackData(:,1).* 111000;
y=trackData(:,2).* cosd(trackData(:,1))*111321;
z=(trackData(:,3).^2+trackData(:,4).^2+trackData(:,5).^2).^(1/2);
earth=ones(length(z),1)*6378000;
z=z-earth;
z=z-min(z);
distance2D=[0];
distance3D=[0];
for i=1:length(trackData(:,1))-1
    deltax=x(i+1)-x(i);
    deltay=y(i+1)-y(i);
    deltaz=z(i+1)-z(i);
    d2D=sqrt(deltax^2+deltay^2);
    d3D=sqrt(deltax^2+deltay^2+deltaz^2);
    distance2D=[distance2D;distance2D(i)+d2D];
    distance3D=[distance3D;distance3D(i)+d3D];
end
trackData=removedupes([x,y,smooth(z),distance2D,distance3D]);


% Modify the track data to include the number of laps. Note that only
% distance will increase, the x,y, & z values will circle back.

% num_laps = 1; 
% n_laps = [];
% for i = 1:num_laps
%     n_laps = [n_laps(1:length(n_laps)-1,:); trackData + [0,0,0,distance2D(length(distance2D))]*(i-1),distance3D(length(distance3D))*(i-1)];
% end
% trackData = n_laps;
% trackData=[1,0,1,1,0;2,0,2,2,sqrt(2);3,0,1,3,sqrt(8)];

% Change the distance to the selected resolution. Resolution should ideally
% be proportional to number of laps. Create gridded interpolants for x,y, &
% z so that one can obtain the respective values for a specified distance.

% x=griddedInterpolant(trackData(:,4),trackData(:,1));
% y=griddedInterpolant(trackData(:,4),trackData(:,2));
z=griddedInterpolant(trackData(:,5),trackData(:,3));
% hangle=heading(x,y,distance);
vangle=griddedInterpolant(trackData(:,5),trackangle(z,trackData(:,5)));
vel=transpose(linspace(0,30,31));
time=transpose(linspace(0,10,1440));

% Assign the parameters for the costrr function and use decorator to keep
% them fixed and make cost only a function of the speed profile.

mass=96;
g=9.8;
gr=1;
tr=1;
cr=0.03;
ca=0.01;
cc=0;
k=10;
final=trackData(length(trackData(:,5)),5);
avgD=final/time(length(time));

cost_array=Inf(length(vel),length(time));
distance_array=Inf(length(vel),length(time));
pointer_array=Inf(length(vel),length(time)-1);

for col=1:length(time)
    t=1;
    for row=1:length(vel)
        v=vel(row);
        if col==1
            cost_array(row,col)=0;
            distance_array(row,col)=0;
        else
            for i=1:length(vel)
                if col==2
                    prevv=0;
                else
                    prevv=vel(i);
                end
                avgV=(v+prevv)/2;
                acc=(v-prevv)/t;
                distance=avgV*t+distance_array(i,col-1);
                va=vangle(distance_array(i,col-1));
                if distance>=final+final/10 || acc<=10
                    costrr=Inf;
                else
                    costrr=cost(v,prevv,t,va,mass,g,ca,cr,tr,gr,k,avgD)+cost_array(i,col-1);
                end
                if i==1
                    mincost=costrr;
                    mindist=distance;
                    pointer=i;
                elseif costrr<=mincost
                    mincost=costrr;
                    mindist=distance;
                    pointer=i;
                end
            end
            cost_array(row,col)=mincost;
            distance_array(row,col)=mindist;
            pointer_array(row,col-1)=pointer;
        end
    end    
end

for row=1:length(vel)
    lastcol=length(cost_array(row,:));
    if row==1
        minimum=cost_array(row,lastcol);
        minrow=row;
    elseif cost_array(row,lastcol)<minimum
        minimum=cost_array(row,lastcol);
        minrow=row;
    end
end

speedprofile=[];
distanceprofile=[];
col=length(pointer_array(minrow,:));
while col>0
    minrow=pointer_array(minrow,col);
    speedprofile=[speedprofile;vel(minrow)];
    distanceprofile=[distanceprofile;vel(minrow)];
    col=col-1;
end

figure(1);
plot(distanceprofile,speedprofile);
figure(2);
plot(trackData(:,5), trackData(:,3));


% Helper functions

function cost=cost(v,prevv,t,va,mass,g,ca,cr,tr,gr,k,avgD)
%
%
avgV=(v+prevv)/2;
acc=(v-prevv)/t;
dist=avgV*t;
f=mass*acc+ca*avgV^2+cr*mass*g*cosd(va)+mass*g*sind(va);
tor=tr/gr*f;
if tor<0
    tor=0;
end
e=gr/tr*dist*tor;
cost=e+k*(avgD-dist)^2;
end

function trackangle = trackangle(z,hor)
% Returns value of vertical angle of the vehicle (elevation of the track).
%
% Parameter z: The z position with respect to a fixed origin at a given
%           distance on the track
% Precondition: z is a gridded interpolant function
%
% Parameter hor: The two dimensional distance the vehicle covers
% Precondition: hor is an array with float attributes

angle=[];
for i = 1:length(hor)
    position=hor(i);
    slope=(z(position+0.01)-z(position-0.01))/(0.02);
    angle=[angle;atand(slope)];
end
trackangle=angle;
end

function heading = heading(x,y,hor)
% Returns value of the change in the orientation (heading) of the vehicle
% at each distance for a given track
%
% Parameter x: The x position with respect to a fixed origin at a given
%           distance on the track
% Precondition: x is a gridded interpolant function
%
% Parameter y: The y position with respect to a fixed origin at a given
%           distance on the track
% Precondition: y is a gridded interpolant function
%
% Parameter hor: The two dimensional distance the vehicle covers
% Precondition: hor is an array with float attributes

hd=[];
for i = 1:length(hor)
    position=hor(i);
    deltax=x(position+0.001)-x(position);
    deltay=y(position+0.001)-y(position);
    if deltax==0
      hd=[hd;0];
    else
      hd=[hd;atan(deltay/deltax)];
    end
end
heading=hd;
end

function modifiedarray=removedupes(array)
% Returns an array without duplicate points. 
%
% Duplicate points are identified as points that have the same x,y and
% 2D distance coordinates.
%
% Parameter array: The array to modify
% Precondition: Array is an array with float attributes in the form of
% x,y,z and 2-dimensional distance.
if isempty(array) || length(array(:,1))==1
    new= array;
else
    eq=true;
    for i=length(array(1,:))
        if i ~= 3
            eq=array(1,i)==array(2,i) & eq;
        end
    end
    if eq
        new=removedupes(array(2:length(array(:,1)),:));
    else 
        new=[array(1,:);removedupes(array(2:length(array(:,1)),:))];
    end
end
modifiedarray=new;
end