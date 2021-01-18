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
distance=[0];
for i=1:length(trackData(:,1))-1
    deltax=x(i+1)-x(i);
    deltay=y(i+1)-y(i);
    d=sqrt(deltax^2+deltay^2);
    distance=[distance;distance(i)+d];
end
trackData=removedupes([x,y,smooth(z),distance]);


% Modify the track data to include the number of laps. Note that only
% distance will increase, the x,y, & z values will circle back.

num_laps = 1;
n_laps = [];
for i = 1:num_laps
    n_laps = [n_laps(1:length(n_laps)-1,:); trackData + [0,0,0,distance(length(distance))]*(i-1)];
end
trackData = n_laps;
% trackData=[1,0,1,1;2,0,2,2;3,0,1,3];

% Change the distance to the selected resolution. Resolution should ideally
% be proportional to number of laps. Create gridded interpolants for x,y, &
% z so that one can obtain the respective values for a specified distance.

distance =linspace(trackData(1,4),trackData(length(trackData(:,1)),4),100*num_laps);
distance = transpose(distance);
x=griddedInterpolant(trackData(:,4),trackData(:,1));
y=griddedInterpolant(trackData(:,4),trackData(:,2));
z=griddedInterpolant(trackData(:,4),trackData(:,3));


% Assign the parameters for the costrr function and use decorator to keep
% them fixed and make cost only a function of the speed profile.

gr=5;
tr=0.355;
m=96;
g=9.8;
cr=0.01;
ca=0.03675;
cc=0;
tmax=2;
k=0;
hangle=heading(x,y,distance);
vangle=trackangle(z,distance);

cost = @(v)costrr(gr, tr, distance, v, m, g, cr, ca, cc, tmax, k, vangle,hangle);


% Assign desired suitable fmincon parameters to find the speed profile that
% yields the lowest cost. The more the evaluations, the lesser the speed.

res = length(distance);
v0 = ones(res,1)*0.5;
A = [];
b = [];
Aeq = [];
beq = [];
lb = ones(res,1)*0.01;
ub = ones(res,1)*30;
ub(1) = 0.01;
nonlcon = [];
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 500000, 'MaxIterations', 3000);
v = fmincon(cost, v0, A, b, Aeq, beq, lb, ub, nonlcon,options);
v = smooth(v);

% Plot a graph for the height of the track versus two dimensional distance
% covered in the xy plane from the start to get to that point on the track.
% Plot a graph for the speed profile as a versus the same.

figure(1);
plot(distance,v);
figure(2);
plot(trackData(:,4), trackData(:,3));


% Primary Cost Function

function costrr=costrr(gr,tr,hor,v,m,g,cr,ca,cc,tmax,k,vangle,hangle)
% Returns the cost of a given speed profile.
%
% The cost is a number of the speed profile which is higher if more energy
% is consumed and the difference between the maximum specified time and the
% time taken multplied by a given constant is higher.
%
% General Precondition: All arrays are of the same number of rows
%
% Parameter gr: The gear ratio of the motor.
% Predcondition: gr is a real number > 0
%
% Parameter tr: The radius of the tire.
% Precondition: tr is a real number > 0.
%
% Parameter hor: The two dimensional distance the vehicle covers
% Precondition: hor is an array with float attributes
%
% Parameter v: The speed profile of the vehicle
% Precondition: v is an array with float attributes
%
% Parameter m: The constant mass of the vehicle
% Precondition: m is a real number > 0
%
% Parameter g: The acceleration due to gravity
% Precondition: g is a real number > 0
%
% Parameter cr: The specific coefficient of rolling resistance of the
%           vehicle
% Precondition: cr is a real number >= 0
%
% Parameter ca: The specifc coefficeint of air resistance of the vehicle
% Precondition: ca is a real number
%
% Parameter cc: The specific coefficent of the vehicle to approximate
%           cornering resistance
% Precondition: cc is a real number >= 0
%
% Parameter tmax: The maximum time permitted to be taken
% Precondition: tmax is a number > 0
%
% Parameter k: The constant that decides the enforcement of t==tmax
% Precondition: k is a number
%
% Parameter vangle: The verticle angle of the car in degrees
% Precondition: vangle is an array with float attributes
%
% Parameter hangle: The change in heading of the car in radians
% Precondition: hangle is an array with float attributes

e=0;
time=0;
for i=1:length(hor)-1
    va=vangle(i);
    vel=(v(i)+v(i+1))/2;
    dist=(hor(i+1)-hor(i))/cosd(va);
    time=time+dist/vel;
    acc=vel*(v(i+1)-v(i))/dist;
    f=m*acc+ca*vel^2+cr*m*g*cosd(va)+m*g*sind(va)+m*g*cc*vel*hangle(i);
    t=tr/gr*f;
    if t<0
        t=0;
    end
%     ADD EFFICIENCY
    e=e+gr/tr*dist*t;
    if i==length(hor)-1
        va=vangle(i+1);
        dist=(hor(i+1)-hor(i))/cosd(va);
        time=time+dist/vel;
        acc=vel*(v(i+1)-v(i))/dist;
        f=m*acc+ca*vel^2+cr*m*g*cosd(va)+m*g*sind(va)+m*g*cc*vel^hangle(i+1);
        t=tr/gr*f;
        if t<0
            t=0;
        end
        e=e+gr/tr*dist*t;
    end
end
costrr=e+k*(time-tmax)^2;
end


% Helper functions

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
factor=hor(length(hor))/length(hor)/2;
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
