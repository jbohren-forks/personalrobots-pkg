%% Ttrajectory = GetInterpolatedHandTrajectory(T0,T1,deltat, deltaq)
%%
%% Ttrajerctory is a 12xN array of 3x4 matrices
function Ttrajectory = GetInterpolatedHandTrajectory(T0,T1,deltat, deltaq)

if( ~exist('deltat','var') )
    deltat = 0.01;
end

if( ~exist('deltaq','var') )
    deltaq = 0.05;
end

q0 = QuatFromRotationMatrix(T0(1:3,1:3));
q1 = QuatFromRotationMatrix(T1(1:3,1:3));
t0 = T0(1:3,4);
t1 = T1(1:3,4);

if( norm(q0+q1) < norm(q0-q1) )
    q1 = -q1;
end

numsegments = 1+max(norm(q0-q1)/deltaq, norm(t0-t1)/deltat);
times = 0:(1/numsegments):1;

Ttrajectory = zeros(12,length(times));
for i = 1:length(times)
    curtime = times(i);
    Ttrajectory(1:9,i) = reshape(RotationMatrixFromQuat(QuatSlerp(q0,q1,curtime,1e-6)),[9 1]);
    Ttrajectory(10:12,i) = t0*(1-curtime)+t1*curtime;
end
