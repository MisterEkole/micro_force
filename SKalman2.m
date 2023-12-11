function [sys,x0,str,ts] = SKalman(t,x,u,flag,A,C,Q,R,x_init,P_init,Tech)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes(x_init,P_init,Tech);
    case 2
        sys = mdlUpdate(t,x,u,A,C,Q,R);
    case 3
        sys = mdlOutputs(t,x,u,A,C,Q,R);
    case 9
        sys = [];
    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts] = mdlInitializeSizes(x_init,P_init,Tech)
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 12;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0 = [x_init ; reshape(P_init,9,1)];
str = [];
ts  = [Tech 0];

function sys = mdlUpdate(t,x,u,A,C,Q,R)
xstate = x(1:3);
Pcov = reshape(x(4:end),3,3);

xpred = A*xstate;
Ppred = A*Pcov*A' + Q;
K = Ppred*C'/(C*Ppred*C' + R);
xest = xpred + K*(u - C*xpred);
Pest = (1 - K*C)*Ppred;

sys = [xest; reshape(Pest,9,1)];

function sys = mdlOutputs(t,x,u,A,C,Q,R)
sys = x(1:3);
