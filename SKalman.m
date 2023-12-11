function [sys,x0,str,ts]=SKalman(t,x,u,flag,A,C,Q,R,x_init,P_init,Tech)
% KalmanForceCte estime par filtrage de Kalman non stationnaire l'�tat �tendu de
% la tige dont la dynamique est du 2e ordre. L'�tat �tendu X[n] est de dimension 3
% et inclu la force externe Fext en 3e composante.

% Comme Fext est inclus dans l'�tat, l'�quation d'�tat n'a pas d'entr�e :
%      X[n+1] = A X[n] + W[n]
%      y[n]   = C X[n] + V[n]

% y[n] est de dimension 1 et est le d�placement x[n] de la tige.
% L'initialisation des param�tres est faite dans initialisation.m

% d�tail des param�tres de la sfunction :
% =======================================
% x contient l'�tat estim� du processus suivi de la matrice de covariance Pcov de l'erreur
% d'estimation sous la forme d'une ligne : 
% x = [xstate(1) ... xstate(3) Pcov(1,1) ... Pcov(3,1) ... ... Pcov(1,3) Pcov(3,3)]'
% x est de dimension (3 + 3*3) = 12

% l'entr�e u de la sfunction est le d�placement mesur� (bruit�) de la tige :
% u = [y_with_noise] de dimension 1

% la sortie de la sfunction correspond au d�placement estim� de la tige et de la force Fext :
% sys = [y Fext]' et est de dimension 2

% A matrice d'�tat 3x3
% C matrice de sortie 1x3

% Q matrice de covariance 3x3 du bruit d'�tat W
% R matrice de covariance 1x1 du bruit de mesure V

% x_init �tat initial estim� de x[0]
% P_init matrice de covariance initiale estim�e 3x3 de l'erreur d'estimation 
% Tech p�riode d'�chantillonnage en seconde

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes(x_init,P_init,Tech);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,                                                
    sys = mdlUpdate(t,x,u,A,C,Q,R); 

  %%%%%%%%%%
  % Output %
  %%%%%%%%%%
  case 3,                                                
    sys = mdlOutputs(t,x,u,A,C,Q,R);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,                                                
    sys = []; % do nothing

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['unhandled flag = ',num2str(flag)]);
end

%end dsfuncKalman

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts] = mdlInitializeSizes(x_init,P_init,Tech)

sizes = simsizes;
sizes.NumContStates  = 0;  % on est en discret
sizes.NumDiscStates  = 12; % �tat (3 composantes)+ �l�ments matrice de cov (9 composantes).
sizes.NumOutputs     = 3;  % d�placement et vitesse estim�s + force externe estim�e 
sizes.NumInputs      = 1;  % d�placement bruit� mesur�
sizes.DirFeedthrough = 0;  % matrice D nulle
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% construit le vecteur x0 qui a la forme suivante :
% sys = [x_init(1) ... x_init(3) P_init(1,1) ... P_init(3,1) ... ... P_init(1,3) P_init(3,3)]'
x0 = [x_init ; reshape(P_init,9,1)];

str = [];
ts  = [Tech 0]; 

% end mdlInitializeSizes

%
%=======================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=======================================================================
%
function sys = mdlUpdate(t,x,u,A,C,Q,R)

% extraction de l'�tat estim� du processus (3 premi�res composantes de x) :
xstate = x(1:3,1);
% extraction de la matrice de covariance (les 9 composantes de x qui suivent) :
Pcov = reshape(x(4:12,1),3,3);

% Etape de pr�diction (nota : pas de "commande")
xpred = A*xstate ;
Ppred = A*Pcov*A' + Q;

% calcul du gain de Kalman (non stationnaire)
K = Ppred*C'*inv(C*Ppred*C' + R);

% Etape d'estimation � l'aide de l'observation du d�placement bruit� u de la tige
xest = xpred + K * (u - C * xpred); 
Pest = Ppred -K*C*Ppred;

% construit le vecteur sys qui a la forme suivante :
% sys = [xest(1) ... xest(3) Pest(1,1) ... Pest(3,1) ... ... Pest(1,3) Pest(3,3)]'
sys = [xest ; reshape(Pest,9,1)];

% end mdlDerivatives

%
%=============================================================================
% mdlOutputs
% Return the output vector for the S-function
%=============================================================================
%
function sys = mdlOutputs(t,x,u,A,C,Q,R)

% x contient l'�tat estim� du processus suivi de la matrice de covariance Pcov de l'erreur
% d'estimation sous la forme d'une ligne : 
% x = [xstate(1) ... xstate(4) Pcov(1,1) ... Pcov(4,1) ... ... Pcov(1,4) Pcov(4,4)]'

% extraction de l'�tat du processus (3 premi�res composantes de x) :
sys = x(1:3,1);

% end mdlOutputs
