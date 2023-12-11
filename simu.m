format long;
processus            % Fen�tre de simu Simulink

% Question 1 : Param�tres du ressort magn�tique

m = 7.4e-5;      % masse tige en kg
Km = 0.0281;     % raideur ressort mag en N/m
Kv = 1.772e-5 ;     % coeff de frottement visqueux tige en N/m*s

% Question 3 : choix de W
W = 10e-9 ;

% Question 5 : Calcul mod�le d'�tat continu �tendu pour le filtrage de Kalman
% L'�tat �tendu est [x dx/dt F]'

Ae= [0 1 0;-Km/m -Kv/m 1/m;0 0 0] ; 
Be= [0;0;0] ;
Ce= [1 0 0] ; 
De= [0] ;
ModeleEtatCont = ss (Ae,Be,Ce,De);


% Question 6 : Param�tres du capteur de d�placement
Te = 0.001 ;
sigma = 0.01e-6 ;
R = sigma^2;


% Question 8 : calcul du mod�le d'�tat �tendu discret

ModeleEtatDisc = c2d(ModeleEtatCont, Te, 'zoh' );
Ad = ModeleEtatDisc.a ;
Bd = ModeleEtatDisc.b ;
Cd = ModeleEtatDisc.c ;
Dd = ModeleEtatDisc.d ;

% Calcul covariance Q du bruit d'�tat discret d� � l'�chantillonnage 
% et au modele d'incertitude associ� � la force inconnue :

Q = calcul_Q(Km,Kv,m,Te,W) ;

% Question 10 : initialisation du filtre de Kalman non stationnaire

X0 = [0 0 0]' ; 		% on suppose la tige au repos et la force nulle

% Matrice de covariance initiale estim�e de l'�tat x[0] de la tige et de la force:
P0 = [1e-17 0 0;0 1e-17 0;0 0 3e-20];

sim('processus_kalman',10);
