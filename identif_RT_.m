%*****************************************%
%Acquisition des donn�es temps-r�el Dspace%
%*****************************************%

%mise � zero du courant bobine
%identif_courant contient la structure de description de la variable qu'on va
%utiliser pour identifier le mod�le
identif_courant = mlib('GetTrcVar', 'Model Root/I_bob/Value');
%courant des bobines � zeo, mise en oscillation de la tige
mlib('Write', identif_courant, 'Data', 0);
%d�lai avant acquisition
pause(1)

%identif_desc contient la structure de description de la variable qu'on va
%utiliser pour identifier le mod�le
identif_desc = mlib('GetTrcVar', 'Model Root/Sum/Out1');

%Configuration de l'acquisition de donn�es
mlib('Set','TraceVars', identif_desc,'Delay', 0,'NumSamples',5000);

% D�marrage de l'acquisition
mlib('StartCapture');


% Attente de la fin de l'acquisition
while mlib('CaptureState')~=0, end;

% Transfert des donn�es temps-r�el vers le workspace Matlab 
identif_data = mlib('FetchData');


% plot results
%plot(identif_data');


%***************************************%
%D�marrage du processus d'identification%
%***************************************%

sortie=(identif_data)';
entree=zeros(5000,1);
don1=iddata(sortie,entree,T_s); % d�finition de la structure de donn�es � identifier

mod1=pem(don1,2,'ss','can','ts',0,'Tolerance',0.01);     % processus d'identification
figure(1)
compare(mod1,don1);       % comparaison de la courbe r�elle � la courbe identifi�e

%**********************************%
%Initialisation du filtre de Kalman%
%**********************************%

%Calcul des valeurs de K_x et K_v du mod�le identifi�
K_x=-mod1.A(2,1)*m
K_v=-mod1.A(2,2)*m

%ecriture du modele lineaire
s = tf('s');
mod_lin = 1/K_x/(1+ K_v/K_x*s + m/K_x*s^2);
Al = [0 1; -K_x/m -K_v/m];
Bl = [0; 1/m];
Cl = [1 0];
mod_etat_lin1 = ss(Al,Bl,Cl,0);

K_x_desc = mlib('GetTrcVar', 'Model Root/K_x/Gain');
mlib('Write', K_x_desc, 'Data', K_x);


% On est content !!!
