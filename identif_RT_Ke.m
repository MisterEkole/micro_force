%*****************************************%
%Acquisition des données temps-réel Dspace%
%*****************************************%

% on verifie que l'on est en boucle ouverte
hdl_switch = mlib('GetTrcVar', 'Model Root/Capteur Levitation/ibob felec switch/Value');
bouclage = mlib('read', hdl_switch);
% et que valid u est à zero
hdl_valid = mlib('GetTrcVar', 'Model Root/valid u/Gain');
valid_u = mlib('read', hdl_valid);
if (bouclage == 1 && valid_u == 0) % on peut continuer sinon on fait rien
    % on met K_e=1
    hdl_K_e = mlib('GetTrcVar', 'Model Root/Capteur Levitation/K_e/Gain');
    mlib('write', hdl_K_e, 'Data', 1);

    % lecture courant bobine
    identif_courant = mlib('GetTrcVar', 'Model Root/Capteur Levitation/I_bob/Value');
    i_bob = mlib('read', identif_courant);

    % aquisition de la force estimee sur un cour instant
    hdl_hatF = mlib('GetTrcVar', 'Model Root/Kalman filter/Xhat');
    mlib('Set','TraceVars', hdl_hatF,'Delay', 0,'NumSamples',500);
    % Démarrage de l'acquisition
    mlib('StartCapture');
    % Attente de la fin de l'acquisition
    while mlib('CaptureState')~=0, end;
    % Transfert des données temps-réel vers le workspace Matlab 
    Xe = mlib('FetchData');
    hatF = mean(Xe(3,:));

    % dou on tire Ke en isu/N
    K_e = i_bob/hatF

    % mise à jour dans le dspace
    mlib('write', hdl_K_e, 'Data', K_e);
    
    % remise a zero de I_bob
    mlib('write', identif_courant, 'Data', 0);
else
    disp(' ');
    disp('ERREUR !');
    disp('passer en boucle ouverte et mettre "valid_u" a zero et recommencer')
    disp(' ');
end
