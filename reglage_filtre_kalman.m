%reglage filtre kalman sur le capteur 300 µm precitec
wf1 = 1e-16;
R = (3e-8)^2;

Q1 = calcul_Q(K_x,K_v,m,T_s,wf1);

% conversion sur le modele d'état étendu intégrant la Fext dans
% le vecteur d'etat
Ae = [mod_etat_lin1.A mod_etat_lin1.B; 0 0 0];
Be = [mod_etat_lin1.B ; 0];
Ce = [1 0 0];
mod_lin_etendu1 = ss(Ae,Be,Ce,0);
mod_etendu_1 = c2d(mod_lin_etendu1,T_s);
A1 = mod_etendu_1.a
B1 = mod_etendu_1.b
C1 = mod_etendu_1.c

% envoi des valeurs sur le dspace
DS_A1 = mlib('GetTrcVar', 'Model Root/A/Value');
mlib('Write', DS_A1, 'Data', A1);
DS_B1 = mlib('GetTrcVar', 'Model Root/B/Value');
mlib('Write', DS_B1, 'Data', B1);
DS_Q1 = mlib('GetTrcVar', 'Model Root/Q/Value');
mlib('Write', DS_Q1, 'Data', Q1);
DS_R1 = mlib('GetTrcVar', 'Model Root/R/Value');
mlib('Write', DS_R1, 'Data', R);

