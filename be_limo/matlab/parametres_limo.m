function p = parametres_limo()
% PARAMETRES_LIMO  Paramètres du robot et du contrôleur (mêmes valeurs que config/be_limo.yaml)

% Robot
p.L = 0.24;              % empattement [m]
p.delta_max = 0.41;      % braquage maximal [rad] (23,6 deg)

% Simulation
p.dt = 0.05;             % pas de temps [s]

% Pure pursuit
p.v_nominale = 0.25;     % vitesse en ligne droite [m/s]
p.Ld_min = 0.25;         % distance de visée à l'arrêt [m]
p.k_Ld = 0.6;            % Ld = Ld_min + k_Ld * v
p.k_virage = 1.5;        % réduction de vitesse en virage : v = v_nominale / (1 + k_virage*|courbure|)
p.tolerance_but = 0.06;  % distance d'arrivée [m]
p.pas_chemin = 0.02;     % distance entre deux points du chemin [m]

% Arrêts (scénario)
p.obstacle_arret = 0.35;       % arrêt si obstacle plus près [m]
p.obstacle_ralenti = 0.80;     % ralentissement en dessous [m]
p.duree_stop = 2.0;            % arrêt au panneau stop [s]
end
