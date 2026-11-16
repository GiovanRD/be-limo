function etat = modele_limo(etat, v, delta, p)
% MODELE_LIMO  Modèle cinématique bicyclette de la LIMO (direction Ackermann).
%   etat = [x; y; theta] : position de l'essieu arrière [m] et cap [rad]
%   v : vitesse [m/s], delta : angle de braquage [rad]

delta = max(-p.delta_max, min(p.delta_max, delta));   % butée de direction

etat(1) = etat(1) + v * cos(etat(3)) * p.dt;
etat(2) = etat(2) + v * sin(etat(3)) * p.dt;
etat(3) = etat(3) + v / p.L * tan(delta) * p.dt;
end
