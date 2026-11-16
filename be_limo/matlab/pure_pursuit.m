function [v, delta, indice, fini] = pure_pursuit(etat, chemin, indice, p)
% PURE_PURSUIT  Loi de commande de suivi de trajectoire.
%   etat   : [x; y; theta] du robot (essieu arrière)
%   chemin : N x 2, points du chemin à suivre
%   indice : point du chemin le plus proche à l'itération précédente
%   Sorties : vitesse v [m/s], braquage delta [rad], nouvel indice, fini = 1 à l'arrivée

N = size(chemin, 1);

% 1) Point le plus proche, cherché seulement vers l'avant
%    (le chemin se recoupe aux carrefours : on ne doit pas sauter sur une autre branche)
fin = min(N, indice + 60);
d = hypot(chemin(indice:fin, 1) - etat(1), chemin(indice:fin, 2) - etat(2));
[~, k] = min(d);
indice = indice + k - 1;

% 2) Arrivée ?
reste = (N - indice) * p.pas_chemin;          % distance restant à parcourir [m]
if reste < p.tolerance_but
    v = 0; delta = 0; fini = 1;
    return
end
fini = 0;

% 3) Point visé : premier point du chemin à la distance Ld
Ld = p.Ld_min + p.k_Ld * p.v_nominale;
cible = N;
for i = indice:N
    if hypot(chemin(i, 1) - etat(1), chemin(i, 2) - etat(2)) >= Ld
        cible = i;
        break
    end
end

% 4) Angle vers le point visé, puis braquage
alpha = atan2(chemin(cible, 2) - etat(2), chemin(cible, 1) - etat(1)) - etat(3);
alpha = atan2(sin(alpha), cos(alpha));        % entre -pi et pi
courbure = 2 * sin(alpha) / Ld;
delta = atan(p.L * courbure);
delta = max(-p.delta_max, min(p.delta_max, delta));

% 5) Vitesse : plus lente dans les virages et près de l'arrivée
v = p.v_nominale / (1 + p.k_virage * abs(courbure));
v = min(v, reste + 0.02);
end
