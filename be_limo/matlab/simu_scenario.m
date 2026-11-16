% SIMU_SCENARIO  Boucle de démonstration avec les trois arrêts :
%   panneau stop (noeud 14), obstacle (voie du haut), feu tricolore (noeud 9).
%   Même machine à états que local_controller_node.py, en version simplifiée.
clear; close all; clc;

p = parametres_limo();
chemin = load('chemin_demo.txt');
N = size(chemin, 1);

% Position des événements le long du chemin (abscisse curviligne s [m])
abscisse = @(x, y) p.pas_chemin * (find(hypot(chemin(:, 1) - x, chemin(:, 2) - y) == ...
                    min(hypot(chemin(:, 1) - x, chemin(:, 2) - y)), 1) - 1);
s_stop     = abscisse(2.121, 1.261);      % ligne d'arrêt du stop (noeud 14)
s_obstacle = abscisse(-0.70, 2.795);      % obstacle sur la voie du haut
s_feu      = abscisse(-0.881, -0.871);    % ligne d'arrêt du feu (noeud 9)

% Feu : rouge 12 s, vert 12 s, etc.
feu_rouge = @(t) mod(t, 24) < 12;

cap0 = atan2(chemin(2, 2) - chemin(1, 2), chemin(2, 1) - chemin(1, 1));
etat = [chemin(1, 1); chemin(1, 2); cap0];
indice = 1; t = 0; fini = 0;
obstacle_present = 1; attente_obstacle = 0;
stop_fait = 0; fin_stop = -1;
T = []; X = []; Y = []; V = []; ETAT = {};

while ~fini && t < 400
    [v, delta, indice, fini] = pure_pursuit(etat, chemin, indice, p);
    s = (indice - 1) * p.pas_chemin;               % position du robot le long du chemin
    mode = 'EN_ROUTE';

    % --- Obstacle : ralentir, puis s'arrêter (distance mesurée depuis l'avant du robot)
    if obstacle_present
        d_obs = s_obstacle - s - 0.2;
        if d_obs > 0 && d_obs < p.obstacle_arret
            v = 0; mode = 'ARRET_OBSTACLE';
            attente_obstacle = attente_obstacle + p.dt;
            if attente_obstacle > 3
                obstacle_present = 0;               % on retire l'obstacle après 3 s
            end
        elseif d_obs > 0 && d_obs < p.obstacle_ralenti
            v = v * max(0.25, (d_obs - p.obstacle_arret) / (p.obstacle_ralenti - p.obstacle_arret));
            mode = 'RALENTI';
        end
    end

    % --- Feu : arrêt à la ligne si le feu est rouge
    d_feu = s_feu - s;
    if d_feu > 0 && d_feu < 0.10 && feu_rouge(t)
        v = 0; mode = 'ARRET_FEU';
    end

    % --- Stop : arrêt complet de 2 s, une seule fois
    d_stop = s_stop - s;
    if ~stop_fait && d_stop > 0 && d_stop < 0.10
        stop_fait = 1; fin_stop = t + p.duree_stop;
    end
    if t < fin_stop
        v = 0; mode = 'ARRET_STOP';
    end

    etat = modele_limo(etat, v, delta, p);
    t = t + p.dt;
    T(end+1) = t; X(end+1) = etat(1); Y(end+1) = etat(2); V(end+1) = v; ETAT{end+1} = mode;
end

% Résumé des arrêts
fprintf('Parcours terminé en %.1f s\n', t);
noms = {'ARRET_STOP', 'ARRET_OBSTACLE', 'ARRET_FEU'};
for i = 1:3
    k = strcmp(ETAT, noms{i});
    if any(k)
        fprintf('%-15s : %.1f s, à partir de t = %.1f s\n', noms{i}, sum(k) * p.dt, T(find(k, 1)));
    else
        fprintf('%-15s : pas d''arrêt\n', noms{i});
    end
end

figure('Name', 'Scénario');
subplot(1, 2, 1);
plot(chemin(:, 1), chemin(:, 2), 'b', 'LineWidth', 3); hold on;
plot(X, Y, 'r');
couleurs = {'m', [1 0.5 0], 'r'};
for i = 1:3
    k = find(strcmp(ETAT, noms{i}), 1);
    if ~isempty(k)
        plot(X(k), Y(k), 's', 'MarkerSize', 10, 'MarkerFaceColor', couleurs{i}, 'MarkerEdgeColor', 'k');
    end
end
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
title('Trajectoire et arrêts (carrés)');

subplot(1, 2, 2);
plot(T, V); grid on;
xlabel('temps (s)'); ylabel('v (m/s)');
title('Vitesse (0 = arrêt)');
