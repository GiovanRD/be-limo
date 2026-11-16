% SIMU_SUIVI  Suivi de la boucle de démonstration par pure pursuit (sans arrêts).
%   Trace la trajectoire, l'écart au chemin et la vitesse.
clear; close all; clc;

p = parametres_limo();
chemin = load('chemin_demo.txt');             % x, y [m]

% Départ au début du chemin (noeud 2), orienté comme le chemin
cap0 = atan2(chemin(2, 2) - chemin(1, 2), chemin(2, 1) - chemin(1, 1));
etat = [chemin(1, 1); chemin(1, 2); cap0];

indice = 1;
t = 0;
fini = 0;
T = []; X = []; Y = []; V = []; ECART = [];

while ~fini && t < 300
    [v, delta, indice, fini] = pure_pursuit(etat, chemin, indice, p);
    etat = modele_limo(etat, v, delta, p);
    t = t + p.dt;

    % écart au chemin = distance au point du chemin le plus proche
    ecart = min(hypot(chemin(:, 1) - etat(1), chemin(:, 2) - etat(2)));

    T(end+1) = t; X(end+1) = etat(1); Y(end+1) = etat(2);
    V(end+1) = v; ECART(end+1) = ecart;
end

% Résultats
longueur = sum(hypot(diff(chemin(:, 1)), diff(chemin(:, 2))));
fprintf('Chemin : %.1f m, parcouru en %.1f s (vitesse moyenne %.2f m/s)\n', longueur, t, longueur / t);
fprintf('Ecart au chemin : moyen %.1f cm, maximum %.1f cm\n', 100 * mean(ECART), 100 * max(ECART));

figure('Name', 'Suivi de trajectoire');
subplot(2, 2, [1 3]);
plot(chemin(:, 1), chemin(:, 2), 'b', 'LineWidth', 3); hold on;
plot(X, Y, 'r', 'LineWidth', 1);
plot(chemin(1, 1), chemin(1, 2), 'ko', 'MarkerFaceColor', 'k');
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
legend('chemin planifié', 'trajectoire du robot', 'départ');
title('Boucle de démonstration');

subplot(2, 2, 2);
plot(T, 100 * ECART); grid on;
xlabel('temps (s)'); ylabel('écart (cm)');
title('Ecart au chemin');

subplot(2, 2, 4);
plot(T, V); grid on;
xlabel('temps (s)'); ylabel('v (m/s)');
title('Vitesse');
