function pintoParedesMapa(paredes_mapa)

    for i = 1:size(paredes_mapa, 1)
        x_pared = [paredes_mapa(i,1), paredes_mapa(i,3)];
        y_pared = [paredes_mapa(i,2), paredes_mapa(i,4)];
        plot(x_pared, y_pared, '-k', 'LineWidth', 2);
    end
end