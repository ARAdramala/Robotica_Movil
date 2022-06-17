function plotCallback
    global state
    global measurements

    state = 4;
    figure
    polarplot(deg2rad(linspace(0, 180, length(measurements))), measurements,...
        LineWidth = 3)
    thetalim([0, 180])
    rl = rlim;
    if (max(rl) > 200)
        rlim([0, 200])
    end
    grid minor
    title("Distancia [mm]")

    terminateLidar
end