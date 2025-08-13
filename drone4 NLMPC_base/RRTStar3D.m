function [x,y,z] = RRTStar3D(start, goal, maxIterations, stepSize)
    mapData = load("dMapCityBlock.mat");
    omap = mapData.omap;
    omap.FreeThreshold = 0.5;

    inflate(omap,1)
    ss = stateSpaceSE3([0 220;0 220;0 100;inf inf;inf inf;inf inf;inf inf]);
    sv = validatorOccupancyMap3D(ss, ...
         Map = omap, ...
         ValidationDistance = 0.1);
    % Membuat planner RRTStar
    planner = plannerRRTStar(ss, sv);

    % Atur properti planner
    planner.MaxConnectionDistance = stepSize;
    planner.MaxIterations = maxIterations;
    planner.GoalReachedFcn = @(~, s, g)(norm(s(1:3) - g(1:3)) < 1);
    planner.GoalBias = 0.2;
    rng(1,"twister");
    [pthObj,solnInfo] = plan(planner,start,goal);
    x = pthObj.States(:,1);
    y = pthObj.States(:,2);
    z = pthObj.States(:,3);

    show(omap)
    axis equal
    view([-10 55])
    hold on
    % Start state
    scatter3(start(1,1),start(1,2),start(1,3),"g","filled")
    % Goal state
    scatter3(goal(1,1),goal(1,2),goal(1,3),"r","filled")
    % Path
    plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3), ...
          "r-",LineWidth=2)
end
