% MATLAB Simulation for Scenario 2: Priority-Driven Exchange with Statistical Measures
clear all; close all; clc;

% Simulation Parameters
numRuns = 10;               % Number of simulation runs
farmSize = [200, 200];      % Farm size (m²)
N = 50;                     % Number of sensors (scalable up to 1000)
M = 5;                      % Number of UAVs (scalable up to 50)
numRelays = 10;             % Number of BPL relay points (scalable up to 100)
uavRange = 100;             % UAV communication range (m)
E_min = 20;                 % Minimum energy (%)
theta = 0.7;                % Priority threshold
energyRate = 0.1;           % Energy consumption per 10 m (%)
C_min = 10;                 % Minimum BPL capacity (Mbps)
maxMoves = 20;              % Max moves per UAV

% Initialize arrays for statistical measures
tau_runs = zeros(numRuns, 1);
E_total_runs = zeros(numRuns, 1);
priority_coverage_runs = zeros(numRuns, 1);

for run = 1:numRuns
    % Randomly place sensors, UAVs, and BPL relays
    rng(42 + run);
    sensors = rand(N, 2) .* farmSize;
    uavPos = rand(M, 2) .* farmSize;
    relayPos = rand(numRelays, 2) .* farmSize;

    % Assign sensors to nearest relay
    distToRelay = pdist2(sensors, relayPos);
    [~, relayAssignment] = min(distToRelay, [], 2);

    % Generate data packets and energy levels
    p = rand(N, 1);
    T_gen = zeros(N, 1);
    E_j = 100 * ones(M, 1);
    C_BPL = 15 * ones(numRelays, 1);

    % Precompute high-priority data per relay
    priorityPerRelay = false(numRelays, 1);
    for k = 1:numRelays
        assignedSources = find(relayAssignment == k);
        priorityPerRelay(k) = any(p(assignedSources) >= theta);
    end

    % Simulate UAV priority-driven exchange
    uavPaths = cell(M, 1);
    T_collect = zeros(N, 1);
    visitedRelays = false(M, numRelays);
    relayVisits = zeros(numRelays, 1);
    storedData = cell(M, 1);

    for j = 1:M
        uavPaths{j} = uavPos(j, :);
        moveCount = 0;
        while E_j(j) >= E_min && moveCount < maxMoves
            distToRelays = pdist2(uavPaths{j}(end, :), relayPos);
            inRange = (distToRelays <= uavRange)';
            availableRelays = inRange & priorityPerRelay & ~visitedRelays(j, :)';
            if ~any(availableRelays)
                break;
            end
            relayDistances = distToRelays' .* availableRelays + 1e6 * ~availableRelays;
            [dist, nearestRelay] = min(relayDistances);
            if C_BPL(nearestRelay) >= C_min
                uavPaths{j} = [uavPaths{j}; relayPos(nearestRelay, :)];
                visitedRelays(j, nearestRelay) = true;
                relayVisits(nearestRelay) = relayVisits(nearestRelay) + 1;
                E_j(j) = E_j(j) - (dist / 10 * energyRate);
                moveCount = moveCount + 1;
                assignedSources = find(relayAssignment == nearestRelay);
                highPriority = assignedSources(p(assignedSources) >= theta);
                T_collect(highPriority) = dist / 10;
            else
                assignedSources = find(relayAssignment == nearestRelay);
                highPriority = assignedSources(p(assignedSources) >= theta);
                storedData{j} = [storedData{j}; highPriority'];
            end
        end
    end

    % Reassign uncollected high-priority data
    highPriorityIdx = find(p >= theta & T_collect == 0);
    for i = highPriorityIdx'
        relayIdx = relayAssignment(i);
        if ~any(visitedRelays(:, relayIdx))
            distToRelay = pdist2(uavPos, relayPos(relayIdx, :));
            [dist, availableUAV] = min(distToRelay .* (E_j >= E_min) + 1e6 * (E_j < E_min));
            if E_j(availableUAV) >= E_min && dist <= uavRange
                if C_BPL(relayIdx) >= C_min
                    uavPaths{availableUAV} = [uavPaths{availableUAV}; relayPos(relayIdx, :)];
                    relayVisits(relayIdx) = relayVisits(relayIdx) + 1;
                    E_j(availableUAV) = E_j(availableUAV) - (dist / 10 * energyRate);
                    T_collect(i) = dist / 10;
                else
                    storedData{availableUAV} = [storedData{availableUAV}; i];
                end
            end
        end
    end

    % Retry stored high-priority data
    for j = 1:M
        if ~isempty(storedData{j}) && E_j(j) >= E_min
            for i = storedData{j}
                relayIdx = relayAssignment(i);
                if C_BPL(relayIdx) >= C_min
                    dist = pdist2(uavPaths{j}(end, :), relayPos(relayIdx, :));
                    if dist <= uavRange
                        uavPaths{j} = [uavPaths{j}; relayPos(relayIdx, :)];
                        relayVisits(relayIdx) = relayVisits(relayIdx) + 1;
                        E_j(j) = E_j(j) - (dist / 10 * energyRate);
                        T_collect(i) = dist / 10;
                    end
                end
            end
        end
    end

    % Store metrics
    tau_runs(run) = mean(T_collect(p >= theta & T_collect > 0));
    E_total_runs(run) = sum(100 - E_j);
    priority_coverage_runs(run) = sum(T_collect(p >= theta) > 0) / sum(p >= theta) * 100;
end

% Compute statistical measures
tau_mean = mean(tau_runs); tau_std = std(tau_runs);
E_total_mean = mean(E_total_runs); E_total_std = std(E_total_runs);
priority_coverage_mean = mean(priority_coverage_runs); priority_coverage_std = std(priority_coverage_runs);

% Plots (using last run for visualization)
figure('Name', 'Scenario 2: Priority-Driven Exchange', 'Position', [100, 100, 1000, 800]);

subplot(3, 2, 1); hold on;
scatter(sensors(:, 1), sensors(:, 2), 50, p, 'filled', 'DisplayName', 'Sensors');
scatter(relayPos(:, 1), relayPos(:, 2), 100, 'r', 's', 'filled', 'DisplayName', 'BPL Relays');
for j = 1:M
    plot(uavPaths{j}(:, 1), uavPaths{j}(:, 2), 'k-', 'LineWidth', 1.5, 'DisplayName', ['UAV ' num2str(j)]);
end
title('UAV Paths and Priority Data (Color: Priority Score)'); xlabel('X (m)'); ylabel('Y (m)'); legend; grid on; colorbar;

subplot(3, 2, 2);
bar([priority_coverage_mean, E_total_mean]);
set(gca, 'XTickLabel', {'Priority Data Covered (%)', 'Total Energy Used (%)'});
title('Coverage and Energy Metrics (Mean ± SD)'); ylabel('Value'); ylim([0 100]);
text(0.05, 0.9, sprintf('Priority Coverage: %.1f ± %.1f%%\nEnergy: %.1f ± %.1f%%', ...
    priority_coverage_mean, priority_coverage_std, E_total_mean, E_total_std), 'Units', 'normalized');

subplot(3, 2, 3);
histogram(T_collect(p >= theta & T_collect > 0), 'Normalization', 'probability');
title('High-Priority Response Time'); xlabel('Time (s)'); ylabel('Probability');
text(0.05, 0.9, sprintf('Response Time (τ) = %.2f ± %.2f s', tau_mean, tau_std), 'Units', 'normalized');

subplot(3, 2, 4);
bar(sum(cellfun(@(p) sum(sqrt(sum(diff(p).^2, 2))), uavPaths)));
title('Total UAV Distance'); ylabel('Distance (m)');

subplot(3, 2, 5);
bar(relayVisits); title('Relay Visitation Frequency'); xlabel('Relay ID'); ylabel('Visits');
xticks(1:numRelays);

subplot(3, 2, 6);
plot(1:N, cumsum(T_collect > 0) / N * 100, 'b-', 'LineWidth', 2);
title('Cumulative Coverage Over Sources'); xlabel('Sensor Index'); ylabel('Coverage (%)');
ylim([0 100]);