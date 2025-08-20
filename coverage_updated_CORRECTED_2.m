% MATLAB Simulation for Scenario 1: Coverage-Driven Exchange with Statistical Measures
clear all; close all; clc;

% Simulation Parameters
numRuns = 10;               % Number of simulation runs
farmSize = [200, 200];      % Farm size (m²)
N = 50;                     % Number of sensors (scalable up to 1000)
M = 5;                      % Number of UAVs (scalable up to 50)
numRelays = 10;             % Number of BPL relay points (scalable up to 100)
uavRange = 100;             % UAV communication range (m)
R = 2;                      % Redundancy factor for critical data
theta = 0.7;                % Priority threshold
energyRate = 0.1;           % Energy consumption per 10 m (%)
C_min = 10;                 % Minimum BPL capacity (Mbps)

% Initialize arrays for statistical measures
lambda_runs = zeros(numRuns, 1);
E_total_runs = zeros(numRuns, 1);
coverage_runs = zeros(numRuns, 1);
redundancy_runs = zeros(numRuns, 1);

for run = 1:numRuns
    % Randomly place sensors, UAVs, and BPL relays
    rng(42 + run); % Vary seed for each run
    sensors = rand(N, 2) .* farmSize;
    uavPos = rand(M, 2) .* farmSize;
    relayPos = rand(numRelays, 2) .* farmSize;

    % Assign sensors to nearest relay
    distToRelay = pdist2(sensors, relayPos);
    [~, relayAssignment] = min(distToRelay, [], 2);

    % Generate data packets and energy levels
    p = rand(N, 1);
    criticalData = p >= theta;
    T_gen = zeros(N, 1);
    E_j = 100 * ones(M, 1);
    C_BPL = 15 * ones(numRelays, 1); % Simulated BPL capacity

    % Simulate UAV coverage-driven exchange
    visitedRelays = false(numRelays, 1);
    uavPaths = cell(M, 1);
    T_collect = zeros(N, 1);
    relayVisits = zeros(numRelays, 1);
    storedData = cell(M, 1);

    for j = 1:M
        uavPaths{j} = uavPos(j, :);
        while any(~visitedRelays) && E_j(j) >= 20
            distToRelays = pdist2(uavPaths{j}(end, :), relayPos);
            inRange = distToRelays <= uavRange & ~visitedRelays';
            if ~any(inRange)
                break;
            end
            [dist, nearestRelay] = min(distToRelays .* inRange + 1e6 * ~inRange);
            if C_BPL(nearestRelay) >= C_min
                uavPaths{j} = [uavPaths{j}; relayPos(nearestRelay, :)];
                visitedRelays(nearestRelay) = true;
                relayVisits(nearestRelay) = relayVisits(nearestRelay) + 1;
                E_j(j) = E_j(j) - (dist / 10 * energyRate);
                assignedSources = find(relayAssignment == nearestRelay);
                T_collect(assignedSources) = dist / 10;
            else
                assignedSources = find(relayAssignment == nearestRelay);
                storedData{j} = [storedData{j}; assignedSources'];
            end
        end
    end

    % Retry stored data
    for j = 1:M
        if ~isempty(storedData{j}) && E_j(j) >= 20
            for i = storedData{j}
                relayIdx = relayAssignment(i);
                if C_BPL(relayIdx) >= C_min && ~visitedRelays(relayIdx)
                    dist = pdist2(uavPaths{j}(end, :), relayPos(relayIdx, :));
                    if dist <= uavRange
                        uavPaths{j} = [uavPaths{j}; relayPos(relayIdx, :)];
                        visitedRelays(relayIdx) = true;
                        relayVisits(relayIdx) = relayVisits(relayIdx) + 1;
                        E_j(j) = E_j(j) - (dist / 10 * energyRate);
                        T_collect(i) = dist / 10;
                    end
                end
            end
        end
    end

    % Ensure redundancy for critical data
    for i = find(criticalData)'
        relayIdx = relayAssignment(i);
        if relayVisits(relayIdx) < R
            distToRelay = pdist2(uavPos, relayPos(relayIdx, :));
            [dist, extraUAV] = min(distToRelay .* (E_j >= 20) + 1e6 * (E_j < 20));
            if E_j(extraUAV) >= 20 && dist <= uavRange
                uavPaths{extraUAV} = [uavPaths{extraUAV}; relayPos(relayIdx, :)];
                relayVisits(relayIdx) = relayVisits(relayIdx) + 1;
                E_j(extraUAV) = E_j(extraUAV) - (dist / 10 * energyRate);
                T_collect(i) = max(T_collect(i), dist / 10);
            end
        end
    end

    % Store metrics
    lambda_runs(run) = mean(T_collect(T_collect > 0));
    E_total_runs(run) = sum(100 - E_j);
    coverage_runs(run) = sum(T_collect > 0) / N * 100;
    redundancy_runs(run) = sum(relayVisits(relayAssignment(criticalData)) >= R) / sum(criticalData) * 100;
end

% Compute statistical measures
lambda_mean = mean(lambda_runs); lambda_std = std(lambda_runs);
E_total_mean = mean(E_total_runs); E_total_std = std(E_total_runs);
coverage_mean = mean(coverage_runs); coverage_std = std(coverage_runs);
redundancy_mean = mean(redundancy_runs); redundancy_std = std(redundancy_runs);

% Plots (using last run for visualization)
figure('Name', 'Scenario 1: Coverage-Driven Exchange', 'Position', [100, 100, 1000, 800]);

subplot(3, 2, 1); hold on;
scatter(sensors(:, 1), sensors(:, 2), 50, 'b', 'filled', 'DisplayName', 'Sensors');
scatter(relayPos(:, 1), relayPos(:, 2), 100, 'r', 's', 'filled', 'DisplayName', 'BPL Relays');
for j = 1:M
    plot(uavPaths{j}(:, 1), uavPaths{j}(:, 2), 'k-', 'LineWidth', 1.5, 'DisplayName', ['UAV ' num2str(j)]);
end
title('UAV Paths'); xlabel('X (m)'); ylabel('Y (m)'); legend; grid on;

subplot(3, 2, 2);
bar([coverage_mean, redundancy_mean]);
set(gca, 'XTickLabel', {'All Data Covered (%)', 'Critical Data Redundancy (%)'});
title(['Coverage Metrics (Mean ± SD)']); ylabel('Percentage'); ylim([0 100]);
text(0.05, 0.9, sprintf('Coverage: %.1f ± %.1f%%\nRedundancy: %.1f ± %.1f%%', ...
    coverage_mean, coverage_std, redundancy_mean, redundancy_std), 'Units', 'normalized');

subplot(3, 2, 3);
histogram(T_collect, 'Normalization', 'probability');
title('Data Collection Timeliness'); xlabel('Time (s)'); ylabel('Probability');
text(0.05, 0.9, sprintf('Average Latency (λ) = %.2f ± %.2f s', lambda_mean, lambda_std), 'Units', 'normalized');

subplot(3, 2, 4);
bar(E_total_mean);
title('Total Energy Used'); ylabel('Energy (%)');
text(0.05, 0.9, sprintf('E_total = %.1f ± %.1f%%', E_total_mean, E_total_std), 'Units', 'normalized');

subplot(3, 2, 5);
bar(relayVisits); title('Relay Visitation Frequency'); xlabel('Relay ID'); ylabel('Visits');
xticks(1:numRelays);

subplot(3, 2, 6);
bar(100 - E_j); title('Energy Consumption per UAV'); xlabel('UAV ID'); ylabel('Energy Used (%)');
xticks(1:M);