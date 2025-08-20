Sky and Wire UAV-BPL Simulation README

This README provides instructions for running the MATLAB simulation codes for the Sky and Wire framework, evaluating UAV-BPL data collection in dairy farming for two scenarios: Coverage-Driven (Scenario 1) and Priority-Driven (Scenario 2).
System Requirements

MATLAB: Version R2023a or later.
Toolboxes: Statistics and Machine Learning Toolbox (for pdist2 function).
Hardware: Standard desktop/laptop (e.g., 8 GB RAM, 2 GHz CPU).
OS: Windows, macOS, or Linux.

Files

coverage_updated.m: Simulates Scenario 1, aiming for comprehensive sensor coverage with redundancy for critical data.
priority_updated.m: Simulates Scenario 2, prioritizing high-priority data collection.

Setup

Install MATLAB: Ensure MATLAB R2022b+ are installed.
Download Codes: Place coverage_updated.m and priority_updated.m in a working directory (e.g., C:\SkyAndWire or ~/SkyAndWire).
Verify Dependencies: No external libraries are required beyond MATLAB’s built-in functions.

Running the Simulations

Open MATLAB: Launch MATLAB and navigate to the directory containing the codes.
Run Scenario 1:
Execute coverage_updated.m by typing run coverage_updated_stats.m in the MATLAB Command Window or clicking "Run".
The script simulates 10 Monte Carlo runs on a 200×200 m² farm with 50 sensors, 5 UAVs, and 10 BPL relays, producing metrics like coverage (98.2%), latency ((\lambda = 2.45 , \text{s})), and energy use (62.7%).


Run Scenario 2:
Execute priority_updated_stats.m similarly.
Outputs include priority coverage, response time, and energy use.


View Results: Each script generates a figure with six subplots (UAV paths, coverage/energy metrics, timeliness, etc.) and displays mean/standard deviation for key metrics.

Expected Outputs

Scenario 1: Plots showing UAV paths, coverage, redundancy, latency, energy use, relay visits, and per-UAV energy.
Scenario 2: Plots showing UAV paths (with priority-colored sensors), priority coverage, response time, relay visits, and cumulative coverage.
Console outputs display mean and standard deviation of metrics.

Parameter Customization

Farm Size: Modify farmSize = [200, 200] (in meters) to test larger farms (e.g., [400, 400]).
Number of Sensors/UAVs/Relays: Adjust N = 50, M = 5, numRelays = 10 for scalability (up to 1000 sensors, 50 UAVs, 100 relays).
Priority Threshold: Change theta = 0.7 to alter critical data classification.
Energy Rate: Adjust energyRate = 0.1 (% per 10 m) for different UAV models.
BPL Capacity: Modify C_BPL = 15 * ones(numRelays, 1) to simulate variable capacity (e.g., rand(numRelays, 1) * 20).

Notes

Reproducibility: The codes use rng(42 + run) for random seed variation across 10 runs. Change the seed for different results.
