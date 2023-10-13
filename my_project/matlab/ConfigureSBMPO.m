

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 500000;
params.max_generations = 1000;
params.horizon_time = 0.10;
params.num_states = 3;
params.num_controls = 3;
params.grid_resolution = [0.10; 0.10; 0.10];
params.start_state = [1; 8; 0];
params.goal_state = [15; 8; 0];
params.branchout_factor = 24;
params.branchouts = [
    [1; 0; 0], [1; 1; 0], [0; 1; 0], [-1; 1; 0], ...
    [-1; 0; 0], [-1; -1; 0], [0; -1; 0], [1; -1; 0], ...
    [1; 0; 1], [1; 1; 1], [0; 1; 1], [-1; 1; 1], ...
    [-1; 0; 1], [-1; -1; 1], [0; -1; 1], [1; -1; 1], ...
    [1; 0; -1], [1; 1; -1], [0; 1; -1], [-1; 1; -1], ...
    [-1; 0; -1], [-1; -1; -1], [0; -1; -1], [1; -1; -1]
    ];


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);