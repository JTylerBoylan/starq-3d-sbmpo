

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 500000;
params.max_generations = 1000;
params.horizon_time = 2.0;
params.num_states = 4;
params.num_controls = 4;
params.grid_resolution = [0.05; 0.05; 0.05; 1.0];
params.start_state = [0.25; 2.5; 0; 0];
params.goal_state = [4.75; 2.5; 0; 0];
params.branchout_factor = 0;
params.branchouts = [];


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);