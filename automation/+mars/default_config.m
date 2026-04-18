function cfg = default_config()
pkg_dir = fileparts(mfilename('fullpath'));
automation_dir = fileparts(pkg_dir);
repo_root = fileparts(automation_dir);
challenge_dir = fullfile(repo_root, 'Challenge-Files');

cfg = struct();
cfg.repo_root = repo_root;
cfg.challenge_dir = challenge_dir;
cfg.results_root = fullfile(repo_root, 'results');
cfg.overrides = struct();

cfg.parallel = struct();
cfg.parallel.prefer = true;
cfg.parallel.show_progress = false;
cfg.parallel.capture_errors = true;

cfg.simulation = struct();
cfg.simulation.time_tolerance = 1e-9;
cfg.simulation.task3_steady_state_start = 0.25;

cfg.task1 = struct();
cfg.task1.name = 'task1';
cfg.task1.model = 'Task1_idealSystem';
cfg.task1.model_file = fullfile(challenge_dir, 'Task1_idealSystem.slx');
cfg.task1.solar_kw_grid = 10:10:200;
cfg.task1.init_soc_grid = 0:10:100;
cfg.task1.solar_block = 'Solar';
cfg.task1.solar_parameter = 'power_rating';
cfg.task1.battery_block = 'Battery';
cfg.task1.battery_parameter = 'initSOC';
cfg.task1.soc_min = 0;
cfg.task1.soc_max = 100;
cfg.task1.logging = {
    'Living Quarters/PS-Simulink Converter', 'loadVoltage_V';
    'Battery/PS-Simulink Converter1', 'battPower_kW';
    'Battery/Gain', 'SOC'
};

cfg.task2 = struct();
cfg.task2.name = 'task2';
cfg.task2.model = 'Task2_MaximumPowerTransfer';
cfg.task2.model_file = fullfile(challenge_dir, 'Task2_MaximumPowerTransfer.slx');
cfg.task2.load_block = 'Load';
cfg.task2.load_parameter = 'R';
cfg.task2.load_variable = 'mars_task2_load_ohm';
cfg.task2.solar_block = 'Solar Cell';
cfg.task2.series_parameter = 'N_series';
cfg.task2.parallel_parameter = 'N_parallel';
cfg.task2.target_voltage_v = 500;
cfg.task2.steady_state_fraction = 0.20;
cfg.task2.coarse_resistance_ohm = 10:-0.5:1;
cfg.task2.fine_half_span_ohm = 0.5;
cfg.task2.fine_step_ohm = 0.05;
cfg.task2.logging = {
    'PS-Simulink Converter5', 'Vsolar';
    'PS-Simulink Converter6', 'Isolar';
    'PS-Simulink Converter7', 'Psolar'
};

cfg.task3 = struct();
cfg.task3.name = 'task3';
cfg.task3.model = 'Task3_MaximumPowerPointTracking';
cfg.task3.model_file = fullfile(challenge_dir, 'Task3_MaximumPowerPointTracking.slx');
cfg.task3.solar_block = 'Solar Array';
cfg.task3.series_parameter = 'N_series';
cfg.task3.parallel_parameter = 'N_parallel';
cfg.task3.mppt_block = 'MPPT';
cfg.task3.enable_time_s = 0.1;
cfg.task3.settling_time_s = 5.0;
cfg.task3.duty_tolerance = 1e-6;
cfg.task3.dinit_grid = 0.35:0.05:0.65;
cfg.task3.dmax_grid = [0.80, 0.85, 0.90, 0.95];
cfg.task3.dmin_grid = [0.05, 0.10, 0.15, 0.20];
cfg.task3.delta_d_grid = [0.001, 0.0025, 0.005, 0.01];
cfg.task3.rail_fraction_limit = 0.20;
cfg.task3.power_fraction_limit = 0.95;
cfg.task3.logging = {
    'PS-Simulink Converter2', 'Vs';
    'PS-Simulink Converter3', 'Is';
    'PS-Simulink Converter4', 'Ps_kW';
    'MPPT/Unit Delay', 'duty_cycle'
};

cfg.task4 = struct();
cfg.task4.name = 'task4';
cfg.task4.model = 'Task4_BatteryVoltageControl';
cfg.task4.model_file = fullfile(challenge_dir, 'Task4_BatteryVoltageControl.slx');
cfg.task4.setpoint_grid = 505:515;
cfg.task4.gain_grid = [1e-9, 2e-9, 5e-9, 1e-8, 2e-8, 5e-8, 1e-7];
cfg.task4.initial_condition_grid = 0.20:0.05:0.80;
cfg.task4.logging = {
    'PS-Simulink Converter', 'Vbatt';
    'PS-Simulink Converter1', 'loadVoltage_V';
    'PS-Simulink Converter2', 'SOC';
    'Discrete-Time Integrator', 'duty_cycle'
};

cfg.task5 = struct();
cfg.task5.name = 'task5';
cfg.task5.model = 'Task5_FullSystem';
cfg.task5.model_file = fullfile(challenge_dir, 'Task5_FullSystem.slx');
cfg.task5.top_k = 5;
cfg.task5.voltage_min = 500;
cfg.task5.voltage_max = 520;
cfg.task5.logging = {
    'Living Quarters/PS-Simulink Converter', 'loadVoltage_V';
    'Battery/PS-Simulink Converter1', 'Vbatt';
    'Battery/PS-Simulink Converter2', 'SOC';
    'Solar Array/PS-Simulink Converter5', 'Ps_kW';
    'Solar Array/MPPT/Unit Delay', 'duty_cycle'
};
end
