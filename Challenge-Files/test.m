cfg = mars.default_config();
cfg.overrides.task3.N_series = 851;  
cfg.overrides.task3.N_parallel = 11;    
cfg.overrides.task3.array_mpp_power_kw = 50; 

result = run_task3_parallel(cfg);