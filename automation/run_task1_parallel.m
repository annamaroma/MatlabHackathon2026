function result = run_task1_parallel(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
    cfg.task1.solar_kw_grid = 50:10:100;
    cfg.task1.init_soc_grid = 20:10:60;
end

result = mars.run_task1(cfg);
end
