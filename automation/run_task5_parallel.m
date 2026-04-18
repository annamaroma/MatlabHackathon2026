function result = run_task5_parallel(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

result = mars.run_task5(cfg);
end
