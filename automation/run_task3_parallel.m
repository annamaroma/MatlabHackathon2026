function result = run_task3_parallel(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

result = mars.run_task3(cfg);
end
