function result = run_task2_parallel(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

result = mars.run_task2(cfg);
end
