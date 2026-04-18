function result = run_task4_parallel(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

result = mars.run_task4(cfg);
end
