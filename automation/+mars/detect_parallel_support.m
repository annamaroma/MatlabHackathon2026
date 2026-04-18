function tf = detect_parallel_support(cfg)
if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

tf = false;
if ~cfg.parallel.prefer
    return;
end

try
    tf = license('test', 'Distrib_Computing_Toolbox') && exist('parsim', 'file') == 2;
catch
    tf = false;
end
end
