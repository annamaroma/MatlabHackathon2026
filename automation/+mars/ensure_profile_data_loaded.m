function ensure_profile_data_loaded(cfg)
persistent loaded_root

if nargin < 1 || isempty(cfg)
    cfg = mars.default_config();
end

if isequal(loaded_root, cfg.challenge_dir)
    return;
end

data_file = fullfile(cfg.challenge_dir, 'profileData.mat');
data = load(data_file);
names = fieldnames(data);
for idx = 1:numel(names)
    assignin('base', names{idx}, data.(names{idx}));
end

addpath(cfg.challenge_dir);
loaded_root = cfg.challenge_dir;
end
