function [config] = getCompleteConfig()
% Gets all config values for storage

config.PDControl = get_config('PDControl');
config.WallFollower = get_config('WallFollower');
config.VelLimitations = get_config('VelLimitations');
config.Sensor = get_config('Sensor');
config.kinModelNoise = get_config('kinModelNoise');
config.mowerParameter = get_config('mowerParameter');
config.odometryModelNoise = get_config('odometryModelNoise');
config.system = get_config('system');
config.mapping = get_config('mapping');
config.globalLocalization = get_config('globalLocalization');
config.particleFilter = get_config('particleFilter');
config.coverageMap = get_config('coverageMap');
config.planning = get_config('planning');
end

