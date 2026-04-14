function resultsTbl = runShotSweep(trackFcn, presetNames, sampleDt, durations)
% runShotSweep
%
% trackFcn should be a function handle:
%   shot = trackFcn(presetName, duration, numSamples)
%
% Example:
%   resultsTbl = runShotSweep(@myTrackFcn, ["serve_default","serve_t"], 0.10, []);
%
% durations is optional containers.Map or struct of preset->duration

    if nargin < 2 || isempty(presetNames)
        error('presetNames required.');
    end
    if nargin < 3 || isempty(sampleDt)
        sampleDt = 0.10;
    end
    if nargin < 4
        durations = [];
    end

    rows = [];

    for i = 1:numel(presetNames)
        presetName = string(presetNames(i));
        preset = getShotPreset(presetName);

        if isempty(durations)
            if startsWith(lower(presetName), "serve")
                duration = 0.90;
                numSamples = 10;
            else
                duration = 0.80;
                numSamples = 9;
            end
        else
            duration = durations.(char(presetName));
            numSamples = max(2, round(duration / sampleDt) + 1);
        end

        shot = trackFcn(presetName, duration, numSamples);
        stats = computeShotErrorStats(shot);

        bounceE = NaN;
        if stats.bounceError.available
            bounceE = stats.bounceError.e;
        end

        row = table( ...
            string(preset.name), ...
            string(preset.type), ...
            stats.n, ...
            stats.meanDx, stats.meanDy, stats.meanDz, ...
            stats.meanE, stats.maxE, stats.rmseE, ...
            bounceE, ...
            'VariableNames', { ...
                'preset','type','n', ...
                'meanDx','meanDy','meanDz', ...
                'meanE','maxE','rmseE','bounceE'});

        if isempty(rows)
            rows = row;
        else
            rows = [rows; row]; %#ok<AGROW>
        end
    end

    resultsTbl = rows;
end