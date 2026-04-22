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

        % Keep the user-facing sweep names, but map them to the
        % internal preset names used by Project_GUI_Development_v1.m
        trackPresetName = mapSweepPresetName(presetName);

        % Summary label/type for the results table
        [displayPreset, displayType] = getDisplayInfo(presetName);

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

        % IMPORTANT: call the GUI tracker with the mapped legacy preset name
        shot = trackFcn(trackPresetName, duration, numSamples);
        stats = computeShotErrorStats(shot);

        bounceE = NaN;
        if stats.bounceError.available
            bounceE = stats.bounceError.e;
        end

        row = table( ...
            string(displayPreset), ...
            string(displayType), ...
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

function legacyName = mapSweepPresetName(presetName)
    switch lower(string(presetName))
        case "serve_default"
            legacyName = "serve_in_1";
        case "serve_t"
            legacyName = "serve_in_2";
        case "serve_wide"
            legacyName = "serve_out_1";
        case "serve_body"
            legacyName = "serve_out_2";
        case "volley_default"
            legacyName = "volley_in_1";
        case "volley_short"
            legacyName = "volley_in_2";
        case "volley_deep"
            legacyName = "volley_out_1";
        case "volley_cross"
            legacyName = "volley_out_2";
        otherwise
            error('Unknown sweep preset: %s', string(presetName));
    end
end

function [displayPreset, displayType] = getDisplayInfo(presetName)
    switch lower(string(presetName))
        case {"serve_default","serve_t","serve_wide","serve_body"}
            displayPreset = string(presetName);
            displayType = "serve";
        case {"volley_default","volley_short","volley_deep","volley_cross"}
            displayPreset = string(presetName);
            displayType = "volley";
        otherwise
            displayPreset = string(presetName);
            displayType = "unknown";
    end
end