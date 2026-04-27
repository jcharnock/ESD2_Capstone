function payload = buildWebPayload(shot, shotCategory, WEB)
% Convert GUI shot struct into website-safe payload.
%
% Inputs:
%   shot         - Internal shot struct from the GUI.
%   shotCategory - High-level label such as "serve", "volley", or "cor".
%   WEB          - Website settings struct, used here mainly for metadata
%                  such as the device name.
%
% Output:
%   payload      - Struct containing only fields appropriate for backend
%                  upload. This should avoid GUI handles, timers, images,
%                  or other heavy/non-serializable MATLAB data.
%
% Payload contents:
%   - Metadata (source, schema version, upload timestamp)
%   - Shot identity and decision
%   - Time vector and validity mask
%   - True/measured/used XYZ trajectories
%   - Bounce/contact/net information
%   - Selected stereo pair history
%   - Restitution
%   - Error metrics and summary block

    % Default shot category if one was not provided.
    if nargin < 2 || isempty(shotCategory)
        shotCategory = "unknown";
    end

    % Default WEB struct if omitted.
    if nargin < 3
        WEB = struct();
    end

    % Start with an empty payload.
    payload = struct();

    % ---------------- Metadata ----------------
    % Version number for your backend/frontend to track payload format.
    payload.schema_version = 1;

    % Where this payload came from.
    payload.source = "matlab_gui";

    % Device/source name. Helpful if multiple GUIs or machines upload.
    payload.device_name = getFieldOrDefault(WEB, 'deviceName', "MATLAB_GUI");

    % Event type supplied by caller: serve / volley / cor / etc.
    payload.event_type = char(string(shotCategory));
    payload.shot_no = getScalarNaN(shot, 'shotNo');
    
    % UTC upload timestamp in ISO-like format for backend storage.
    payload.uploaded_utc = char(datetime('now', ...
        'TimeZone', 'UTC', ...
        'Format', 'yyyy-MM-dd''T''HH:mm:ss''Z'''));

    % High-level shot name and decision.
    payload.shot_name = getStringField(shot, 'name', "");
    payload.decision = getStringField(shot, 'decision', "UNKNOWN");

    % ---------------- Time / trajectory data ----------------
    % Row vector of times.
    payload.t = getRowVectorField(shot, 't');

    % Validity mask for measured samples.
    payload.valid = getLogicalRowVectorField(shot, 'valid');

    % Ground-truth trajectory.
    payload.xyz_true = getMatrixField(shot, 'xyzTrue', 3);

    % Measured trajectory from tracker/backend.
    payload.xyz_measured = getMatrixField(shot, 'xyzMeas', 3);

    % XYZ actually used by GUI logic (currently often same as xyzTrue).
    payload.xyz_used = getMatrixField(shot, 'xyz', 3);

    % ---------------- Shot markers ----------------
    % Bounce sample index, if present.
    payload.bounce_idx = getScalarOrEmpty(shot, 'bounceIdx');

    % Bounce location in XYZ.
    payload.bounce_xyz = getVector3Field(shot, 'bounceXYZ');

    % Court-contact time and position.
    payload.contact_t = getScalarNaN(shot, 'contactT');
    payload.contact_xyz = getVector3Field(shot, 'contactXYZ');

    % Net-crossing time and position.
    payload.net_t = getScalarNaN(shot, 'netT');
    payload.net_xyz = getVector3Field(shot, 'netXYZ');

    % Which stereo pair was selected at each sample.
    payload.selected_pair = getRowVectorField(shot, 'selectedPair');

    % Restitution value, if this shot includes COR analysis.
    payload.restitution = getScalarNaN(shot, 'restitution');

    % ---------------- Summary counts ----------------
    % Number of samples in the shot.
    payload.num_samples = numel(payload.t);

    % Number of samples marked valid.
    payload.num_valid_samples = sum(payload.valid);

    % Fraction of valid samples.
    payload.valid_fraction = localSafeDivide(payload.num_valid_samples, max(payload.num_samples,1));

    % ---------------- Error metrics ----------------
    % Compute per-sample XYZ error and aggregate metrics.
    [payload.error_xyz, payload.rmse_xyz, payload.mean_abs_error_xyz, payload.mean_error_mag] = ...
        computeShotErrors(payload.xyz_true, payload.xyz_measured, payload.valid);

    % ---------------- Small summary block ----------------
    % Useful for frontend cards or latest-shot widgets.
    payload.summary = struct();
    payload.summary.decision = payload.decision;
    payload.summary.bounce_xyz = payload.bounce_xyz;
    payload.summary.restitution = payload.restitution;
    payload.summary.num_valid_samples = payload.num_valid_samples;
    payload.summary.mean_error_mag = payload.mean_error_mag;
end

% =====================================================================
% Helper functions:
% =====================================================================

function out = getStringField(S, fieldName, defaultVal)
% Return a field as char/string-safe text.
% If missing or empty, return the provided default.
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = char(string(S.(fieldName)));
    else
        out = char(string(defaultVal));
    end
end

function out = getRowVectorField(S, fieldName)
% Return a field as a row vector.
% Useful for time vectors and pair-index history.
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = S.(fieldName)(:).';
    else
        out = [];
    end
end

function out = getLogicalRowVectorField(S, fieldName)
% Return a field as a logical row vector.
% Used for validity masks.
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = logical(S.(fieldName)(:).');
    else
        out = false(1,0);
    end
end

function out = getMatrixField(S, fieldName, nCols)
% Return a numeric matrix only if it has the expected column count.
% For XYZ trajectories, nCols should be 3.
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = S.(fieldName);
        if size(out,2) ~= nCols
            out = [];
        end
    else
        out = [];
    end
end

function out = getVector3Field(S, fieldName)
% Return a field as a 1x3 vector.
% If missing or malformed, return [NaN NaN NaN].
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        v = S.(fieldName);
        if numel(v) == 3
            out = reshape(v, 1, 3);
        else
            out = [NaN NaN NaN];
        end
    else
        out = [NaN NaN NaN];
    end
end

function out = getScalarNaN(S, fieldName)
% Return a scalar field, or NaN if missing.
    if isfield(S, fieldName) && ~isempty(S.(fieldName)) && isscalar(S.(fieldName))
        out = S.(fieldName);
    else
        out = NaN;
    end
end

function out = getScalarOrEmpty(S, fieldName)
% Return a scalar field, or [] if missing.
    if isfield(S, fieldName) && ~isempty(S.(fieldName)) && isscalar(S.(fieldName))
        out = S.(fieldName);
    else
        out = [];
    end
end

function out = getFieldOrDefault(S, fieldName, defaultVal)
% Return a field if it exists; otherwise return default.
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = S.(fieldName);
    else
        out = defaultVal;
    end
end

function q = localSafeDivide(a,b)
% Divide safely, returning NaN on divide-by-zero.
    if b == 0
        q = NaN;
    else
        q = a / b;
    end
end

function [errXYZ, rmseXYZ, maeXYZ, meanErrMag] = computeShotErrors(xyzTrue, xyzMeas, validMask)
% Compute per-sample XYZ error and summary metrics.
%
% Outputs:
%   errXYZ      - xyzMeas - xyzTrue for all samples
%   rmseXYZ     - RMSE in X/Y/Z over valid finite samples
%   maeXYZ      - Mean absolute error in X/Y/Z
%   meanErrMag  - Mean Euclidean error magnitude

    % Defaults if error computation is not possible.
    errXYZ = [];
    rmseXYZ = [NaN NaN NaN];
    maeXYZ = [NaN NaN NaN];
    meanErrMag = NaN;

    % Need both true and measured trajectories.
    if isempty(xyzTrue) || isempty(xyzMeas)
        return;
    end

    % Must be Nx3 matrices of same size.
    if size(xyzTrue,1) ~= size(xyzMeas,1) || size(xyzTrue,2) ~= 3 || size(xyzMeas,2) ~= 3
        return;
    end

    % If no valid mask is supplied, assume everything is valid.
    if isempty(validMask)
        validMask = true(size(xyzTrue,1),1);
    else
        validMask = logical(validMask(:));
    end

    % Sample-by-sample XYZ error.
    errXYZ = xyzMeas - xyzTrue;

    % Keep only samples that are valid and finite.
    keep = validMask & all(isfinite(errXYZ),2);
    if ~any(keep)
        return;
    end

    E = errXYZ(keep,:);

    % RMSE per coordinate.
    rmseXYZ = sqrt(mean(E.^2, 1, 'omitnan'));

    % Mean absolute error per coordinate.
    maeXYZ = mean(abs(E), 1, 'omitnan');

    % Mean Euclidean magnitude of XYZ error.
    meanErrMag = mean(vecnorm(E, 2, 2), 'omitnan');
end