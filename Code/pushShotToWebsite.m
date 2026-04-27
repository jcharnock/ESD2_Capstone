function response = pushShotToWebsite(payload, WEB)
%PUSHSHOTTOWEBSITE Sends the built shot payload to SQL through sql_link.m.
%
% Even though this file is still named pushShotToWebsite, we are using it
% as the final upload bridge. The GUI does not need to change.

    % If uploads are disabled, return early.
    if ~isfield(WEB, 'enabled') || ~WEB.enabled
        response = struct('ok', false, 'message', 'WEB.enabled is false');
        return;
    end

    % Build the image path that plotSingleShotCourt(...) saved.
    imageDir = fullfile(pwd, 'images');

    shotName = "shot";
    if isfield(payload, 'shot_name') && strlength(string(payload.shot_name)) > 0
        shotName = string(payload.shot_name);
    end

    shotNo = 1;
    if isfield(payload, 'shot_no') && ~isempty(payload.shot_no)
        shotNo = payload.shot_no;
    end

    safeShotName = regexprep(char(shotName), '[^\w\-]', '_');
    image_source = fullfile(imageDir, sprintf('%s_shot_no_%d.png', safeShotName, shotNo));

    if ~isfile(image_source)
        warning('SQL upload skipped: image file not found: %s', image_source);
        response = struct('ok', false, 'message', 'Image file not found');
        return;
    end

    % Convert the large web payload into a smaller SQL-friendly struct.
    sqlStruct = buildSQLStructFromPayload(payload);

    % Send to MySQL using sql_link.m.
    sql_link(sqlStruct, image_source);

    response = struct();
    response.ok = true;
    response.message = "Uploaded to SQL through sql_link.m";
end


function sqlStruct = buildSQLStructFromPayload(payload)
%BUILDSQLSTRUCTFROMPAYLOAD Converts payload into SQL-friendly fields.
%
% Keep this relatively flat. SQL tables do not like deeply nested MATLAB
% structs, arrays, or long matrices unless you intentionally encode them.

    sqlStruct = struct();

    sqlStruct.uploaded_utc = string(getFieldOrDefault(payload, 'uploaded_utc', ""));
    sqlStruct.source = string(getFieldOrDefault(payload, 'source', "matlab_gui"));
    sqlStruct.device_name = string(getFieldOrDefault(payload, 'device_name', "MATLAB_GUI"));

    sqlStruct.event_type = string(getFieldOrDefault(payload, 'event_type', "unknown"));
    sqlStruct.shot_name = string(getFieldOrDefault(payload, 'shot_name', ""));
    sqlStruct.decision = string(getFieldOrDefault(payload, 'decision', "UNKNOWN"));

    sqlStruct.num_samples = getNumericFieldOrNaN(payload, 'num_samples');
    sqlStruct.num_valid_samples = getNumericFieldOrNaN(payload, 'num_valid_samples');
    sqlStruct.valid_fraction = getNumericFieldOrNaN(payload, 'valid_fraction');

    % Bounce location
    bounceXYZ = getVector3(payload, 'bounce_xyz');
    sqlStruct.bounce_x = bounceXYZ(1);
    sqlStruct.bounce_y = bounceXYZ(2);
    sqlStruct.bounce_z = bounceXYZ(3);

    % Contact location/time
    contactXYZ = getVector3(payload, 'contact_xyz');
    sqlStruct.contact_t = getNumericFieldOrNaN(payload, 'contact_t');
    sqlStruct.contact_x = contactXYZ(1);
    sqlStruct.contact_y = contactXYZ(2);
    sqlStruct.contact_z = contactXYZ(3);

    % Net location/time
    netXYZ = getVector3(payload, 'net_xyz');
    sqlStruct.net_t = getNumericFieldOrNaN(payload, 'net_t');
    sqlStruct.net_x = netXYZ(1);
    sqlStruct.net_y = netXYZ(2);
    sqlStruct.net_z = netXYZ(3);

    % Restitution / error
    sqlStruct.restitution = getNumericFieldOrNaN(payload, 'restitution');
    sqlStruct.mean_error_mag = getNumericFieldOrNaN(payload, 'mean_error_mag');

    rmseXYZ = getVector3(payload, 'rmse_xyz');
    sqlStruct.rmse_x = rmseXYZ(1);
    sqlStruct.rmse_y = rmseXYZ(2);
    sqlStruct.rmse_z = rmseXYZ(3);

    maeXYZ = getVector3(payload, 'mean_abs_error_xyz');
    sqlStruct.mae_x = maeXYZ(1);
    sqlStruct.mae_y = maeXYZ(2);
    sqlStruct.mae_z = maeXYZ(3);

    % Store selected pair as the final selected pair.
    if isfield(payload, 'selected_pair') && ~isempty(payload.selected_pair)
        sp = payload.selected_pair;
        sp = sp(isfinite(sp));
        if ~isempty(sp)
            sqlStruct.selected_pair_final = sp(end);
        else
            sqlStruct.selected_pair_final = NaN;
        end
    else
        sqlStruct.selected_pair_final = NaN;
    end

    % Optional: store full trajectories as JSON strings.
    % This avoids trying to place Nx3 matrices directly into SQL columns.
    sqlStruct.t_json = jsonencode(getFieldOrDefault(payload, 't', []));
    sqlStruct.xyz_true_json = jsonencode(getFieldOrDefault(payload, 'xyz_true', []));
    sqlStruct.xyz_measured_json = jsonencode(getFieldOrDefault(payload, 'xyz_measured', []));
    sqlStruct.valid_json = jsonencode(getFieldOrDefault(payload, 'valid', []));
end


function out = getFieldOrDefault(S, fieldName, defaultVal)
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = S.(fieldName);
    else
        out = defaultVal;
    end
end


function out = getNumericFieldOrNaN(S, fieldName)
    if isfield(S, fieldName) && ~isempty(S.(fieldName)) && isnumeric(S.(fieldName))
        out = S.(fieldName);
        if ~isscalar(out)
            out = out(1);
        end
    else
        out = NaN;
    end
end


function v = getVector3(S, fieldName)
    if isfield(S, fieldName) && ~isempty(S.(fieldName)) && numel(S.(fieldName)) >= 3
        temp = S.(fieldName);
        v = reshape(temp(1:3), 1, 3);
    else
        v = [NaN NaN NaN];
    end
end