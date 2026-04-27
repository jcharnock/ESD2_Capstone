function response = pushShotToWebsite(payload, WEB)
%PUSHSHOTTOWEBSITE Sends the shot payload to SQL through sql_link.m.

    if ~isfield(WEB, 'enabled') || ~WEB.enabled
        response = struct('ok', false, 'message', 'WEB.enabled is false');
        return;
    end

    % Find saved court image
    imageDir = fullfile(pwd, 'images');

    shotName = "shot";
    if isfield(payload, 'shot_name') && strlength(string(payload.shot_name)) > 0
        shotName = string(payload.shot_name);
    end

    shotNo = 1;
    if isfield(payload, 'shot_no') && ~isempty(payload.shot_no) && isfinite(payload.shot_no)
        shotNo = payload.shot_no;
    end

    safeShotName = regexprep(char(shotName), '[^\w\-]', '_');
    image_source = fullfile(imageDir, sprintf('%s_shot_no_%d.png', safeShotName, shotNo));

    if ~isfile(image_source)
        warning('SQL upload skipped: image file not found: %s', image_source);
        response = struct('ok', false, 'message', 'Image file not found');
        return;
    end

    % Build SQL row using exact frontend/database variable names
    sqlStruct = buildSQLStructFromPayload(payload, image_source);

    % Push to SQL
    sql_link(sqlStruct, image_source);

    response = struct('ok', true, 'message', 'Uploaded to SQL');
end


function sqlStruct = buildSQLStructFromPayload(payload, image_source)
%BUILDSQLSTRUCTFROMPAYLOAD Converts payload into exact SQL/frontend fields.

    sqlStruct = struct();

    sqlStruct.timestamp = char(datetime('now', 'Format', 'HH:mm:ss'));

    bounceXYZ = getVector3(payload, 'bounce_xyz');

    sqlStruct.x_coordinate = round(bounceXYZ(1), 3);
    sqlStruct.y_coordinate = round(bounceXYZ(2), 3);

    sqlStruct.ruling = char(string(getFieldOrDefault(payload, 'decision', 'UNKNOWN')));

    [~, imgName, imgExt] = fileparts(image_source);
    sqlStruct.court_image = ['../images/' imgName imgExt];
end


function out = getFieldOrDefault(S, fieldName, defaultVal)
    if isfield(S, fieldName) && ~isempty(S.(fieldName))
        out = S.(fieldName);
    else
        out = defaultVal;
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