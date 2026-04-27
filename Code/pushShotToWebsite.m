function response = pushShotToWebsite(payload, WEB)
%PUSHSHOTTOWEBSITE Sends the built shot payload to SQL through sql_link.m.
%
% The GUI still calls uploadLastShotToWebsite(...), but this file routes
% the result into sql_link.m instead of sending it to a web API.

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

    % Convert payload into exact SQL/frontend field names.
    sqlStruct = buildSQLStructFromPayload(payload, image_source);

    % Send to MySQL using sql_link.m.
    sql_link(sqlStruct, image_source);

    response = struct();
    response.ok = true;
    response.message = "Uploaded to SQL through sql_link.m";
end


function sqlStruct = buildSQLStructFromPayload(payload, image_source)
%BUILDSQLSTRUCTFROMPAYLOAD Converts the MATLAB payload into the exact
%field names expected by the website/SQL side.

    sqlStruct = struct();

    % Match Python/backend variable names exactly.
    sqlStruct.timestamp = char(datetime('now', 'Format', 'HH:mm:ss'));

    % Use bounce/contact point as the final court coordinate.
    bounceXYZ = getVector3(payload, 'bounce_xyz');

    sqlStruct.x_coordinate = round(bounceXYZ(1), 3);
    sqlStruct.y_coordinate = round(bounceXYZ(2), 3);

    % IN / OUT / NET
    sqlStruct.ruling = char(string(getFieldOrDefault(payload, 'decision', 'UNKNOWN')));

    % Store relative image path in the same style as your Python example.
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