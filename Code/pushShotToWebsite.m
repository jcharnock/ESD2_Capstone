function response = pushShotToWebsite(payload, WEB)
% Send payload to backend API as JSON.
%
% Inputs:
%   payload - Struct already prepared for website upload.
%   WEB     - Website/API settings struct.
%
% Output:
%   response - Parsed response returned by the backend via webwrite.
%
% Notes:
%   - This function assumes the backend accepts JSON POST requests.
%   - If WEB.apiKey is non-empty, it is sent as an X-API-Key header.
%   - If WEB.enabled is false, no upload is attempted.

    % If uploads are disabled, return early instead of making a web request.
    if ~isfield(WEB, 'enabled') || ~WEB.enabled
        response = struct('ok', false, 'message', 'WEB.enabled is false');
        return;
    end

    % Make sure a target API URL exists.
    if ~isfield(WEB, 'apiUrl') || strlength(string(WEB.apiUrl)) == 0
        error('WEB.apiUrl is empty.');
    end

    % Default: no custom headers.
    headerFields = {};

    % Add API key header only if provided.
    if isfield(WEB, 'apiKey') && strlength(string(WEB.apiKey)) > 0
        headerFields = {'X-API-Key', char(string(WEB.apiKey))};
    end

    % Default timeout, overridden if WEB.timeoutSec exists.
    timeoutSec = 15;
    if isfield(WEB, 'timeoutSec') && ~isempty(WEB.timeoutSec)
        timeoutSec = WEB.timeoutSec;
    end

    % Configure the HTTP request for JSON upload.
    opts = weboptions( ...
        'MediaType', 'application/json', ...
        'Timeout', timeoutSec, ...
        'HeaderFields', headerFields);

    % Send the POST request. webwrite will JSON-encode the payload struct.
    response = webwrite(char(string(WEB.apiUrl)), payload, opts);
end