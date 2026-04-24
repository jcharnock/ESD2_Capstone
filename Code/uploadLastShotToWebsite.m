function response = uploadLastShotToWebsite(lastShot, shotCategory, WEB)
% Build a website payload and send it.
%
% Inputs:
%   lastShot     - Shot struct produced by the GUI/tracking pipeline.
%   shotCategory - String label for the type of event being uploaded,
%                  e.g. "serve", "volley", or "cor".
%   WEB          - Website/API settings struct. Expected fields include:
%                    WEB.enabled
%                    WEB.apiUrl
%                    WEB.apiKey
%                    WEB.timeoutSec
%                    WEB.deviceName
%
% Output:
%   response     - Whatever the backend returns from the POST request.
%
% Purpose:
%   This helper keeps the GUI code simple. Instead of building JSON and
%   posting directly inside the GUI callbacks, the GUI can call this one
%   wrapper function.

    % Convert the internal MATLAB shot struct into a clean payload intended
    % for the website/backend.
    payload = buildWebPayload(lastShot, shotCategory, WEB);

    % Send that payload to the backend API.
    response = pushShotToWebsite(payload, WEB);
end