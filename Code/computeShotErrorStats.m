function stats = computeShotErrorStats(shot)
% computeShotErrorStats
% Returns mean / max trajectory error stats for a tracked shot.
%
% Input:
%   shot struct with fields:
%       xyzTrue   [N x 3]
%       xyzMeas   [N x 3]
%       valid     [N x 1] logical (optional)
%       name      string/char (optional)
%       bounceIdx vector (optional)
%
% Output:
%   stats struct with fields:
%       ok
%       name
%       n
%       dx, dy, dz          per-sample signed errors
%       e                   per-sample magnitude errors
%       meanDx, meanDy, meanDz
%       meanE, maxE, rmseE
%       bounceError         struct for first bounce, if available

    stats = struct( ...
        'ok', false, ...
        'name', "", ...
        'n', 0, ...
        'dx', [], ...
        'dy', [], ...
        'dz', [], ...
        'e', [], ...
        'meanDx', NaN, ...
        'meanDy', NaN, ...
        'meanDz', NaN, ...
        'meanE', NaN, ...
        'maxE', NaN, ...
        'rmseE', NaN, ...
        'bounceError', struct( ...
            'available', false, ...
            'idx', NaN, ...
            'dx', NaN, ...
            'dy', NaN, ...
            'dz', NaN, ...
            'e', NaN));

    if ~isstruct(shot) || ~isfield(shot,'xyzTrue') || ~isfield(shot,'xyzMeas')
        return;
    end

    if isempty(shot.xyzTrue) || isempty(shot.xyzMeas)
        return;
    end

    if size(shot.xyzTrue,2) ~= 3 || size(shot.xyzMeas,2) ~= 3
        return;
    end

    n = min(size(shot.xyzTrue,1), size(shot.xyzMeas,1));
    xyzTrue = shot.xyzTrue(1:n,:);
    xyzMeas = shot.xyzMeas(1:n,:);

    if isfield(shot,'valid') && ~isempty(shot.valid)
        valid = logical(shot.valid(1:min(numel(shot.valid),n)));
        if numel(valid) < n
            valid(end+1:n,1) = true;
        end
    else
        valid = all(isfinite(xyzTrue),2) & all(isfinite(xyzMeas),2);
    end

    good = valid ...
        & all(isfinite(xyzTrue),2) ...
        & all(isfinite(xyzMeas),2);

    if ~any(good)
        return;
    end

    d = xyzMeas(good,:) - xyzTrue(good,:);
    e = sqrt(sum(d.^2,2));

    stats.ok = true;
    if isfield(shot,'name') && ~isempty(shot.name)
        stats.name = string(shot.name);
    end
    stats.n = size(d,1);

    stats.dx = d(:,1);
    stats.dy = d(:,2);
    stats.dz = d(:,3);
    stats.e  = e;

    stats.meanDx = mean(d(:,1),'omitnan');
    stats.meanDy = mean(d(:,2),'omitnan');
    stats.meanDz = mean(d(:,3),'omitnan');
    stats.meanE  = mean(e,'omitnan');
    stats.maxE   = max(e,[],'omitnan');
    stats.rmseE  = sqrt(mean(e.^2,'omitnan'));

    if isfield(shot,'bounceIdx') && ~isempty(shot.bounceIdx)
        k = shot.bounceIdx(1);
        if isnumeric(k) && isfinite(k) && k >= 1 && k <= n ...
                && all(isfinite(xyzTrue(k,:))) && all(isfinite(xyzMeas(k,:)))
            db = xyzMeas(k,:) - xyzTrue(k,:);
            stats.bounceError.available = true;
            stats.bounceError.idx = k;
            stats.bounceError.dx = db(1);
            stats.bounceError.dy = db(2);
            stats.bounceError.dz = db(3);
            stats.bounceError.e  = norm(db);
        end
    end
end