function Project_GUI_BlenderOnly
%% Project GUI + Blender Only
% MATLAB handles:
%   - GUI
%   - Blender rendering control
%   - shot tracking over time
%   - coefficient of restitution
%   - LED visualization
%   - serve / volley tracking
%   - instant replay
%
% C++ backend handles:
%   - ball detection
%   - stereo pair selection
%   - triangulation

clc; close all;

%% -------------------- Blender Server Settings --------------------
BLENDER.server_ip   = '127.0.0.1';
BLENDER.server_port = 55001;
BLENDER.ballName    = "tennisBall";

BLENDER.pair1.camL = "Camera1";
BLENDER.pair1.camR = "Camera2";
BLENDER.pair1.camBaseX = 0.0;
BLENDER.pair1.camBaseY = -15.0;
BLENDER.pair1.camBaseZ = 12.0;
BLENDER.pair1.B_m      = 3;
BLENDER.pair1.camPitch = 30.0;
BLENDER.pair1.camRoll  = 0.0;
BLENDER.pair1.camYaw   = 0.0;

BLENDER.pair2.camL = "Camera3";
BLENDER.pair2.camR = "Camera4";
BLENDER.pair2.camBaseX = 0.0;
BLENDER.pair2.camBaseY = -3.0;
BLENDER.pair2.camBaseZ = 12.0;
BLENDER.pair2.B_m      = 3;
BLENDER.pair2.camPitch = 38.0;
BLENDER.pair2.camRoll  = 0.0;
BLENDER.pair2.camYaw   = 0.0;

BLENDER.width  = 1280;
BLENDER.height = 720;
BLENDER.ballPitch = 0;
BLENDER.ballRoll  = 0;
BLENDER.ballYaw   = 0;

%% -------------------- Defaults --------------------
defaults.Ztrue_m = 1.5;
defaults.f_px = 853.3333;
defaults.netVFrac = 0.42;
defaults.radiusDiffPx = 2.0;
defaults.overlayRadiusPx = 16;
defaults.maxReplayFPS = 20;
defaults.outBlinkHz = 10;
defaults.sampleDt = 0.05;
defaults.serveDuration = 1.00;
defaults.volleyDuration = 0.90;
defaults.serveNumSamples = 21;
defaults.volleyNumSamples = 19;

% Real Blender court-plane height from your measured vertices
WORLD.groundZ = 0.35951;

if ispc
    defaults.outputRoot = 'C:\Users\Matthew\CPET\CPET563\Project\ESD2_Capstone\Code\Project_Code\Project_Code\tracker_runs';
else
    defaults.outputRoot = fullfile(tempdir, 'tracker_runs');
end

%% -------------------- State --------------------
client = [];
busy = false;

trajXYZ = zeros(0,3);
trajT   = zeros(0,1);

lastShot = struct( ...
    'name', "", ...
    't', zeros(0,1), ...
    'xyzTrue', zeros(0,3), ...
    'xyzMeas', zeros(0,3), ...
    'xyz', zeros(0,3), ...
    'valid', false(0,1), ...
    'decision', "", ...
    'bounceIdx', [], ...
    'restitution', NaN, ...
    'framesL', {{}}, ...
    'framesR', {{}}, ...
    'selectedPair', zeros(0,1) );

replayTimer = [];
ledTimer = [];
ledBlinkState = false;

COURT = defaultCourtGeometry();

% Real Blender court boundary polygon from measured corner vertices
% near left: (-4.7767, -10.318, 0.35951)
% near right: (4.7767, -10.318, 0.35951)
% far left: (-4.7767, 10.347, 0.35951)
% far right: (4.7088, 10.259, 0.35951)
COURT.polyX = [-4.7767,  4.7767,  4.7088, -4.7767];
COURT.polyY = [-10.318, -10.318, 10.259, 10.347];
COURT.groundZ = WORLD.groundZ;

if ~exist(defaults.outputRoot, 'dir')
    mkdir(defaults.outputRoot);
end

%% -------------------- GUI --------------------
ui = uifigure('Name','Ball Tracking GUI (MATLAB UI + Blender only)', ...
    'Position',[40 30 1500 920]);
ui.CloseRequestFcn = @(~,~) onClose();

main = uigridlayout(ui,[3 3]);
main.ColumnWidth = {470,'1x','1x'};
main.RowHeight   = {430, 230, '1x'};
main.Padding = [10 10 10 10];
main.RowSpacing = 10;
main.ColumnSpacing = 10;

pCtrl = uipanel(main,'Title','Controls');
pCtrl.Layout.Row = 1;
pCtrl.Layout.Column = 1;

g = uigridlayout(pCtrl,[16 3]);
g.RowHeight = {24,24,24,24,24,30,30,30,30,30,30,24,24,24,24,20};
g.Padding = [6 6 6 6];
g.RowSpacing = 2;
g.ColumnSpacing = 6;
g.ColumnWidth = {125,'1x',90};

uilabel(g,'Text','Blender depth (m)');
sZ = uislider(g,'Limits',[0.5 3.0],'Value',defaults.Ztrue_m);
sZ.Layout.Row = 1; sZ.Layout.Column = 2;
eZ = uieditfield(g,'numeric','Value',defaults.Ztrue_m);
eZ.Layout.Row = 1; eZ.Layout.Column = 3;

uilabel(g,'Text','f (pixels)');
eF = uieditfield(g,'numeric','Value',defaults.f_px);
eF.Layout.Row = 2; eF.Layout.Column = [2 3];

uilabel(g,'Text','Net line frac');
eNet = uieditfield(g,'numeric','Value',defaults.netVFrac,'Limits',[0.05 0.95]);
eNet.Layout.Row = 3; eNet.Layout.Column = [2 3];

uilabel(g,'Text','Radius diff (px)');
eRad = uieditfield(g,'numeric','Value',defaults.radiusDiffPx,'Limits',[0 50]);
eRad.Layout.Row = 4; eRad.Layout.Column = [2 3];

uilabel(g,'Text','Backend exe');
exeField = uieditfield(g,'text','Value',getBackendExecutablePath());
exeField.Layout.Row = 5; exeField.Layout.Column = [2 3];

btnRun = uibutton(g,'Text','Render + Track','ButtonPushedFcn',@(~,~) safeUpdateOnce());
btnRun.Layout.Row = 6; btnRun.Layout.Column = [1 3];

btnClear = uibutton(g,'Text','Clear Trajectory','ButtonPushedFcn',@(~,~) clearTrajectory());
btnClear.Layout.Row = 7; btnClear.Layout.Column = [1 3];

btnServe = uibutton(g,'Text','Track Serve','ButtonPushedFcn',@(~,~) onTrackServe());
btnServe.Layout.Row = 8; btnServe.Layout.Column = [1 3];

btnVolley = uibutton(g,'Text','Track Volley','ButtonPushedFcn',@(~,~) onTrackVolley());
btnVolley.Layout.Row = 9; btnVolley.Layout.Column = [1 3];

btnCOR = uibutton(g,'Text','Compute Restitution','ButtonPushedFcn',@(~,~) onComputeCOR());
btnCOR.Layout.Row = 10; btnCOR.Layout.Column = [1 3];

btnReplay = uibutton(g,'Text','Instant Replay','ButtonPushedFcn',@(~,~) onInstantReplay());
btnReplay.Layout.Row = 11; btnReplay.Layout.Column = [1 3];

btnSweep = uibutton(g,'Text','Run Sweep','ButtonPushedFcn',@(~,~) onRunSweep());
btnSweep.Layout.Row = 12; btnSweep.Layout.Column = [1 3];

shotSummary = uilabel(g,'Text','Shot: --');
shotSummary.Layout.Row = 13; shotSummary.Layout.Column = [1 3];

uilabel(g,'Text','LED state');
lblLED = uilabel(g,'Text','--');
lblLED.Layout.Row = 14; lblLED.Layout.Column = [2 3];

statusLabel = uilabel(g,'Text','Idle.');
statusLabel.Layout.Row = 15; statusLabel.Layout.Column = [1 3];
pL = uipanel(main,'Title','Chosen Left Image');
pL.Layout.Row = 1; pL.Layout.Column = 2;
axL = uiaxes(pL);
axL.Position = [10 10 560 230];
axis(axL,'image'); axis(axL,'off');

pR = uipanel(main,'Title','Chosen Right Image');
pR.Layout.Row = 1; pR.Layout.Column = 3;
axR = uiaxes(pR);
axR.Position = [10 10 560 230];
axis(axR,'image'); axis(axR,'off');

pOut = uipanel(main,'Title','Output');
pOut.Layout.Row = [2 3]; pOut.Layout.Column = 1;

go = uigridlayout(pOut,[16 1]);
go.RowHeight = {30,24,24,24,24,24,24,24,24,24,24,24,24,24,54,24,'1x'};
go.Padding = [8 8 8 8];
go.RowSpacing = 1;

lblPair   = uilabel(go,'Text','Chosen pair: --','FontWeight','bold');
lblCourt  = uilabel(go,'Text','Court side: --');
lblL      = uilabel(go,'Text','Left centroid: --');
lblR      = uilabel(go,'Text','Right centroid: --');
lblDisp   = uilabel(go,'Text','Disparity: --');
lblX      = uilabel(go,'Text','X: --');
lblY      = uilabel(go,'Text','Y: --');
lblZ      = uilabel(go,'Text','Z: --');
lblErr = uilabel(go,'Text','Error: --');
lblMsg1   = uilabel(go,'Text','Backend: --');
lblMsg2   = uilabel(go,'Text','Blender: --');
lblShotType = uilabel(go,'Text','Shot type: --');
lblDecision = uilabel(go,'Text','Decision: --');
lblCOR      = uilabel(go,'Text','Restitution: --');

ledPanel = uipanel(go,'Title','LED Visualization');
ledPanel.Layout.Row = 14;
ledGrid = uigridlayout(ledPanel,[1 4]);
ledGrid.ColumnWidth = {45,120,45,120};

ledGreen = uilamp(ledGrid,'Color',[0.2 0.2 0.2]);
ledGreen.Layout.Row = 1; ledGreen.Layout.Column = 1;
uilabel(ledGrid,'Text','IN');

ledRed = uilamp(ledGrid,'Color',[0.2 0.2 0.2]);
ledRed.Layout.Row = 1; ledRed.Layout.Column = 3;
uilabel(ledGrid,'Text','OUT');

lblBounce = uilabel(go,'Text','Bounce: --');

msg = uitextarea(go,'Editable','off');
msg.Layout.Row = 16;

p3 = uipanel(main,'Title','3D Trajectory');
p3.Layout.Row = 3;
p3.Layout.Column = [2 3];
ax3 = uiaxes(p3);
ax3.Position = [10 10 980 320];

xlabel(ax3,'X (m)');
ylabel(ax3,'Y (m)');
zlabel(ax3,'Z (m)');
title(ax3,'Trajectory View');
grid(ax3,'on');
view(ax3,3);
hold(ax3,'on');

hTraj = plot3(ax3, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);
hTrue = plot3(ax3, NaN, NaN, NaN, 'k--', 'LineWidth', 1.2);
hPt = plot3(ax3, NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hBounce = plot3(ax3, NaN, NaN, NaN, 'mo', 'MarkerSize', 12, 'LineWidth', 2);

hold(ax3,'off');

pCourt = uipanel(main,'Title','Court View (Top Down)');
pCourt.Layout.Row = 2;
pCourt.Layout.Column = [2 3];

axCourt = uiaxes(pCourt);
axCourt.Position = [10 10 980 250];
xlabel(axCourt,'Y (m)');
ylabel(axCourt,'-X (m)');
title(axCourt,'Court View (Top Down) | Rotated to match Blender top view');
grid(axCourt,'on');

drawCourtTopDown();
hold(axCourt,'on');

hCourtBounce = plot(axCourt, NaN, NaN, 'rx', 'MarkerSize', 12, 'LineWidth', 2);
hCourtCurrent = plot(axCourt, NaN, NaN, 'bo', 'MarkerSize', 8, 'LineWidth', 2);
hCourtTarget  = plot(axCourt, NaN, NaN, 'ks', 'MarkerSize', 8, 'LineWidth', 2);

hold(axCourt,'off');

sZ.ValueChangingFcn = @(~,evt) setIfValid(eZ,'Value',evt.Value);
sZ.ValueChangedFcn  = @(src,~) setIfValid(eZ,'Value',src.Value);
eZ.ValueChangedFcn  = @(src,~) setIfValid(sZ,'Value',src.Value);

    %% -------------------- Main update --------------------
    function updateOnce()
        if ~isempty(lastShot) && isfield(lastShot,'xyzTrue') && ~isempty(lastShot.xyzTrue)
            kShow = getPrimaryBounceIndex(lastShot.bounceIdx, size(lastShot.xyzTrue,1));
            if isempty(kShow)
                kShow = size(lastShot.xyzTrue,1);
            end
            ballXYZ = lastShot.xyzTrue(kShow,:);
        else
            ballXYZ = [0, -7.5, WORLD.groundZ + eZ.Value];
        end
    
        renderTrackAtBall(ballXYZ, true);
    end

    function pt = renderTrackAtBall(ballXYZ, appendToTrajectory)
        pt = struct('ok',false);

        if busy || ~isUIAlive()
            return;
        end

        busy = true;
        disableButtons();
        c = onCleanup(@() releaseBusy()); %#ok<NASGU>

        f_px = eF.Value;
        netFrac = eNet.Value;
        radiusDiff = eRad.Value;
        backendExe = strtrim(exeField.Value);

        if ~isfile(backendExe)
            safeStatus('Backend executable not found.');
            safeMsg(["Backend executable not found:"; string(backendExe)]);
            return;
        end

        safeStatus('Rendering from Blender...');
        drawnow limitrate;

        [I1L, I1R, I2L, I2R, blenderInfo] = getFourViewsFromBlender(ballXYZ);
        if isempty(I1L) || isempty(I1R) || isempty(I2L) || isempty(I2R)
            safeStatus('Blender render failed.');
            safeMsg(blenderInfo(:));
            if appendToTrajectory
                clearReadouts();
                claIfValid(axL);
                claIfValid(axR);
            end
            return;
        end

        safeStatus('Writing images + calling backend...');
        drawnow limitrate;

        runFolder = fullfile(defaults.outputRoot, datestr(now,'yyyymmdd_HHMMSS_FFF'));
        mkdir(runFolder);

        p1L = fullfile(runFolder,'pair1_left.png');
        p1R = fullfile(runFolder,'pair1_right.png');
        p2L = fullfile(runFolder,'pair2_left.png');
        p2R = fullfile(runFolder,'pair2_right.png');
        cfg = fullfile(runFolder,'config.txt');
        res = fullfile(runFolder,'result.txt');

        imwrite(I1L, p1L);
        imwrite(I1R, p1R);
        imwrite(I2L, p2L);
        imwrite(I2R, p2R);

        writeBackendConfig(cfg, f_px, netFrac, radiusDiff, BLENDER);

        [ok, backendInfo, result] = callTrackerBackend(backendExe, p1L, p1R, p2L, p2R, cfg, res);
        dx = NaN; dy = NaN; dz = NaN; dmag = NaN;
        if ok
            dx = result.X - ballXYZ(1);
            dy = result.Y - ballXYZ(2);
            dz = result.Z - ballXYZ(3);
            dmag = hypot(hypot(dx,dy),dz);
        end

        if ~ok
            safeStatus('Backend failed.');
            safeMsg([blenderInfo(:); " "; backendInfo(:); ...
                sprintf('Target XYZ = (%.3f, %.3f, %.3f)', ballXYZ(1), ballXYZ(2), ballXYZ(3)); ...
                sprintf('Measured XYZ = (%.3f, %.3f, %.3f)', result.X, result.Y, result.Z); ...
                sprintf('Error XYZ = (%+.3f, %+.3f, %+.3f)', dx, dy, dz)]);            if appendToTrajectory
                clearReadouts();
            end
            return;
        end

        if result.pair_index == 1
            IL = I1L; 
            IR = I1R;
        else
            IL = I2L; 
            IR = I2R;
        end

        if appendToTrajectory
            showTrackedImages(IL, IR, result);

            lblPair.Text  = sprintf('Chosen pair: %d', result.pair_index);
            lblCourt.Text = sprintf('Court side: %s', result.court_side);
            lblL.Text     = sprintf('Left centroid: (%.2f, %.2f)', result.left_u, result.left_v);
            lblR.Text     = sprintf('Right centroid: (%.2f, %.2f)', result.right_u, result.right_v);
            lblDisp.Text  = sprintf('Disparity: du=%.3f px  dv=%.3f px', result.disparity_u, result.disparity_v);
            lblX.Text     = sprintf('X: %.4f m', result.X);
            lblY.Text     = sprintf('Y: %.4f m', result.Y);
            lblZ.Text     = sprintf('Z: %.4f m', result.Z);
            
            lblErr.Text = sprintf('Error: dX=%+.3f  dY=%+.3f  dZ=%+.3f  |e|=%.3f m', ...
                dx, dy, dz, dmag);
            lblMsg1.Text  = sprintf('Backend: %s', result.message);
            lblMsg2.Text  = sprintf('Blender: %s', strjoin(blenderInfo, ' | '));
        end

        if appendToTrajectory
            trajXYZ(end+1,:) = [result.X result.Y result.Z]; %#ok<AGROW>
            if isempty(trajT)
                trajT(end+1,1) = 0;
            else
                trajT(end+1,1) = trajT(end) + defaults.sampleDt;
            end
        
            if exist('hCourtCurrent','var') && isgraphics(hCourtCurrent)
                hCourtCurrent.XData = result.Y;
                hCourtCurrent.YData = -result.X;
            end
            
            if exist('hCourtTarget','var') && isgraphics(hCourtTarget)
                hCourtTarget.XData = ballXYZ(2);
                hCourtTarget.YData = -ballXYZ(1);
            end
        
            update3DPlot();
            autoScale3D();
        end

        safeStatus('Done.');
        safeMsg([blenderInfo(:); " "; backendInfo(:); ...
            sprintf('Target XYZ = (%.3f, %.3f, %.3f)', ballXYZ(1), ballXYZ(2), ballXYZ(3)); ...
            sprintf('Measured XYZ = (%.3f, %.3f, %.3f)', result.X, result.Y, result.Z); ...
            sprintf('Error XYZ = (%+.3f, %+.3f, %+.3f)', dx, dy, dz)]);
        
        pt.ok = true;
        pt.X = result.X;
        pt.Y = result.Y;
        pt.Z = result.Z;
        pt.result = result;
        pt.IL = IL;
        pt.IR = IR;
        pt.infoLines = [blenderInfo(:); " "; backendInfo(:)];
    end

    function onTrackServe()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Tracking serve...');
        drawnow limitrate;
    
        tVec = linspace(0, defaults.serveDuration, defaults.serveNumSamples);
        shot = captureShotTrajectory(tVec, 'serve_default');
    
        % testing the error vs. different locations on court
        serveStats = computeShotErrorStats(shot);
        disp(serveStats)

        if ~any(shot.valid)
            safeStatus('Serve tracking failed.');
            return;
        end
    
        shot.decision = classifyInOut(shot.xyzMeas, shot.bounceIdx, COURT, 'serve');
        shot.restitution = computeRestitution(shot.t, shot.xyzMeas, shot.bounceIdx);
    
        lastShot = shot;
        trajXYZ = shot.xyzMeas;
        trajT   = shot.t;

        hTrue.XData = shot.xyzTrue(:,1);
        hTrue.YData = shot.xyzTrue(:,2);
        hTrue.ZData = shot.xyzTrue(:,3);

        updateCourtBounceMarkers(shot);
        updateShotReadouts(shot);
        updateLEDFromDecision(shot.decision);
        update3DPlot();
        autoScale3D();
        safeStatus('Serve tracked.');
        
        showShotFrame(shot, 'Serve');
    end

    function onTrackVolley()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Tracking volley...');
        drawnow limitrate;
    
        tVec = linspace(0, defaults.volleyDuration, defaults.volleyNumSamples);
        shot = captureShotTrajectory(tVec, 'volley_default');
    
        % Tracking error vs court location for volley
        volleyStats = computeShotErrorStats(shot);
        disp(volleyStats)

        if ~any(shot.valid)
            safeStatus('Volley tracking failed.');
            return;
        end
    
        shot.decision = classifyInOut(shot.xyzMeas, shot.bounceIdx, COURT, 'volley');
        shot.restitution = computeRestitution(shot.t, shot.xyzMeas, shot.bounceIdx);
    
        lastShot = shot;
        trajXYZ = shot.xyzMeas;
        trajT   = shot.t;

        hTrue.XData = shot.xyzTrue(:,1);
        hTrue.YData = shot.xyzTrue(:,2);
        hTrue.ZData = shot.xyzTrue(:,3);

        updateCourtBounceMarkers(shot);
        updateShotReadouts(shot);
        updateLEDFromDecision(shot.decision);
        update3DPlot();
        autoScale3D();
        safeStatus('Volley tracked.');
        
        showShotFrame(shot, 'Volley');
    end

    function onComputeCOR()
        if isempty(lastShot) || isempty(lastShot.t)
            safeStatus('No shot available for restitution.');
            return;
        end

        cor = computeRestitution(lastShot.t, lastShot.xyz, lastShot.bounceIdx);
        lastShot.restitution = cor;

        if isnan(cor)
            lblCOR.Text = 'Restitution: not enough bounce data';
            safeStatus('Could not compute restitution.');
        else
            lblCOR.Text = sprintf('Restitution: %.4f', cor);
            safeStatus('Restitution computed.');
        end
    end

    function onInstantReplay()
        if isempty(lastShot) || isempty(lastShot.t) || ~any(lastShot.valid)
            safeStatus('No saved shot for replay.');
            return;
        end
    
        stopReplayTimer();
        replayIdx = 1;
        N = numel(lastShot.t);
        period = max(0.03, 1/defaults.maxReplayFPS);
    
        replayTimer = timer( ...
            'ExecutionMode','fixedRate', ...
            'Period', period, ...
            'TimerFcn', @doReplayStep, ...
            'BusyMode','drop');
    
        safeStatus('Instant replay running...');
        start(replayTimer);
    
        function doReplayStep(~,~)
            if ~isUIAlive()
                stopReplayTimer();
                return;
            end
    
            if replayIdx > N
                stopReplayTimer();
                safeStatus('Replay finished.');
                return;
            end
    
            xyzNow = lastShot.xyzMeas(replayIdx,:);
            hPt.XData = xyzNow(1);
            hPt.YData = xyzNow(2);
            hPt.ZData = xyzNow(3);
    
            hTraj.XData = lastShot.xyzMeas(1:replayIdx,1);
            hTraj.YData = lastShot.xyzMeas(1:replayIdx,2);
            hTraj.ZData = lastShot.xyzMeas(1:replayIdx,3);
    
            bounceShown = lastShot.bounceIdx(lastShot.bounceIdx <= replayIdx);
            if ~isempty(bounceShown)
                bp = lastShot.xyzMeas(bounceShown,:);
                hBounce.XData = bp(:,1);
                hBounce.YData = bp(:,2);
                hBounce.ZData = bp(:,3);
            else
                hBounce.XData = NaN;
                hBounce.YData = NaN;
                hBounce.ZData = NaN;
            end
    
            if replayIdx <= numel(lastShot.framesL) && ~isempty(lastShot.framesL{replayIdx})
                imshow(lastShot.framesL{replayIdx}, 'Parent', axL);
                title(axL, sprintf('Replay Left | %s | t=%.2f s', lastShot.name, lastShot.t(replayIdx)));
            end
            if replayIdx <= numel(lastShot.framesR) && ~isempty(lastShot.framesR{replayIdx})
                imshow(lastShot.framesR{replayIdx}, 'Parent', axR);
                title(axR, sprintf('Replay Right | %s | t=%.2f s', lastShot.name, lastShot.t(replayIdx)));
            end
    
            drawnow limitrate;
            replayIdx = replayIdx + 1;
        end
    end

    function shot = captureShotTrajectory(tVec, shotPreset)
        shot = struct( ...
            'name', string(shotPreset), ...
            't', tVec(:), ...
            'xyzTrue', NaN(numel(tVec),3), ...
            'xyzMeas', NaN(numel(tVec),3), ...
            'xyz', NaN(numel(tVec),3), ...
            'valid', false(numel(tVec),1), ...
            'decision', "", ...
            'bounceIdx', [], ...
            'restitution', NaN, ...
            'framesL', {cell(numel(tVec),1)}, ...
            'framesR', {cell(numel(tVec),1)}, ...
            'selectedPair', NaN(numel(tVec),1) );
    
        stopReplayTimer();
    
        for k = 1:numel(tVec)
            if ~isUIAlive()
                return;
            end
    
            ballXYZ = shotFunctionAtTime(tVec(k), shotPreset);
            shot.xyzTrue(k,:) = ballXYZ;
    
            pt = renderTrackAtBall(ballXYZ, false);
    
            if pt.ok
                shot.xyzMeas(k,:) = [pt.X pt.Y pt.Z];
                shot.xyz(k,:) = shot.xyzMeas(k,:);
                shot.valid(k) = true;
                shot.framesL{k} = pt.IL;
                shot.framesR{k} = pt.IR;
                shot.selectedPair(k) = pt.result.pair_index;
            end
        end
    
        % Find bounce on full TRUE trajectory first
        fullBounceIdx = findBounceIndex(shot.xyzTrue);
        fullBounceIdx = getPrimaryBounceIndex(fullBounceIdx, numel(shot.t));
    
        % Save originals before filtering
        tFull = shot.t;
        validFull = shot.valid;
    
        keep = shot.valid;
        shot.t = shot.t(keep);
        shot.xyzTrue = shot.xyzTrue(keep,:);
        shot.xyzMeas = shot.xyzMeas(keep,:);
        shot.xyz = shot.xyzMeas;
        shot.valid = shot.valid(keep);
        shot.framesL = shot.framesL(keep);
        shot.framesR = shot.framesR(keep);
        shot.selectedPair = shot.selectedPair(keep);
    
        % Map true bounce time to nearest surviving valid sample
        shot.bounceIdx = [];
        if ~isempty(fullBounceIdx) && validFull(fullBounceIdx) && ~isempty(shot.t)
            bounceTime = tFull(fullBounceIdx);
            [~, kNearest] = min(abs(shot.t - bounceTime));
            shot.bounceIdx = kNearest;
        elseif ~isempty(fullBounceIdx) && ~isempty(shot.t)
            bounceTime = tFull(fullBounceIdx);
            [~, kNearest] = min(abs(shot.t - bounceTime));
            shot.bounceIdx = kNearest;
        end
            if isempty(shot.bounceIdx) && ~isempty(shot.xyzMeas)
                [~, kMinMeas] = min(shot.xyzMeas(:,3));
                if ~isempty(kMinMeas) && isfinite(kMinMeas)
                    shot.bounceIdx = kMinMeas;
                end
            end
            fprintf('Preset %s | bounceIdx = ', string(shotPreset));
            disp(shot.bounceIdx)
    end

    function ballXYZ = shotFunctionAtTime(t, shotPreset)
        if ischar(shotPreset) || isstring(shotPreset)
            preset = getShotPreset(shotPreset, WORLD.groundZ);
        else
            preset = shotPreset;
        end
    
        x = preset.x0 + preset.vx*t;
        y = preset.y0 + preset.vy*t;
        z = preset.z0 + preset.vz*t - 0.5*preset.g*t^2;
    
        if t > preset.bounceT
            xb = preset.x0 + preset.vx*preset.bounceT;
            yb = preset.y0 + preset.vy*preset.bounceT;
            zb = max(WORLD.groundZ, preset.z0 + preset.vz*preset.bounceT - 0.5*preset.g*preset.bounceT^2);
            vzBefore = preset.vz - preset.g*preset.bounceT;
            vzAfter  = abs(vzBefore) * preset.eApprox;
            dt = t - preset.bounceT;
    
            x = xb + preset.vx*dt;
            y = yb + preset.vy*dt;
            z = zb + vzAfter*dt - 0.5*preset.g*dt^2;
        end
    
        z = max(WORLD.groundZ, z);
        ballXYZ = [x, y, z];
    end

    function idx = findBounceIndex(xyz)
        idx = [];
        if size(xyz,1) < 3
            return;
        end
    
        z = xyz(:,3);
    
        % Prefer the global minimum, which is much more robust than
        % requiring a very low local minimum threshold.
        [~, idxMin] = min(z);
    
        if idxMin > 1 && idxMin < size(xyz,1)
            idx = idxMin;
        end
    end

    function e = computeRestitution(t, xyz, bounceIdx)
        e = NaN;
        k = getPrimaryBounceIndex(bounceIdx, numel(t));
        if isempty(k) || numel(t) < 5 || k < 3 || k > numel(t)-2
            return;
        end

        z = xyz(:,3);
        vzBefore = (z(k) - z(k-2)) / (t(k) - t(k-2));
        vzAfter  = (z(k+2) - z(k)) / (t(k+2) - t(k));

        if abs(vzBefore) < 1e-6
            return;
        end

        e = abs(vzAfter / vzBefore);
    end

    function decision = classifyInOut(xyz, bounceIdx, courtGeom, shotType)
        decision = "UNKNOWN";

        k = getPrimaryBounceIndex(bounceIdx, size(xyz,1));
        if isempty(k)
            return;
        end

        p = xyz(k,:);
        inCourt = inpolygon(p(1), p(2), courtGeom.polyX, courtGeom.polyY);

        switch lower(string(shotType))
            case "serve"
                if inCourt
                    decision = "IN";
                else
                    decision = "OUT";
                end
            otherwise
                if inCourt
                    decision = "IN";
                else
                    decision = "OUT";
                end
        end
    end

    function geom = defaultCourtGeometry()
        geom.singlesHalfWidth = 4.115;
        geom.courtYMin = -11.885;
        geom.courtYMax = 11.885;
        geom.serviceBoxYMin = 0.00;
        geom.serviceBoxYMax = 6.40;
        geom.polyX = [];
        geom.polyY = [];
        geom.groundZ = WORLD.groundZ;
    end

    function updateShotReadouts(shot)
        shotSummary.Text = sprintf('Shot: %s', shot.name);
        lblShotType.Text = sprintf('Shot type: %s', shot.name);
        lblDecision.Text = sprintf('Decision: %s', shot.decision);
    
        if isnan(shot.restitution)
            lblCOR.Text = 'Restitution: --';
        else
            lblCOR.Text = sprintf('Restitution: %.4f', shot.restitution);
        end

        if strcmpi(shot.decision,"IN")
            hBounce.Color = [0 1 0];
        else
            hBounce.Color = [1 0 0];
        end

        if isempty(shot.bounceIdx)
            lblBounce.Text = 'Bounce: not found';
            hBounce.XData = NaN; 
            hBounce.YData = NaN; 
            hBounce.ZData = NaN;
        else
            bp = shot.xyzMeas(shot.bounceIdx,:);
            lblBounce.Text = sprintf('Bounces found: %d', size(bp,1));
            hBounce.XData = bp(:,1);
            hBounce.YData = bp(:,2);
            hBounce.ZData = bp(:,3);
        end
        stats = computeShotErrorStats(shot);
        if stats.ok
            lblErr.Text = sprintf(['Error: mean dX=%+.3f  dY=%+.3f  dZ=%+.3f  ' ...
                '|e|mean=%.3f  |e|max=%.3f  RMSE=%.3f m'], ...
                stats.meanDx, stats.meanDy, stats.meanDz, ...
                stats.meanE, stats.maxE, stats.rmseE);
        else
            lblErr.Text = 'Error: --';
        end
        if stats.bounceError.available
            fprintf('Bounce error: dX=%+.3f  dY=%+.3f  dZ=%+.3f  |e|=%.3f m\n', ...
                stats.bounceError.dx, ...
                stats.bounceError.dy, ...
                stats.bounceError.dz, ...
                stats.bounceError.e);
        else
            fprintf('No bounce error available.\n');
        end
    end

    function updateLEDFromDecision(decision)
        stopLEDTimer();

        switch upper(string(decision))
            case "IN"
                ledGreen.Color = [0 1 0];
                ledRed.Color = [0.2 0.2 0.2];
                lblLED.Text = 'IN: green solid';

            case "OUT"
                ledGreen.Color = [0.2 0.2 0.2];
                lblLED.Text = 'OUT: red blinking';
                ledBlinkState = false;
                ledTimer = timer( ...
                    'ExecutionMode','fixedRate', ...
                    'Period', 1/(2*defaults.outBlinkHz), ...
                    'TimerFcn', @toggleRedLED, ...
                    'BusyMode','drop');
                start(ledTimer);

            otherwise
                ledGreen.Color = [0.2 0.2 0.2];
                ledRed.Color   = [0.2 0.2 0.2];
                lblLED.Text = '--';
        end
    end

    function toggleRedLED(~,~)
        if ~isUIAlive()
            stopLEDTimer();
            return;
        end

        ledBlinkState = ~ledBlinkState;
        if ledBlinkState
            ledRed.Color = [1 0 0];
        else
            ledRed.Color = [0.2 0.2 0.2];
        end
    end

    function showTrackedImages(IL, IR, result)
        if ~isvalid(axL) || ~isvalid(axR)
            return;
        end

        imshow(IL,'Parent',axL);
        title(axL, sprintf('Left | Pair %d', result.pair_index));
        hold(axL,'on');
        plot(axL, result.left_u, result.left_v, 'g+','MarkerSize',10,'LineWidth',2);
        try
            viscircles(axL,[result.left_u result.left_v],defaults.overlayRadiusPx,'Color','g','LineWidth',1);
        catch
        end
        hold(axL,'off');

        imshow(IR,'Parent',axR);
        title(axR, sprintf('Right | Pair %d', result.pair_index));
        hold(axR,'on');
        plot(axR, result.right_u, result.right_v, 'g+','MarkerSize',10,'LineWidth',2);
        try
            viscircles(axR,[result.right_u result.right_v],defaults.overlayRadiusPx,'Color','g','LineWidth',1);
        catch
        end
        hold(axR,'off');
    end

    function update3DPlot()
        if isempty(trajXYZ)
            hTraj.XData = NaN; hTraj.YData = NaN; hTraj.ZData = NaN;
            hPt.XData = NaN; hPt.YData = NaN; hPt.ZData = NaN;
            hBounce.XData = NaN; hBounce.YData = NaN; hBounce.ZData = NaN;
            return;
        end

        hTraj.XData = trajXYZ(:,1);
        hTraj.YData = trajXYZ(:,2);
        hTraj.ZData = trajXYZ(:,3);

        hPt.XData = trajXYZ(end,1);
        hPt.YData = trajXYZ(end,2);
        hPt.ZData = trajXYZ(end,3);
    end

    function autoScale3D()
        xlim(ax3, [-6 6]);
        ylim(ax3, [-13 13]);
        zlim(ax3, [WORLD.groundZ-0.1 15]);
        view(ax3, 3);
    end

    function k = getPrimaryBounceIndex(bounceIdx, n)
        k = [];
        if isempty(bounceIdx)
            return;
        end
        bounceIdx = bounceIdx(:)';
        bounceIdx = bounceIdx(bounceIdx >= 1 & bounceIdx <= n);
        if isempty(bounceIdx)
            return;
        end
        k = bounceIdx(1);
    end

    function updateCourtBounceMarkers(shot)
        if ~isgraphics(hCourtBounce)
            return;
        end

        if isempty(shot.bounceIdx)
            hCourtBounce.XData = NaN;
            hCourtBounce.YData = NaN;
        else
            hCourtBounce.XData = shot.xyzMeas(shot.bounceIdx,2);
            hCourtBounce.YData = -shot.xyzMeas(shot.bounceIdx,1);
        end
    end

    function clearTrajectory()
        trajXYZ = zeros(0,3);
        trajT   = zeros(0,1);
    
        lastShot = struct( ...
            'name', "", ...
            't', zeros(0,1), ...
            'xyzTrue', zeros(0,3), ...
            'xyzMeas', zeros(0,3), ...
            'xyz', zeros(0,3), ...
            'valid', false(0,1), ...
            'decision', "", ...
            'bounceIdx', [], ...
            'restitution', NaN, ...
            'framesL', {{}}, ...
            'framesR', {{}}, ...
            'selectedPair', zeros(0,1) );
    
        stopReplayTimer();
        stopLEDTimer();
    
        ledGreen.Color = [0.2 0.2 0.2];
        ledRed.Color   = [0.2 0.2 0.2];
        lblLED.Text = '--';
    
        hTraj.XData = NaN; hTraj.YData = NaN; hTraj.ZData = NaN;
        hPt.XData   = NaN; hPt.YData   = NaN; hPt.ZData   = NaN;
        hBounce.XData = NaN; hBounce.YData = NaN; hBounce.ZData = NaN;
    
        hTrue.XData = NaN;
        hTrue.YData = NaN;
        hTrue.ZData = NaN;
        
        hCourtBounce.XData = NaN;
        hCourtBounce.YData = NaN;

        hCourtCurrent.XData = NaN;
        hCourtCurrent.YData = NaN;

        hCourtTarget.XData = NaN;
        hCourtTarget.YData = NaN;

        shotSummary.Text = 'Shot: --';
        lblShotType.Text = 'Shot type: --';
        lblDecision.Text = 'Decision: --';
        lblCOR.Text      = 'Restitution: --';
        lblBounce.Text   = 'Bounce: --';
    
        clearReadouts();
        autoScale3D();
        safeStatus('Trajectory cleared.');
    end

    function clearReadouts()
        if ~isUIAlive(), return; end
        lblPair.Text  = 'Chosen pair: --';
        lblCourt.Text = 'Court side: --';
        lblL.Text     = 'Left centroid: --';
        lblR.Text     = 'Right centroid: --';
        lblDisp.Text  = 'Disparity: --';
        lblX.Text     = 'X: --';
        lblY.Text     = 'Y: --';
        lblZ.Text     = 'Z: --';
        lblErr.Text   = 'Error: --';
        lblMsg1.Text  = 'Backend: --';
        lblMsg2.Text  = 'Blender: --';
    end

    function [ok, infoLines, result] = callTrackerBackend(backendExe, p1L, p1R, p2L, p2R, cfg, res)
        ok = false;
        infoLines = string.empty(0,1);
        result = struct('pair_index',0,'court_side','--','left_u',NaN,'left_v',NaN, ...
            'right_u',NaN,'right_v',NaN,'disparity_u',NaN,'disparity_v',NaN, ...
            'X',NaN,'Y',NaN,'Z',NaN,'message','--');

        cmd = sprintf('"%s" "%s" "%s" "%s" "%s" "%s" "%s"', ...
            backendExe, p1L, p1R, p2L, p2R, cfg, res);

        disp("MATLAB command being run:")
        disp(cmd)
        [status, cmdout] = system(cmd);
        disp("Backend exit status:")
        disp(status)
        disp("Backend stdout:")
        disp(cmdout)
        disp("Result file path:")
        disp(res)
        disp("Result file exists?")
        disp(isfile(res))

        infoLines = ["Command:"; string(cmd); "Stdout:"; string(splitlines(string(cmdout)))];

        if status ~= 0
            infoLines(end+1) = sprintf('Backend exit code: %d', status);
            if ~isfile(res)
                return;
            end
        elseif ~isfile(res)
            infoLines(end+1) = 'Result file not produced.';
            return;
        end

        parsed = parseKeyValueFile(res);
        if ~isfield(parsed,'status') || ~strcmpi(parsed.status,'ok')
            if isfield(parsed,'message')
                infoLines(end+1) = "Backend message: " + string(parsed.message);
            end
            return;
        end

        result.pair_index  = str2double(parsed.pair_index);
        result.court_side  = string(parsed.court_side);
        result.left_u      = str2double(parsed.left_u);
        result.left_v      = str2double(parsed.left_v);
        result.right_u     = str2double(parsed.right_u);
        result.right_v     = str2double(parsed.right_v);
        result.disparity_u = str2double(parsed.disparity_u);
        result.disparity_v = str2double(parsed.disparity_v);
        result.X           = str2double(parsed.X);
        result.Y           = str2double(parsed.Y);
        result.Z           = str2double(parsed.Z);
        result.message     = string(parsed.message);
        ok = true;
    end

    function S = parseKeyValueFile(filename)
        S = struct();
        lines = string(splitlines(fileread(filename)));
        lines = strtrim(lines);
        lines(lines == "") = [];
        for i = 1:numel(lines)
            line = lines(i);
            eq = strfind(line, '=');
            if isempty(eq), continue; end
            k = matlab.lang.makeValidName(strtrim(extractBefore(line, eq(1))));
            v = strtrim(extractAfter(line, eq(1)));
            S.(k) = char(v);
        end
    end

    function writeBackendConfig(filename, f_px, netFrac, radiusDiff, BL)
        fid = fopen(filename,'w');
        cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    
        fx_px = f_px;
        fy_px = f_px;
    
        fprintf(fid,'fx_px=%.12g\n', fx_px);
        fprintf(fid,'fy_px=%.12g\n', fy_px);
        fprintf(fid,'cx=%.12g\n', (BL.width - 1)/2);
        fprintf(fid,'cy=%.12g\n', (BL.height - 1)/2);
        fprintf(fid,'pair1_baseline_m=%.12g\n', BL.pair1.B_m);
        fprintf(fid,'pair2_baseline_m=%.12g\n', BL.pair2.B_m);
        fprintf(fid,'net_v_frac=%.12g\n', netFrac);
        fprintf(fid,'radius_diff_px=%.12g\n', radiusDiff);
        fprintf(fid,'image_width=%d\n', BL.width);
        fprintf(fid,'image_height=%d\n', BL.height);
        fprintf(fid,'pair1_camBaseX=%.12g\n', BL.pair1.camBaseX);
        fprintf(fid,'pair1_camBaseY=%.12g\n', BL.pair1.camBaseY);
        fprintf(fid,'pair1_camBaseZ=%.12g\n', BL.pair1.camBaseZ);
        fprintf(fid,'pair1_camPitchDeg=%.12g\n', BL.pair1.camPitch);
        fprintf(fid,'pair2_camBaseX=%.12g\n', BL.pair2.camBaseX);
        fprintf(fid,'pair2_camBaseY=%.12g\n', BL.pair2.camBaseY);
        fprintf(fid,'pair2_camBaseZ=%.12g\n', BL.pair2.camBaseZ);
        fprintf(fid,'pair2_camPitchDeg=%.12g\n', BL.pair2.camPitch);
    end

    function pathOut = getBackendExecutablePath()
        if ispc
            pathOut = 'C:\Users\coope\Downloads\Project_ESD\triangulation\x64\Debug\triangulation.exe';
        else
            thisFile = mfilename('fullpath');
            thisDir = fileparts(thisFile);
            pathOut = fullfile(thisDir, 'triangulation');
        end
    end

    function [I1L, I1R, I2L, I2R, infoLines] = getFourViewsFromBlender(ballXYZ)
        I1L = []; I1R = []; I2L = []; I2R = [];
        infoLines = string.empty(0,1);
    
        if ~ensureClient()
            infoLines = "No TCP client (ensureClient failed).";
            return;
        end
    
        flushClient();
    
        try
            blenderLink(client, BLENDER.width, BLENDER.height, ...
                ballXYZ(1), ballXYZ(2), ballXYZ(3), ...
                BLENDER.ballPitch, BLENDER.ballRoll, BLENDER.ballYaw, BLENDER.ballName);
        catch ME
            infoLines = ["Error moving ball in Blender."; string(ME.message)];
            return;
        end
    
        p1 = BLENDER.pair1;
        p2 = BLENDER.pair2;
    
        try
            I1L = blenderLink(client, BLENDER.width, BLENDER.height, ...
                p1.camBaseX - p1.B_m/2, p1.camBaseY, p1.camBaseZ, ...
                p1.camPitch, p1.camRoll, p1.camYaw, p1.camL);
    
            I1R = blenderLink(client, BLENDER.width, BLENDER.height, ...
                p1.camBaseX + p1.B_m/2, p1.camBaseY, p1.camBaseZ, ...
                p1.camPitch, p1.camRoll, p1.camYaw, p1.camR);
    
            I2L = blenderLink(client, BLENDER.width, BLENDER.height, ...
                p2.camBaseX - p2.B_m/2, p2.camBaseY, p2.camBaseZ, ...
                p2.camPitch, p2.camRoll, p2.camYaw, p2.camL);
    
            I2R = blenderLink(client, BLENDER.width, BLENDER.height, ...
                p2.camBaseX + p2.B_m/2, p2.camBaseY, p2.camBaseZ, ...
                p2.camPitch, p2.camRoll, p2.camYaw, p2.camR);
    
        catch ME
            infoLines = ["Error rendering one or more Blender images."; string(ME.message)];
            I1L = []; I1R = []; I2L = []; I2R = [];
            return;
        end
    
        infoLines = [
            "Blender 4-view OK."
            sprintf('Ball world target = (%.3f, %.3f, %.3f) m', ballXYZ(1), ballXYZ(2), ballXYZ(3))
            sprintf('Pair1 center = (%.2f, %.2f, %.2f) | B = %.2f m | pitch = %.1f deg', ...
                p1.camBaseX, p1.camBaseY, p1.camBaseZ, p1.B_m, p1.camPitch)
            sprintf('Pair2 center = (%.2f, %.2f, %.2f) | B = %.2f m | pitch = %.1f deg', ...
                p2.camBaseX, p2.camBaseY, p2.camBaseZ, p2.B_m, p2.camPitch)
            sprintf('Render size = %d x %d', BLENDER.width, BLENDER.height)
        ];
    end

    function ok = ensureClient()
        ok = true;
        try
            if isempty(client) || ~isvalid(client)
                client = tcpclient(BLENDER.server_ip, BLENDER.server_port, 'Timeout', 20);
            end
        catch ME
            ok = false;
            safeMsg(["Failed to create tcpclient."; string(ME.message); "Make sure Blender server is running."]);
        end
    end

    function flushClient()
        try
            if ~isempty(client) && isvalid(client)
                n = client.NumBytesAvailable;
                if n > 0
                    read(client, n, 'uint8');
                end
            end
        catch
        end
    end

    function safeUpdateOnce()
        try
            updateOnce();
        catch ME
            if isUIAlive()
                safeStatus('MATLAB-side error.');
                safeMsg(["MATLAB-side error:"; string(getReport(ME,'extended','hyperlinks','off'))]);
            end
        end
    end

    function tf = isUIAlive()
        tf = ~isempty(ui) && isvalid(ui) && ...
             ~isempty(statusLabel) && isvalid(statusLabel) && ...
             ~isempty(msg) && isvalid(msg);
    end

    function safeStatus(txt)
        if ~isempty(statusLabel) && isvalid(statusLabel)
            statusLabel.Text = txt;
        end
    end

    function safeMsg(linesIn)
        if ~isempty(msg) && isvalid(msg)
            msg.Value = string(linesIn);
        end
    end

    function setIfValid(obj, prop, val)
        if ~isempty(obj) && isvalid(obj)
            obj.(prop) = val;
        end
    end

    function claIfValid(ax)
        if ~isempty(ax) && isvalid(ax)
            cla(ax);
        end
    end

    function disableButtons()
        setButtonState('off');
    end

    function enableButtons()
        setButtonState('on');
    end

    function setButtonState(stateStr)
        objs = {btnRun, btnClear, btnServe, btnVolley, btnCOR, btnReplay, btnSweep};
        for ii = 1:numel(objs)
            if ~isempty(objs{ii}) && isvalid(objs{ii})
                objs{ii}.Enable = stateStr;
            end
        end
    end

    function releaseBusy()
        busy = false;
        enableButtons();
    end

    function stopReplayTimer()
        try
            if ~isempty(replayTimer) && isvalid(replayTimer)
                stop(replayTimer);
                delete(replayTimer);
            end
        catch
        end
        replayTimer = [];
    end

    function stopLEDTimer()
        try
            if ~isempty(ledTimer) && isvalid(ledTimer)
                stop(ledTimer);
                delete(ledTimer);
            end
        catch
        end
        ledTimer = [];
    end

    function onClose()
        stopReplayTimer();
        stopLEDTimer();
        try
            if ~isempty(client) && isvalid(client)
                clear client
            end
        catch
        end
        if ~isempty(ui) && isvalid(ui)
            delete(ui);
        end
    end

    function showShotFrame(shot, shotName)
        kShow = getPrimaryBounceIndex(shot.bounceIdx, numel(shot.framesL));
        if isempty(kShow)
            kShow = numel(shot.framesL);
        end
    
        claIfValid(axL);
        claIfValid(axR);
    
        if numel(shot.framesL) >= kShow && ~isempty(shot.framesL{kShow})
            imshow(shot.framesL{kShow}, 'Parent', axL);
            title(axL, sprintf('Left | %s | Bounce sample %d', shotName, kShow));
        else
            title(axL, sprintf('Left | %s | no frame', shotName));
        end
    
        if numel(shot.framesR) >= kShow && ~isempty(shot.framesR{kShow})
            imshow(shot.framesR{kShow}, 'Parent', axR);
            title(axR, sprintf('Right | %s | Bounce sample %d', shotName, kShow));
        else
            title(axR, sprintf('Right | %s | no frame', shotName));
        end
    end

    function drawCourtTopDown()
        if ~isvalid(axCourt), return; end
    
        cla(axCourt);
        hold(axCourt,'on');
    
        % Original Blender/world court polygon
        xw = [COURT.polyX COURT.polyX(1)];
        yw = [COURT.polyY COURT.polyY(1)];
    
        % Rotate display 90 deg clockwise:
        % displayX = worldY
        % displayY = -worldX
        plotX = yw;
        plotY = -xw;
    
        plot(axCourt, plotX, plotY, 'Color',[0 0.5 0], 'LineWidth',1.5);
    
        % Court extents in world coords
        xL = min(COURT.polyX);
        xR = max(COURT.polyX);
        yN = min(COURT.polyY);
        yF = max(COURT.polyY);
        yMid = mean(COURT.polyY);
    
        % Singles sidelines (world)
        xSinglesL = -4.115;
        xSinglesR =  4.115;
    
        % Service lines (world)
        yServiceNear = yMid - 6.40;
        yServiceFar  = yMid + 6.40;
    
        % Helper for rotated plotting
        rotPlot = @(xv,yv) deal(yv, -xv);
    
        % Baselines
        [px, py] = rotPlot([xL xR], [yN yN]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.5);
    
        [px, py] = rotPlot([xL xR], [yF yF]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.5);
    
        % Singles sidelines
        [px, py] = rotPlot([xSinglesL xSinglesL], [yN yF]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.2);
    
        [px, py] = rotPlot([xSinglesR xSinglesR], [yN yF]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.2);
    
        % Net
        [px, py] = rotPlot([xL xR], [yMid yMid]);
        plot(axCourt, px, py, 'Color',[0.8 0.2 0.2], 'LineWidth', 1.5);
    
        % Service lines
        [px, py] = rotPlot([xSinglesL xSinglesR], [yServiceNear yServiceNear]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.2);
    
        [px, py] = rotPlot([xSinglesL xSinglesR], [yServiceFar yServiceFar]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.2);
    
        % Center service line
        [px, py] = rotPlot([0 0], [yServiceNear yServiceFar]);
        plot(axCourt, px, py, 'w-', 'LineWidth', 1.2);
    
        % Limits in rotated display coordinates
        xlim(axCourt, [yN-1, yF+1]);
        ylim(axCourt, [-xR-1, -xL+1]);
    
        axis(axCourt,'normal');
        grid(axCourt,'on');
        axCourt.PositionConstraint = 'innerposition';
    
        hold(axCourt,'off');
    end

    function shot = trackPresetForSweep(presetName, duration, numSamples)
        tVec = linspace(0, duration, numSamples);
        shot = captureShotTrajectory(tVec, presetName);
    end

    function onRunSweep()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Running shot sweep...');
        drawnow limitrate;
    
        presetNames = ["serve_default","serve_t","serve_wide","serve_body", ...
                       "volley_default","volley_short","volley_deep","volley_cross"];
    
        try
            resultsTbl = runShotSweep(@trackPresetForSweep, presetNames, defaults.sampleDt, []);
            disp(resultsTbl)
    
            safeMsg(["Sweep complete."; string(evalc('disp(resultsTbl)'))]);
            safeStatus('Shot sweep complete.');
        catch ME
            safeStatus('Shot sweep failed.');
            safeMsg(["Shot sweep failed:"; string(getReport(ME,'extended','hyperlinks','off'))]);
        end
    end

end