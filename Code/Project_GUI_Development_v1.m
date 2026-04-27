function Project_GUI_Development_v1
%% Project GUI + Blender 
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
defaults.replaySlowdown = 0.35;     % smaller = slower replay
defaults.replayInterpDt = 0.02;     % replay timeline spacing
defaults.showReplayFullPath = true; % show faint full path on 2-D court
defaults.outBlinkHz = 10;
defaults.useDenseTrajectorySampling = true;

if defaults.useDenseTrajectorySampling
    defaults.sampleDt = 0.04;
    defaults.serveNumSamples = 26;
    defaults.volleyNumSamples = 23;
else
    defaults.sampleDt = 0.05;
    defaults.serveNumSamples = 21;
    defaults.volleyNumSamples = 19;
end

defaults.serveDuration = 1.00;
defaults.volleyDuration = 0.90;
% Real Blender court-plane height from your measured vertices
WORLD.groundZ = 0.35951;

defaults.validationX = [-3.0, 0.0, 3.0];
defaults.validationY = [-8.0, -4.0, 0.0, 4.0, 8.0];
defaults.validationZ = WORLD.groundZ + [0.5, 1.5, 3.0];
defaults.validationRepeats = 3;

defaults.weakValidationX = [-3.0, 0.0, 3.0];
defaults.weakValidationY = [-8.0, -4.0, 0.0, 4.0, 8.0];
defaults.weakValidationZ = WORLD.groundZ + [0.5, 1.5];
defaults.weakValidationRepeats = 5;

defaults.corPreset = "serve_in_1";
defaults.corShotType = "serve";   % "serve" or "volley"

WEB.enabled = true;
WEB.apiUrl = "http://127.0.0.1:5000/api/shots";   % change to your real backend route
WEB.apiKey = "";                                  % optional
WEB.timeoutSec = 15;
WEB.uploadOnlyFinalDisplayedShot = true;          % set false if you want every preset uploaded
WEB.deviceName = "MATLAB_GUI_1";

if ispc
    defaults.outputRoot = 'C:\Users\coope\Downloads\Project_ESD\tracker_runs';
else
    defaults.outputRoot = fullfile(tempdir, 'tracker_runs');
end

%% -------------------- State --------------------
client = [];
busy = false;
correctionModel = [];

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
COURT.polyX = [-5.485,  5.485,  5.485, -5.485];
COURT.polyY = [-11.885, -11.885, 11.885, 11.885];
COURT.groundZ = WORLD.groundZ;

if ~exist(defaults.outputRoot, 'dir')
    mkdir(defaults.outputRoot);
end

%% -------------------- GUI --------------------
ui = uifigure('Name','Ball Tracking GUI (MATLAB UI + Blender only)', ...
    'Position',[20 20 1650 1020]);
ui.CloseRequestFcn = @(~,~) onClose();

main = uigridlayout(ui,[3 3]);
main.ColumnWidth = {560,'1x','1x'};
main.RowHeight   = {550, 250, '1x'};
main.Padding = [10 10 10 10];
main.RowSpacing = 10;
main.ColumnSpacing = 10;

pCtrl = uipanel(main,'Title','Controls');
pCtrl.Layout.Row = 1;
pCtrl.Layout.Column = 1;

g = uigridlayout(pCtrl,[20 3]);
g.RowHeight = {24,24,24,24,24,30,30,30,30,30,30,30,30,30,30,24,24,24,24,20};
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

btnValidate = uibutton(g,'Text','Run Validation','ButtonPushedFcn',@(~,~) onRunValidation());
btnValidate.Layout.Row = 13; btnValidate.Layout.Column = [1 3];

btnWeakValidate = uibutton(g,'Text','Run Weak-Zone Validation', ...
    'ButtonPushedFcn', @(~,~) onRunWeakValidation());
btnWeakValidate.Layout.Row = 14; 
btnWeakValidate.Layout.Column = [1 3];

btnFitCorrection = uibutton(g,'Text','Fit XYZ Correction', ...
    'ButtonPushedFcn', @(~,~) onFitCorrection());
btnFitCorrection.Layout.Row = 15;
btnFitCorrection.Layout.Column = [1 3];

btnCamAccuracy = uibutton(g, ...
    'Text','Run Camera Accuracy', ...
    'ButtonPushedFcn', @(~,~) onRunCameraPositionAccuracy());
btnCamAccuracy.Layout.Row = 16;
btnCamAccuracy.Layout.Column = [1 3];

shotSummary.Text = 'Shot: --';
shotSummary.Layout.Row = 17;
shotSummary.Layout.Column = [1 3];

uilabel(g,'Text','LED state');
lblLED = uilabel(g,'Text','--');
lblLED.Layout.Row = 18;
lblLED.Layout.Column = [2 3];

statusLabel = uilabel(g,'Text','Idle.');
statusLabel.Layout.Row = 19;
statusLabel.Layout.Column = [1 3];

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

% Replay overlays on 2-D court
hCourtReplayFull  = plot(axCourt, NaN, NaN, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
hCourtReplayTrail = plot(axCourt, NaN, NaN, '-',  'Color', [0 0.45 0.74], 'LineWidth', 2.5);
hCourtReplayHead  = plot(axCourt, NaN, NaN, 'yo', 'MarkerSize', 10, 'LineWidth', 2);

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
    
        % =========================================================
        % CHOOSE MODE HERE:
        %   "one" -> run only the selected serve preset
        %   "all" -> run all serve presets
        % =========================================================
        serveMode = "one";
        selectedServePreset = "serve_out_1";
        % Examples:
        % selectedServePreset = "serve_in_2";
        % selectedServePreset = "serve_out_1";
        % selectedServePreset = "serve_out_2";
        % selectedServePreset = "serve_net_1";
        % selectedServePreset = "serve_net_2";
    
        allServePresets = ["serve_in_1","serve_in_2","serve_out_1","serve_out_2","serve_net_1","serve_net_2"];
    
        switch lower(string(serveMode))
            case "one"
                servePresets = selectedServePreset;
                safeStatus(sprintf('Tracking 1 serve: %s ...', char(selectedServePreset)));
            case "all"
                servePresets = allServePresets;
                safeStatus('Tracking all serves...');
            otherwise
                error('Unknown serveMode: %s. Use "one" or "all".', char(serveMode));
        end
    
        drawnow limitrate;
    
        results = cell(numel(servePresets), 6);
    
        for i = 1:numel(servePresets)
            thisPreset = servePresets(i);
    
            tVec = linspace(0, defaults.serveDuration, defaults.serveNumSamples);
            shot = captureShotTrajectory(tVec, thisPreset);
    
            shot.xyz = shot.xyzTrue;
    
            [shot.decision, shot.bounceXYZ, shot.netXYZ, shot.contactT, shot.netT] = ...
                classifyShotOutcome('serve', shot.t, shot.xyzTrue, COURT);
    
            shot.name = char(thisPreset);
            shot.shotNo = 1;
            
            plotSingleShotCourt(shot, char(thisPreset), COURT, 1);

            shot.restitution = NaN;
    
            results{i,1} = char(thisPreset);
            results{i,2} = shot.bounceXYZ(1);
            results{i,3} = shot.bounceXYZ(2);
            results{i,4} = shot.bounceXYZ(3);
            results{i,5} = char(shot.decision);
            results{i,6} = nnz(shot.valid);
    
            shot.name = char(thisPreset);
            shot.shotNo = i;

            plotSingleShotCourt(shot, char(thisPreset), COURT, i);
    
            % Keep the most recently run shot for display/replay
            lastShot = shot;
    
            try
                if ~WEB.uploadOnlyFinalDisplayedShot || i == numel(servePresets)
                    uploadLastShotToWebsite(lastShot, "serve", WEB);
                end
            catch ME
                warnMsg = char(ME.message);
                warning('%s', ['Upload failed: ' warnMsg]);
            end
    
            trajXYZ = shot.xyzTrue;
            trajT   = shot.t;
    
            hTrue.XData = shot.xyzTrue(:,1);
            hTrue.YData = shot.xyzTrue(:,2);
            hTrue.ZData = shot.xyzTrue(:,3);
    
            updateCourtBounceMarkers(shot);
            updateShotReadouts(shot);
            updateLEDFromDecision(shot.decision);
            update3DPlot();
            autoScale3D();
    
            lblShotType.Text = 'Shot type: Serve';
            lblDecision.Text = sprintf('Decision: %s', char(shot.decision));
            lblCOR.Text = 'Restitution: skipped';
            lblLED.Text = char(shot.decision);
    
            showShotFrame(shot, 'Serve');
        end
    
        T = cell2table(results, ...
            'VariableNames', {'Preset','BounceX','BounceY','BounceZ','Decision','ValidFrames'});
    
        disp('=== SERVE RESULT ===');
        disp(T)
    
        if numel(servePresets) == 1
            shotSummary.Text = sprintf('Tracked 1 serve: %s', char(servePresets));
            safeMsg(["Single serve complete."; string(evalc('disp(T)'))]);
            safeStatus('Single serve complete.');
        else
            shotSummary.Text = sprintf('Tracked %d serves', numel(servePresets));
            safeMsg(["Serve sweep complete."; string(evalc('disp(T)'))]);
            safeStatus('Serve sweep complete.');
        end
    end

    function onTrackVolley()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Tracking 5 volleys...');
        drawnow limitrate;
    
        volleyPresets = ["volley_in_1","volley_in_2","volley_out_1","volley_out_2","volley_net_1","volley_net_2"];
        results = cell(numel(volleyPresets), 6);
    
        for i = 1:numel(volleyPresets)
            tVec = linspace(0, defaults.volleyDuration, defaults.volleyNumSamples);
            shot = captureShotTrajectory(tVec, volleyPresets(i));
    
            shot.xyz = shot.xyzTrue;

            [shot.decision, shot.bounceXYZ, shot.netXYZ, shot.contactT, shot.netT] = ...
                classifyShotOutcome('volley', shot.t, shot.xyzTrue, COURT);
            
            shot.restitution = NaN;
    
            results{i,1} = char(volleyPresets(i));
            results{i,2} = shot.bounceXYZ(1);
            results{i,3} = shot.bounceXYZ(2);
            results{i,4} = shot.bounceXYZ(3);
            results{i,5} = char(shot.decision);
            results{i,6} = nnz(shot.valid);
    
            shot.name = char(volleyPresets(i));
            shot.shotNo = i;

            plotSingleShotCourt(shot, char(volleyPresets(i)), COURT, i);

            if i == numel(volleyPresets)
                lastShot = shot;
            
                try
                    uploadLastShotToWebsite(lastShot, "volley", WEB);
                catch ME
                    msg = char(ME.message);
                    warning('%s', ['Upload failed: ' msg]);
                end

                trajXYZ = shot.xyzTrue;
                trajT   = shot.t;
    
                hTrue.XData = shot.xyzTrue(:,1);
                hTrue.YData = shot.xyzTrue(:,2);
                hTrue.ZData = shot.xyzTrue(:,3);
    
                updateCourtBounceMarkers(shot);
                updateShotReadouts(shot);
                updateLEDFromDecision(shot.decision);
                update3DPlot();
                autoScale3D();
    
                lblShotType.Text = 'Shot type: Volley';
                lblDecision.Text = sprintf('Decision: %s', char(shot.decision));
                lblCOR.Text = 'Restitution: skipped';
                lblLED.Text = char(shot.decision);
    
                showShotFrame(shot, 'Volley');
            end
        end
    
        T = cell2table(results, ...
            'VariableNames', {'Preset','BounceX','BounceY','BounceZ','Decision','ValidFrames'});
    
        disp('=== VOLLEY RESULTS ===');
        disp(T)
    
        shotSummary.Text = sprintf('Tracked %d volleys', numel(volleyPresets));
        safeMsg(["Volley sweep complete."; string(evalc('disp(T)'))]);
        safeStatus('Volley sweep complete.');
    end

    function onComputeCOR()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Running dedicated COR shot...');
        drawnow limitrate;
    
        try
            presetName = defaults.corPreset;
            shotType   = defaults.corShotType;
    
            if strcmpi(shotType, "serve")
                tVec = linspace(0, defaults.serveDuration, defaults.serveNumSamples);
            else
                tVec = linspace(0, defaults.volleyDuration, defaults.volleyNumSamples);
            end
    
            shot = captureShotTrajectory(tVec, presetName);
            shot.xyz = shot.xyzTrue;
    
            [shot.decision, shot.bounceXYZ, shot.netXYZ, shot.contactT, shot.netT] = ...
                classifyShotOutcome(shotType, shot.t, shot.xyzTrue, COURT);
    
            % COR only makes sense for a true bouncing shot, not NET
            if strcmpi(shot.decision, "NET")
                lblCOR.Text = 'Restitution: NET shot';
                safeStatus('COR skipped: selected preset hits the net.');
                safeMsg([
                    "COR skipped.";
                    "The selected COR preset is classified as NET.";
                    "Choose a bouncing preset like serve_in_1 or volley_in_1."
                ]);
                return;
            end
    
            outCOR = analyze_court_restitution(shot.t, shot.xyzTrue, ...
                'GroundZ', WORLD.groundZ, ...
                'BounceIdx', shot.bounceIdx, ...
                'PlotResults', true, ...
                'Label', sprintf('Court Restitution Analysis - %s', presetName));
    
            lastShot = shot;

            trajXYZ = shot.xyzTrue;
            trajT   = shot.t;
    
            hTrue.XData = shot.xyzTrue(:,1);
            hTrue.YData = shot.xyzTrue(:,2);
            hTrue.ZData = shot.xyzTrue(:,3);
    
            updateCourtBounceMarkers(shot);
            updateShotReadouts(shot);
            updateLEDFromDecision(shot.decision);
            update3DPlot();
            autoScale3D();
    
            lblShotType.Text = sprintf('Shot type: %s', char(shotType));
            lblDecision.Text = sprintf('Decision: %s', char(shot.decision));
            lblLED.Text = char(shot.decision);
    
            showShotFrame(shot, sprintf('COR - %s', char(presetName)));
    
            if ~outCOR.ok
                lblCOR.Text = 'Restitution: could not compute';
                safeStatus('Could not compute restitution.');
                safeMsg(outCOR.message);
                return;
            end
    
            lastShot.restitution = outCOR.meanCOR;
            lblCOR.Text = sprintf('Restitution: %.4f', outCOR.meanCOR);
    
            safeStatus(sprintf('COR computed from preset %s.', char(presetName)));
            safeMsg([
                "COR analysis complete.";
                sprintf('Preset = %s', char(presetName));
                sprintf('Mean COR = %.4f', outCOR.meanCOR);
                sprintf('Std COR = %.4f', outCOR.stdCOR);
                sprintf('Valid bounces = %d', numel(outCOR.corValues))
            ]);
    
            if ~isfield(lastShot, 'name') || isempty(lastShot.name)
                lastShot.name = "cor_result";
            end
            
            lastShot.shotNo = 1;

             try
                uploadLastShotToWebsite(lastShot, "cor", WEB);
            catch ME
                msg = char(ME.message);
                warning('%s', ['Upload failed: ' msg]);
             end

        catch ME
            lblCOR.Text = 'Restitution: error';
            safeStatus('COR analysis failed.');
            safeMsg(string(getReport(ME,'extended','hyperlinks','off')));
        end
    end

    function onInstantReplay()
        if isempty(lastShot) || isempty(lastShot.t) || ~any(lastShot.valid)
            safeStatus('No saved shot for replay.');
            return;
        end
    
        validMask = lastShot.valid(:) ...
            & all(isfinite(lastShot.xyzMeas),2) ...
            & isfinite(lastShot.t(:));
    
        if nnz(validMask) < 2
            safeStatus('Not enough valid replay samples.');
            return;
        end
    
        stopReplayTimer();
    
        % Base measured trajectory used for replay
        tBase = lastShot.t(validMask);
        xyzBase = lastShot.xyzMeas(validMask,:);
        framesLBase = lastShot.framesL(validMask);
        framesRBase = lastShot.framesR(validMask);
    
        % Make sure times are strictly increasing
        [tBase, uniqIdx] = unique(tBase, 'stable');
        xyzBase = xyzBase(uniqIdx,:);
        framesLBase = framesLBase(uniqIdx);
        framesRBase = framesRBase(uniqIdx);
    
        if numel(tBase) < 2
            safeStatus('Not enough replay samples after filtering.');
            return;
        end
    
        % Slower/longer replay timeline
        shotDuration = tBase(end) - tBase(1);
        replayDuration = shotDuration / max(0.05, defaults.replaySlowdown);
        replayDt = max(defaults.replayInterpDt, 1/defaults.maxReplayFPS);
    
        replayT = (0:replayDt:replayDuration).';
        tMapped = tBase(1) + replayT * defaults.replaySlowdown;
    
        % Interpolate XYZ for smoother replay
        xReplay = interp1(tBase, xyzBase(:,1), tMapped, 'pchip');
        yReplay = interp1(tBase, xyzBase(:,2), tMapped, 'pchip');
        zReplay = interp1(tBase, xyzBase(:,3), tMapped, 'pchip');
        xyzReplay = [xReplay yReplay zReplay];
    
        % Map each replay instant to nearest original frame index
        frameIdxMap = interp1(tBase, 1:numel(tBase), tMapped, 'nearest', 'extrap');
        frameIdxMap = max(1, min(numel(tBase), round(frameIdxMap)));
    
        % Pre-show the whole replay path on the 2-D court
        if defaults.showReplayFullPath
            hCourtReplayFull.XData = xyzReplay(:,2);   % display X = world Y
            hCourtReplayFull.YData = -xyzReplay(:,1);  % display Y = -world X
        else
            hCourtReplayFull.XData = NaN;
            hCourtReplayFull.YData = NaN;
        end
    
        hCourtReplayTrail.XData = NaN;
        hCourtReplayTrail.YData = NaN;
        hCourtReplayHead.XData = NaN;
        hCourtReplayHead.YData = NaN;
    
        % Optional: hide static "current point" while replay is running
        hCourtCurrent.XData = NaN;
        hCourtCurrent.YData = NaN;
    
        replayIdx = 1;
        N = size(xyzReplay,1);
        period = max(0.03, 1/defaults.maxReplayFPS);
    
        replayTimer = timer( ...
            'ExecutionMode','fixedRate', ...
            'Period', period, ...
            'TimerFcn', @doReplayStep, ...
            'BusyMode','drop');
    
        safeStatus(sprintf('Instant replay running... (%.2fx slower)', 1/defaults.replaySlowdown));
        start(replayTimer);
    
        function doReplayStep(~,~)
            if ~isUIAlive()
                stopReplayTimer();
                return;
            end
    
            if replayIdx > N
                stopReplayTimer();
                safeStatus('Replay finished.');
    
                % Leave the full path visible, but clear animated overlays
                hCourtReplayTrail.XData = NaN;
                hCourtReplayTrail.YData = NaN;
                hCourtReplayHead.XData = NaN;
                hCourtReplayHead.YData = NaN;
                return;
            end
    
            xyzNow = xyzReplay(replayIdx,:);
    
            % 3-D replay
            hPt.XData = xyzNow(1);
            hPt.YData = xyzNow(2);
            hPt.ZData = xyzNow(3);
    
            hTraj.XData = xyzReplay(1:replayIdx,1);
            hTraj.YData = xyzReplay(1:replayIdx,2);
            hTraj.ZData = xyzReplay(1:replayIdx,3);
    
            % Bounce markers up to current mapped original sample
            origIdxNow = frameIdxMap(replayIdx);
            bounceShown = lastShot.bounceIdx(lastShot.bounceIdx <= origIdxNow);
            if ~isempty(bounceShown)
                bp = lastShot.xyzMeas(bounceShown,:);
                bp = bp(all(isfinite(bp),2),:);
                if ~isempty(bp)
                    hBounce.XData = bp(:,1);
                    hBounce.YData = bp(:,2);
                    hBounce.ZData = bp(:,3);
                end
            else
                hBounce.XData = NaN;
                hBounce.YData = NaN;
                hBounce.ZData = NaN;
            end
    
            % 2-D court replay overlays
            hCourtReplayTrail.XData = xyzReplay(1:replayIdx,2);
            hCourtReplayTrail.YData = -xyzReplay(1:replayIdx,1);
    
            hCourtReplayHead.XData = xyzNow(2);
            hCourtReplayHead.YData = -xyzNow(1);
    
            % Replay frames from nearest original sample
            kFrame = frameIdxMap(replayIdx);
    
            if kFrame <= numel(framesLBase) && ~isempty(framesLBase{kFrame})
                imshow(framesLBase{kFrame}, 'Parent', axL);
                title(axL, sprintf('Replay Left | %s | t = %.2f s', lastShot.name, tMapped(replayIdx)));
            end
    
            if kFrame <= numel(framesRBase) && ~isempty(framesRBase{kFrame})
                imshow(framesRBase{kFrame}, 'Parent', axR);
                title(axR, sprintf('Replay Right | %s | t = %.2f s', lastShot.name, tMapped(replayIdx)));
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
            'bounceXYZ', [NaN NaN NaN], ...
            'restitution', NaN, ...
            'framesL', {cell(numel(tVec),1)}, ...
            'framesR', {cell(numel(tVec),1)}, ...
            'contactXYZ', [NaN NaN NaN], ...
            'contactT', NaN, ...
            'netXYZ', [NaN NaN NaN], ...
            'netT', NaN, ...
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
                shot.valid(k) = true;
                shot.framesL{k} = pt.IL;
                shot.framesR{k} = pt.IR;
                shot.selectedPair(k) = pt.result.pair_index;
            end
        end

        % Always use the provided true XYZ path for bounce/call logic.
        shot.xyz = shot.xyzTrue;

        [contactXYZ, contactT, idxBefore] = findCourtContact(shot.t, shot.xyzTrue, WORLD.groundZ);
        
        shot.contactXYZ = contactXYZ;
        shot.contactT = contactT;
        shot.bounceIdx = idxBefore;
        
        if all(isfinite(contactXYZ))
            shot.bounceXYZ = contactXYZ;
        else
            shot.bounceXYZ = [NaN NaN NaN];
        end
                
        fprintf('Preset %s | bounceIdx = ', char(shotPreset));
        if isempty(shot.bounceIdx)
            fprintf('[]\n');
        else
            fprintf('%d\n', shot.bounceIdx);
        end
        
        if all(isfinite(shot.bounceXYZ))
            fprintf('Bounce XYZ = [%.4f %.4f %.4f]\n', ...
                shot.bounceXYZ(1), shot.bounceXYZ(2), shot.bounceXYZ(3));
        else
            fprintf('Bounce XYZ = [NaN NaN NaN]\n');
        end
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
        x = p(1); y = p(2);
        tol = 1e-6;

        switch lower(string(shotType))
            case "serve"
                inX = abs(x) <= courtGeom.singlesHalfWidth + tol;
                inY = (y >= courtGeom.serviceBoxYMin - tol) && (y <= courtGeom.serviceBoxYMax + tol);
                if inX && inY
                    decision = "IN";
                else
                    decision = "OUT";
                end
            otherwise
                inCourt = inpolygon(x, y, courtGeom.polyX, courtGeom.polyY);
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
    
        % Tennis ball / court line dimensions in meters.
        % Standard tennis ball diameter is about 6.54 cm to 6.86 cm,
        % so radius is approximately 0.0335 m.
        geom.ballRadius = 0.0335;
    
        % Typical court line width is about 5 cm.
        % If the court boundary is drawn/measured at the center of the painted line,
        % add half the line width to the ball radius.
        geom.lineWidth = 0.05;
    
        % Line-touch rule:
        % A ball is IN if any part of the ball touches the line.
        geom.lineTouchTol = geom.ballRadius + geom.lineWidth/2;
    
        % volley target area on far side
        geom.farCourtYMin = 0.00;
        geom.farCourtYMax = geom.courtYMax;
    
        % net model
        geom.netY = 0.00;
        geom.netHeight = 0.914;   % simple flat center-height model
    
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
            bp = shot.bounceXYZ;
            lblBounce.Text = sprintf('Bounce: idx %d | XYZ = [%.3f %.3f %.3f]', ...
                shot.bounceIdx, bp(1), bp(2), bp(3));
            hBounce.XData = bp(1);
            hBounce.YData = bp(2);
            hBounce.ZData = bp(3);
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

            case "NET"
                ledGreen.Color = [0.2 0.2 0.2];
                ledRed.Color   = [1 1 0];
                lblLED.Text = 'NET: yellow solid';

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
            hCourtBounce.XData = shot.bounceXYZ(2);
            hCourtBounce.YData = -shot.bounceXYZ(1);
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
            'bounceXYZ', [NaN NaN NaN], ...
            'restitution', NaN, ...
            'framesL', {{}}, ...
            'framesR', {{}}, ...
            'contactXYZ', [NaN NaN NaN], ...
            'contactT', NaN, ...
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

        hCourtReplayFull.XData = NaN;
        hCourtReplayFull.YData = NaN;
        
        hCourtReplayTrail.XData = NaN;
        hCourtReplayTrail.YData = NaN;
        
        hCourtReplayHead.XData = NaN;
        hCourtReplayHead.YData = NaN;

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
        objs = {btnRun, btnClear, btnServe, btnVolley, btnCOR, btnReplay, btnSweep, btnValidate, btnWeakValidate, btnFitCorrection, btnCamAccuracy};
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

    if exist('hCourtReplayTrail','var') && isgraphics(hCourtReplayTrail)
        hCourtReplayTrail.XData = NaN;
        hCourtReplayTrail.YData = NaN;
    end
    if exist('hCourtReplayHead','var') && isgraphics(hCourtReplayHead)
        hCourtReplayHead.XData = NaN;
        hCourtReplayHead.YData = NaN;
    end
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

   function onRunValidation()
        disp(['busy = ' num2str(busy)])
        disp(['isUIAlive = ' num2str(isUIAlive())])
    
        if busy || ~isUIAlive() 
            disp('RETURNING EARLY FROM onRunValidation');
            return; 
        end
    
        disp('ENTERED onRunValidation')
        safeStatus('ENTERED onRunValidation');
        drawnow;
    
        safeStatus('Running full validation...');
        drawnow limitrate;
    
        try
            disp('START onRunValidation')

            staticTbl = runStaticValidation();
            disp('DONE runStaticValidation')
            disp(size(staticTbl))
            
            shotTbl = runShotSweep(@trackPresetForSweep, ...
                ["serve_default","serve_t","serve_wide","serve_body", ...
                 "volley_default","volley_short","volley_deep","volley_cross"], ...
                defaults.sampleDt, []);
            disp('DONE runShotSweep')
            disp(size(shotTbl))
            
            staticSummary = summarizeStaticValidation(staticTbl);
            disp('DONE summarizeStaticValidation')
            
            shotSummaryTbl = summarizeShotValidation(shotTbl);
            disp('DONE summarizeShotValidation')
            
            disp('=== STATIC VALIDATION ===')
            disp(staticTbl)
            
            disp('=== SHOT VALIDATION ===')
            disp(shotTbl)
            
            disp('=== STATIC SUMMARY ===')
            disp(staticSummary)
            
            disp('=== SHOT SUMMARY ===')
            disp(shotSummaryTbl)

            disp('ABOUT TO PLOT error map')
            plotStaticErrorMap(staticTbl);
            
            disp('ABOUT TO PLOT error by height')
            plotStaticErrorByHeight(staticTbl);
            
            disp('ABOUT TO PLOT failure map')
            plotStaticFailureMap(staticTbl);
            
            disp('FINISHED onRunValidation')
        catch ME
            disp('=== VALIDATION FAILED ===');
            disp(getReport(ME,'extended','hyperlinks','off'));
            safeStatus('Validation failed.');
            safeMsg(["Validation failed:"; string(getReport(ME,'extended','hyperlinks','off'))]);
            rethrow(ME);
        end
    end

    function T = runStaticValidation()
        Xs = defaults.validationX;
        Ys = defaults.validationY;
        Zs = defaults.validationZ;
        nRep = defaults.validationRepeats;

        rows = [];

        for iz = 1:numel(Zs)
            for iy = 1:numel(Ys)
                for ix = 1:numel(Xs)
                    xyzTrue = [Xs(ix), Ys(iy), Zs(iz)];

                    for ir = 1:nRep
                        pt = renderTrackAtBall(xyzTrue, false);
                        drawnow;

                        if pt.ok
                            dx = pt.X - xyzTrue(1);
                            dy = pt.Y - xyzTrue(2);
                            dz = pt.Z - xyzTrue(3);
                            e  = hypot(hypot(dx,dy),dz);

                            pairIdx = NaN;
                            if isfield(pt,'result') && isfield(pt.result,'pair_index')
                                pairIdx = pt.result.pair_index;
                            end

                            rows = [rows; ...
                                xyzTrue, ...
                                pt.X, pt.Y, pt.Z, ...
                                dx, dy, dz, e, ...
                                pairIdx, ir]; %#ok<AGROW>
                        else
                            rows = [rows; ...
                                xyzTrue, ...
                                NaN, NaN, NaN, ...
                                NaN, NaN, NaN, NaN, ...
                                NaN, ir]; %#ok<AGROW>
                        end
                    end
                end
            end
        end

        T = array2table(rows, 'VariableNames', { ...
            'Xtrue','Ytrue','Ztrue', ...
            'Xmeas','Ymeas','Zmeas', ...
            'dX','dY','dZ','e', ...
            'pairIdx','repeatIdx'});
    end

    function S = summarizeStaticValidation(T)
        good = isfinite(T.e);
        Tg = T(good,:);

        if isempty(Tg)
            S = table();
            return;
        end

        meanDx = mean(Tg.dX,'omitnan');
        meanDy = mean(Tg.dY,'omitnan');
        meanDz = mean(Tg.dZ,'omitnan');
        meanE  = mean(Tg.e,'omitnan');
        maxE   = max(Tg.e,[],'omitnan');
        rmseE  = sqrt(mean(Tg.e.^2,'omitnan'));

        pair1Count = sum(Tg.pairIdx == 1);
        pair2Count = sum(Tg.pairIdx == 2);

        S = table(meanDx, meanDy, meanDz, meanE, maxE, rmseE, pair1Count, pair2Count);
    end

    function S = summarizeShotValidation(T)
        serveMask  = strcmpi(string(T.type), "serve");
        volleyMask = strcmpi(string(T.type), "volley");

        serveMeanE  = mean(T.meanE(serveMask),'omitnan');
        serveMaxE   = max(T.maxE(serveMask),[],'omitnan');
        serveBounce = mean(T.bounceE(serveMask),'omitnan');

        volleyMeanE  = mean(T.meanE(volleyMask),'omitnan');
        volleyMaxE   = max(T.maxE(volleyMask),[],'omitnan');
        volleyBounce = mean(T.bounceE(volleyMask),'omitnan');

        S = table( ...
            serveMeanE, serveMaxE, serveBounce, ...
            volleyMeanE, volleyMaxE, volleyBounce);
    end

    function plotStaticErrorMap(T)
        good = isfinite(T.e);
        Tg = T(good,:);
    
        if isempty(Tg)
            figure('Name','Static Validation Error Map');
            title('No valid static validation points');
            return;
        end
    
        % Collapse repeats so each true XYZ location appears once
        G = groupsummary(Tg, {'Xtrue','Ytrue','Ztrue'}, 'mean', 'e');
        G.Properties.VariableNames{'mean_e'} = 'meanE';
    
        zLevels = unique(G.Ztrue);
        nZ = numel(zLevels);
    
        f = figure('Name','Static Validation Error Map','Color','w', ...
            'Position',[100 100 1400 420]);
    
        tl = tiledlayout(1, nZ, 'Padding','compact', 'TileSpacing','compact');
    
        for i = 1:nZ
            nexttile;
            hold on;
            grid on;
    
            zNow = zLevels(i);
            Gi = G(abs(G.Ztrue - zNow) < 1e-9,:);
    
            % Draw court in rotated top-down display coordinates
            xw = [COURT.polyX COURT.polyX(1)];
            yw = [COURT.polyY COURT.polyY(1)];
            plotX = yw;
            plotY = -xw;
            plot(plotX, plotY, 'k-', 'LineWidth', 1.2);
    
            % Scatter true locations, colored by mean error
            sc = scatter(Gi.Ytrue, -Gi.Xtrue, 90, Gi.meanE*1000, 'filled', 'MarkerEdgeColor','k');
    
            % Label each point with mm error
            for k = 1:height(Gi)
                text(Gi.Ytrue(k)+0.15, -Gi.Xtrue(k), sprintf('%.1f', Gi.meanE(k)*1000), ...
                    'FontSize',8);
            end
    
            xlabel('Y (m)');
            ylabel('-X (m)');
            title(sprintf('Static Error Map | Z = %.3f m', zNow));
            axis equal;
    
            xlim([min(COURT.polyY)-1, max(COURT.polyY)+1]);
            ylim([-max(COURT.polyX)-1, -min(COURT.polyX)+1]);
    
            cb = colorbar;
            cb.Label.String = 'Mean error (mm)';
    
            hold off;
        end
    
        title(tl, 'Static Validation Error by Court Position');
    end

    function plotStaticErrorByHeight(T)
        good = isfinite(T.e);
        Tg = T(good,:);
    
        if isempty(Tg)
            figure('Name','Static Error by Height');
            title('No valid static validation points');
            return;
        end
    
        G = groupsummary(Tg, 'Ztrue', {'mean','max'}, 'e');
        G.Properties.VariableNames{'mean_e'} = 'meanE';
        G.Properties.VariableNames{'max_e'}  = 'maxE';
    
        figure('Name','Static Error by Height','Color','w', ...
            'Position',[150 150 700 450]);
    
        plot(G.Ztrue, G.meanE*1000, '-o', 'LineWidth', 1.5, 'MarkerSize', 8);
        hold on;
        plot(G.Ztrue, G.maxE*1000, '-s', 'LineWidth', 1.5, 'MarkerSize', 8);
        hold off;
    
        grid on;
        xlabel('True Z (m)');
        ylabel('Error (mm)');
        title('Static Validation Error vs Height');
        legend('Mean error','Max error','Location','best');
    end

    function plotStaticFailureMap(T)
        if isempty(T) || height(T) == 0
            figure('Name','Static Validation Failure Map','Color','w');
            title('No static validation data');
            return;
        end
    
        % Define failures
        if ismember('success', T.Properties.VariableNames)
            failMask = ~T.success;
        elseif ismember('e', T.Properties.VariableNames)
            failMask = ~isfinite(T.e);
        else
            failMask = false(height(T),1);
        end
    
        % Group by true XYZ
        [G, Xv, Yv, Zv] = findgroups(T.Xtrue, T.Ytrue, T.Ztrue);
    
        totalCount = splitapply(@numel, T.Xtrue, G);
        failCount  = splitapply(@sum, double(failMask), G);
        failRate   = failCount ./ max(totalCount, 1);
    
        S = table(Xv, Yv, Zv, totalCount, failCount, failRate, ...
            'VariableNames', {'Xtrue','Ytrue','Ztrue','GroupCount','FailCount','FailRate'});
    
        zVals = unique(S.Ztrue);
        figure('Name','Static Validation Failure Map','Color','w');
        tiledlayout(numel(zVals),1,'Padding','compact','TileSpacing','compact');
    
        for k = 1:numel(zVals)
            z0 = zVals(k);
            Sz = S(S.Ztrue == z0, :);
    
            nexttile;
            scatter(Sz.Xtrue, Sz.Ytrue, 140, Sz.FailRate, 'filled');
            colorbar;
            caxis([0 1]);
            grid on;
            axis equal;
            xlabel('X true');
            ylabel('Y true');
            title(sprintf('Failure Rate at Z = %.3f m', z0));
    
            for i = 1:height(Sz)
                text(Sz.Xtrue(i), Sz.Ytrue(i), sprintf(' %.2f', Sz.FailRate(i)), ...
                    'FontSize', 8, ...
                    'HorizontalAlignment', 'left', ...
                    'VerticalAlignment', 'middle');
            end
        end
    end

    function onRunWeakValidation()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Running weak-zone validation...');
        drawnow limitrate;
    
        try
            weakTbl = runWeakZoneValidation();
            weakSummary = summarizeStaticValidation(weakTbl);
    
            disp('=== WEAK-ZONE VALIDATION ===')
            disp(weakTbl)
    
            disp('=== WEAK-ZONE SUMMARY ===')
            disp(weakSummary)
    
            plotStaticErrorMap(weakTbl);
            plotStaticErrorByHeight(weakTbl);
            plotStaticFailureMap(weakTbl);
    
            safeMsg([ ...
                "Weak-zone validation complete."; ...
                " "; ...
                "WEAK-ZONE SUMMARY:"; ...
                string(evalc('disp(weakSummary)')) ...
            ]);
    
            safeStatus('Weak-zone validation complete.');
        catch ME
            safeStatus('Weak-zone validation failed.');
            safeMsg(["Weak-zone validation failed:"; ...
                string(getReport(ME,'extended','hyperlinks','off'))]);
        end
    end
    
    function T = runWeakZoneValidation()
        Xs = defaults.weakValidationX;
        Ys = defaults.weakValidationY;
        Zs = defaults.weakValidationZ;
        nRep = defaults.weakValidationRepeats;
    
        rows = [];
    
        for iz = 1:numel(Zs)
            for iy = 1:numel(Ys)
                for ix = 1:numel(Xs)
                    xyzTrue = [Xs(ix), Ys(iy), Zs(iz)];
    
                    for ir = 1:nRep
                        pt = renderTrackAtBall(xyzTrue, false);
                        drawnow;
                        
                        if pt.ok
                            dx = pt.X - xyzTrue(1);
                            dy = pt.Y - xyzTrue(2);
                            dz = pt.Z - xyzTrue(3);
                            e  = hypot(hypot(dx,dy),dz);
    
                            pairIdx = NaN;
                            if isfield(pt,'result') && isfield(pt.result,'pair_index')
                                pairIdx = pt.result.pair_index;
                            end
    
                            rows = [rows; ...
                                xyzTrue, ...
                                pt.X, pt.Y, pt.Z, ...
                                dx, dy, dz, e, ...
                                pairIdx, ir]; %#ok<AGROW>
                        else
                            rows = [rows; ...
                                xyzTrue, ...
                                NaN, NaN, NaN, ...
                                NaN, NaN, NaN, NaN, ...
                                NaN, ir]; %#ok<AGROW>
                        end
                    end
                end
            end
        end
    
        T = array2table(rows, 'VariableNames', { ...
            'Xtrue','Ytrue','Ztrue', ...
            'Xmeas','Ymeas','Zmeas', ...
            'dX','dY','dZ','e', ...
            'pairIdx','repeatIdx'});
    end
    
    function onFitCorrection()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Fitting XYZ correction...');
        drawnow limitrate;
    
        try
            staticTbl = runStaticValidation();
            correctionModel = fitAffineCorrection(staticTbl);
    
            disp('=== XYZ CORRECTION MODEL ===')
            disp(correctionModel)
    
            safeMsg([ ...
                "XYZ correction fitted."; ...
                " "; ...
                "Ax = " + mat2str(correctionModel.Ax,4); ...
                "Ay = " + mat2str(correctionModel.Ay,4); ...
                "Az = " + mat2str(correctionModel.Az,4) ...
            ]);
    
            safeStatus('XYZ correction fitted.');
        catch ME
            safeStatus('XYZ correction fit failed.');
            safeMsg(["XYZ correction fit failed:"; ...
                string(getReport(ME,'extended','hyperlinks','off'))]);
        end
    end
    
    function model = fitAffineCorrection(T)
        good = isfinite(T.Xmeas) & isfinite(T.Ymeas) & isfinite(T.Zmeas);
        Tg = T(good,:);
    
        M = [ones(height(Tg),1), Tg.Xmeas, Tg.Ymeas, Tg.Zmeas];
    
        model.Ax = M \ Tg.Xtrue;
        model.Ay = M \ Tg.Ytrue;
        model.Az = M \ Tg.Ztrue;
        model.ok = true;
    end
    
    function xyzCorr = applyAffineCorrection(xyzMeas, model)
        if isempty(model) || ~isfield(model,'ok') || ~model.ok
            xyzCorr = xyzMeas;
            return;
        end
    
        M = [1, xyzMeas(1), xyzMeas(2), xyzMeas(3)];
        xyzCorr = [ ...
            M * model.Ax, ...
            M * model.Ay, ...
            M * model.Az ];
    end

    function pt = renderTrackAtBall_withBL(ballXYZ, BL_override)
        BL_save = BLENDER;
        BLENDER = BL_override;
    
        try
            pt = renderTrackAtBall(ballXYZ, false);
        catch ME
            BLENDER = BL_save;
            rethrow(ME);
        end
    
        BLENDER = BL_save;
    end

    function onRunCameraPositionAccuracy()
        if busy || ~isUIAlive(), return; end
    
        safeStatus('Running camera-motion accuracy analysis...');
        drawnow limitrate;
    
        try
            % Use a ball point that is already known to track reliably
            ballXYZ = [0, -7.5, WORLD.groundZ + 1.5];
    
            trackFcn = @(ballXYZ_in, BL_in) renderTrackAtBall_withBL(ballXYZ_in, BL_in);
    
            windLevels = ["light","medium","strong"];
            summaryRows = cell(numel(windLevels), 6);
    
            for i = 1:numel(windLevels)
                level = windLevels(i);
                W = getWindProfile(level);
    
                safeStatus(sprintf('Running %s wind test (%d of %d)...', level, i, numel(windLevels)));
                drawnow limitrate;
    
                [Tcam, Scam, ~] = run_camera_position_accuracy( ...
                    ballXYZ, ...
                    BLENDER, ...
                    trackFcn, ...
                    'NumTrials', 300, ...
                    'Mode', 'gaussian', ...
                    'TranslateSigma', W.TranslateSigma, ...
                    'RotateSigmaDeg', W.RotateSigmaDeg, ...
                    'TranslateMax', W.TranslateMax, ...
                    'RotateMaxDeg', W.RotateMaxDeg, ...
                    'PerturbationType', 'rigid_pair', ...
                    'PlotResults', true, ...
                    'PlotMode', 'scatterXY', ...
                    'StatusFcn', @safeStatus, ...
                    'MsgFcn', @safeMsg, ...
                    'Label', "Camera Motion Validation - " + level);
    
                fprintf('\n=== %s WIND TABLE ===\n', upper(level));
                disp(Tcam);
    
                fprintf('\n=== %s WIND SUMMARY ===\n', upper(level));
                disp(Scam);
    
                summaryRows{i,1} = char(level);
                summaryRows{i,2} = Scam.validFrac;
                summaryRows{i,3} = Scam.meanE;
                summaryRows{i,4} = Scam.rmseE;
                summaryRows{i,5} = Scam.p95E;
                summaryRows{i,6} = Scam.maxE;
            end
    
            windSummary = cell2table(summaryRows, ...
                'VariableNames', {'WindLevel','ValidFrac','MeanE','RMSE','P95E','MaxE'});
    
            disp('=== ALL WIND LEVELS SUMMARY ===');
            disp(windSummary);
    
            safeMsg([ ...
                "Camera motion validation complete."; ...
                " "; ...
                "Summary by wind level:"; ...
                string(evalc('disp(windSummary)')) ...
            ]);
    
            safeStatus('Camera-motion accuracy complete.');
    
        catch ME
            safeStatus('Camera-motion accuracy failed.');
            safeMsg(["Camera-motion accuracy failed:"; ...
                string(getReport(ME,'extended','hyperlinks','off'))]);
        end
    end

    function W = getWindProfile(level)
        switch lower(string(level))
            case "light"
                W.TranslateSigma = [0.0015 0.0015 0.0008];
                W.RotateSigmaDeg = [0.015 0.008 0.015];
                W.TranslateMax   = [0.0045 0.0045 0.0024];
                W.RotateMaxDeg   = [0.045 0.024 0.045];
    
            case "medium"
                W.TranslateSigma = [0.003 0.003 0.0015];
                W.RotateSigmaDeg = [0.03 0.015 0.03];
                W.TranslateMax   = [0.009 0.009 0.0045];
                W.RotateMaxDeg   = [0.09 0.045 0.09];
    
            case "strong"
                W.TranslateSigma = [0.005 0.005 0.0025];
                W.RotateSigmaDeg = [0.05 0.025 0.05];
                W.TranslateMax   = [0.015 0.015 0.0075];
                W.RotateMaxDeg   = [0.15 0.075 0.15];
    
            otherwise
                error('Unknown wind level: %s', level);
        end
    end

    function bounceIdx = findBounceIndexFromTrue(xyzTrue)
    bounceIdx = [];
        if isempty(xyzTrue) || size(xyzTrue,1) < 3
            return;
        end
    
        z = xyzTrue(:,3);
        if ~all(isfinite(z))
            return;
        end
    
        % First local minimum
        for k = 2:numel(z)-1
            if z(k) <= z(k-1) && z(k) <= z(k+1)
                bounceIdx = k;
                return;
            end
        end
    
        % Fallback
        [~, bounceIdx] = min(z);
    end

    function decision = classifyShotFromBounce(shotType, bounceXYZ, courtGeom)
        decision = "UNKNOWN";
    
        if isempty(bounceXYZ) || ~all(isfinite(bounceXYZ(1:2)))
            return;
        end
    
        x = bounceXYZ(1);
        y = bounceXYZ(2);
    
        % Use ball/line tolerance so that line contact counts as IN.
        if isfield(courtGeom, 'lineTouchTol')
            lineTol = courtGeom.lineTouchTol;
        elseif isfield(courtGeom, 'ballRadius')
            lineTol = courtGeom.ballRadius;
        else
            lineTol = 0.0335;
        end
    
        % Use the same court geometry that is being plotted.
        % This makes the visual court boundary and IN/OUT logic match.
        xMin = min(courtGeom.polyX);
        xMax = max(courtGeom.polyX);
        yMin = min(courtGeom.polyY);
        yMax = max(courtGeom.polyY);
    
        % Expand the court boundary outward by the ball/line tolerance.
        % If the ball center is just outside the line, but the ball edge
        % still touches the line, it counts as IN.
        inX = (x >= xMin - lineTol) && (x <= xMax + lineTol);
        inY = (y >= yMin - lineTol) && (y <= yMax + lineTol);
    
        if inX && inY
            decision = "IN";
        else
            decision = "OUT";
        end
    end

    function dMin = distancePointToPolygonEdges(x, y, polyX, polyY)
        dMin = inf;
    
        n = numel(polyX);
    
        for k = 1:n
            k2 = k + 1;
    
            if k2 > n
                k2 = 1;
            end
    
            x1 = polyX(k);
            y1 = polyY(k);
            x2 = polyX(k2);
            y2 = polyY(k2);
    
            d = distancePointToSegment(x, y, x1, y1, x2, y2);
    
            if d < dMin
                dMin = d;
            end
        end
    end

    function d = distancePointToSegment(px, py, x1, y1, x2, y2)
        vx = x2 - x1;
        vy = y2 - y1;
    
        wx = px - x1;
        wy = py - y1;
    
        c1 = wx*vx + wy*vy;
        c2 = vx*vx + vy*vy;
    
        if c2 <= eps
            d = hypot(px - x1, py - y1);
            return;
        end
    
        t = c1 / c2;
        t = max(0, min(1, t));
    
        projX = x1 + t*vx;
        projY = y1 + t*vy;
    
        d = hypot(px - projX, py - projY);
    end

    function preset = getShotPreset(name, groundZ)
        preset.g = 9.81;
        preset.eApprox = 0.72;
    
        switch lower(char(name))
    
            % ---------------- SERVES: 2 IN ----------------
            case 'serve_in_1'
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.6;
                preset.vx = 0.2;  preset.vy = 18.0;  preset.vz = 1.8;
                preset.bounceT = 0.80;
    
            case 'serve_in_2'
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.6;
                preset.vx = -0.8; preset.vy = 18.2;  preset.vz = 1.9;
                preset.bounceT = 0.79;
    
            % ---------------- SERVES: 2 OUT ----------------
            case 'serve_out_1'   % too wide
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.6;
                preset.vx = 7.2;  preset.vy = 18.0;  preset.vz = 1.8;
                preset.bounceT = 0.80;
    
            case 'serve_out_2'   % too deep
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.6;
                preset.vx = 0.1;  preset.vy = 27.5;  preset.vz = 1.8;
                preset.bounceT = 0.86;
    
            % ---------------- SERVES: 2 NET ----------------
            case 'serve_net_1'
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.4;
                preset.vx = 0.0;  preset.vy = 15.5;  preset.vz = 0.45;
                preset.bounceT = 0.75;
    
            case 'serve_net_2'
                preset.x0 = 0.0;  preset.y0 = -11.0; preset.z0 = 2.4;
                preset.vx = 0.4;  preset.vy = 15.3;  preset.vz = 0.35;
                preset.bounceT = 0.74;
    
            % ---------------- VOLLEYS: 2 IN ----------------
            case 'volley_in_1'
                preset.x0 = -1.0; preset.y0 = -2.0; preset.z0 = 2.0;
                preset.vx = 0.8;  preset.vy = 11.0; preset.vz = 0.8;
                preset.bounceT = 0.62;
    
            case 'volley_in_2'
                preset.x0 = 1.0;  preset.y0 = -1.5; preset.z0 = 2.1;
                preset.vx = -0.5; preset.vy = 10.6; preset.vz = 0.75;
                preset.bounceT = 0.64;
    
            % ---------------- VOLLEYS: 2 OUT ----------------
            case 'volley_out_1'   % too wide
                preset.x0 = -1.0; preset.y0 = -2.0; preset.z0 = 2.0;
                preset.vx = 9.0;  preset.vy = 11.0; preset.vz = 0.8;
                preset.bounceT = 0.64;
            
            case 'volley_out_2'   % too deep
                preset.x0 = 0.0;  preset.y0 = -1.5; preset.z0 = 2.0;
                preset.vx = 0.2;  preset.vy = 21.0; preset.vz = 0.7;
                preset.bounceT = 0.84;
            
            % ---------------- VOLLEYS: 2 NET ----------------
            case 'volley_net_1'
                preset.x0 = 0.0;  preset.y0 = -2.0; preset.z0 = 1.10;
                preset.vx = 0.1;  preset.vy = 7.0;  preset.vz = 0.15;
                preset.bounceT = 0.58;
            
            case 'volley_net_2'
                preset.x0 = -0.6; preset.y0 = -2.0; preset.z0 = 1.05;
                preset.vx = 0.3;  preset.vy = 7.2;  preset.vz = 0.10;
                preset.bounceT = 0.60;
    
            otherwise
                error('Unknown shot preset: %s', char(name));
        end
    
        if preset.z0 <= groundZ
            preset.z0 = groundZ + 0.5;
        end
    end

    function [contactXYZ, contactT] = findCourtContactFromPreset(shotPreset, groundZ)
        if ischar(shotPreset) || isstring(shotPreset)
            preset = getShotPreset(shotPreset, groundZ);
        else
            preset = shotPreset;
        end
    
        contactT = preset.bounceT;
        xC = preset.x0 + preset.vx * contactT;
        yC = preset.y0 + preset.vy * contactT;
    
        contactXYZ = [xC, yC, groundZ];
    end

    function [contactXYZ, contactT, idxBefore] = findCourtContact(t, xyz, groundZ)
        contactXYZ = [NaN NaN NaN];
        contactT = NaN;
        idxBefore = [];
    
        if numel(t) < 2 || size(xyz,1) < 2
            return;
        end
    
        z = xyz(:,3);
    
        for k = 1:numel(t)-1
            z1 = z(k);
            z2 = z(k+1);
    
            if ~isfinite(z1) || ~isfinite(z2)
                continue;
            end
    
            % first downward crossing of the court plane
            if z1 >= groundZ && z2 <= groundZ && z2 < z1
                alpha = (groundZ - z1) / (z2 - z1);
                alpha = max(0, min(1, alpha));
    
                contactT = t(k) + alpha * (t(k+1) - t(k));
                xC = xyz(k,1) + alpha * (xyz(k+1,1) - xyz(k,1));
                yC = xyz(k,2) + alpha * (xyz(k+1,2) - xyz(k,2));
    
                contactXYZ = [xC, yC, groundZ];
                idxBefore = k;
                return;
            end
        end
    
        % fallback: use the minimum-z sample
        [~, kmin] = min(z);
        if ~isempty(kmin) && isfinite(z(kmin))
            contactXYZ = [xyz(kmin,1), xyz(kmin,2), groundZ];
            contactT = t(kmin);
            idxBefore = kmin;
        end
    end
    
    function [isNet, netXYZ, netT] = checkNetCrossing(t, xyz, courtGeom)
        isNet = false;
        netXYZ = [NaN NaN NaN];
        netT = NaN;
    
        if numel(t) < 2 || size(xyz,1) < 2
            return;
        end
    
        yNet = courtGeom.netY;
        zNet = courtGeom.netHeight;
    
        for k = 1:numel(t)-1
            y1 = xyz(k,2);
            y2 = xyz(k+1,2);
    
            if ~isfinite(y1) || ~isfinite(y2)
                continue;
            end
    
            % crossing from near side to far side
            if y1 <= yNet && y2 >= yNet && y2 > y1
                alpha = (yNet - y1) / (y2 - y1);
                alpha = max(0, min(1, alpha));
    
                xC = xyz(k,1) + alpha * (xyz(k+1,1) - xyz(k,1));
                zC = xyz(k,3) + alpha * (xyz(k+1,3) - xyz(k,3));
                tC = t(k) + alpha * (t(k+1) - t(k));
    
                netXYZ = [xC, yNet, zC];
                netT = tC;
    
                if zC <= zNet
                    isNet = true;
                end
                return;
            end
        end
    end
    
    function [decision, contactXYZ, netXYZ, contactT, netT] = classifyShotOutcome(shotType, t, xyz, courtGeom)
        [isNet, netXYZ, netT] = checkNetCrossing(t, xyz, courtGeom);
        [contactXYZ, contactT, ~] = findCourtContact(t, xyz, courtGeom.groundZ);
    
        if isNet
            decision = "NET";
            return;
        end
    
        if all(isfinite(contactXYZ))
            decision = classifyShotFromBounce(shotType, contactXYZ, courtGeom);
        else
            decision = "UNKNOWN";
        end
    end

    function plotSingleShotCourt(shot, shotName, courtGeom, shotNo)
        figure('Name', sprintf('Court Plot - %s', shotName), 'Color', 'w');
        ax = axes;
        hold(ax, 'on');
        grid(ax, 'on');
    
        % Court boundary in rotated display coordinates
        xw = [courtGeom.polyX courtGeom.polyX(1)];
        yw = [courtGeom.polyY courtGeom.polyY(1)];
        plotX = yw;
        plotY = -xw;
        plot(ax, plotX, plotY, 'Color', [0 0.5 0], 'LineWidth', 1.5);
    
        % Net line
        [px, py] = deal([courtGeom.netY courtGeom.netY], ...
                        [-max(courtGeom.polyX) -min(courtGeom.polyX)]);
        plot(ax, px, py, 'r-', 'LineWidth', 1.2);

        % Choose which point to plot
        if strcmpi(shot.decision, "NET") && all(isfinite(shot.netXYZ))
            plotXYZ = shot.netXYZ;
        elseif all(isfinite(shot.bounceXYZ))
            plotXYZ = shot.bounceXYZ;
        else
            plotXYZ = [NaN NaN NaN];
        end
    
        % Draw point
        if all(isfinite(plotXYZ))
            bx = plotXYZ(2);   % display X = world Y
            by = -plotXYZ(1);  % display Y = -world X
    
            switch upper(string(shot.decision))
                case "IN"
                    mk = 'go';
                case "OUT"
                    mk = 'ro';
                case "NET"
                    mk = 'yo';
                otherwise
                    mk = 'ko';
            end
    
            % Draw the real tennis ball footprint to scale.
            % This circle is in meters, unlike MarkerSize which is just screen size.
            if isfield(courtGeom, 'ballRadius')
                ballR = courtGeom.ballRadius;
            else
                ballR = 0.0335;   % meters
            end
            
            theta = linspace(0, 2*pi, 100);
            
            ballCircleX = bx + ballR*cos(theta);
            ballCircleY = by + ballR*sin(theta);
            
            % Use decision color for the real ball footprint
            switch upper(string(shot.decision))
                case "IN"
                    ballColor = 'g';
                case "OUT"
                    ballColor = 'r';
                case "NET"
                    ballColor = 'y';
                otherwise
                    ballColor = 'k';
            end
            
            % Actual ball edge, to scale
            plot(ax, ballCircleX, ballCircleY, '-', ...
                'Color', ballColor, 'LineWidth', 1.5);
            
            % Center point, not meant to represent ball size
            plot(ax, bx, by, '.', ...
                'Color', ballColor, 'MarkerSize', 12);
            
            % Optional small visual marker so you can still see the shot location.
            % This is NOT to scale.
            plot(ax, bx, by, mk, 'MarkerSize', 6, 'LineWidth', 1.0);
            
            text(bx + 0.2, by, sprintf('%s', shot.decision), ...
                'FontSize', 10, 'FontWeight', 'bold');
        end
    
        xlabel(ax, 'Y (m)');
        ylabel(ax, '-X (m)');
        title(ax, sprintf('%s | %s', shotName, shot.decision));
        axis(ax, 'equal');
        xlim(ax, [min(courtGeom.polyY)-1, max(courtGeom.polyY)+1]);
        ylim(ax, [-max(courtGeom.polyX)-1, -min(courtGeom.polyX)+1]);
        hold(ax, 'off');

        % image folder inside the current code folder
        imageDir = fullfile(pwd, 'images');
        
        if ~exist(imageDir, 'dir')
            mkdir(imageDir);
        end
        
        % safe for a filename
        safeShotName = regexprep(char(shotName), '[^\w\-]', '_');
        
        % save as a PNG
        saveas(gcf, fullfile(imageDir, sprintf('%s_shot_no_%d.png', safeShotName, shotNo)));
    end

end