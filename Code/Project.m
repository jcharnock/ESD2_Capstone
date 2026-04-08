function Project
%% Project - Tracking a ball from serve to end using Blender
%
% PURPOSE
%   - Connect MatLab to Blender and render left/right camera images.
%   - Detect the ball centroid in BOTH images.
%   - Use stereo triangulation to compute the 3D position (x,y,z) of the ball.
%   - Plot distance vs accuracy for distances from 3 m to 5 m
%    
% *Note (as of 03/17): Our restitution calculation (e) is only defined if
% the trajectory actually has a bounce (ball hits court). This is in the
% imported .dat files we are using, which don't appear to have this.

clc; close all;

%% -------------------- Blender Server Settings --------------------
BLENDER.server_ip   = '127.0.0.1';
BLENDER.server_port = 55001;

% Object names in Blender
BLENDER.ballName = "tennisBall";   % must match Blender object name

% Two stereo pairs (tennisCourt.blend — measured world transforms):
%   Camera1 (-1.5,-13,12)  Camera2 (1.5,-13,12)  -> baseline 3 m along X, center (0,-13,12)
%   Camera3 (1.5,-2,12)      Camera4 (-1.5,-2,12)  -> baseline 3 m along X, center (0,-2,12)
% Code places L/R at camBase +/- B/2 on X (matches -1.5 and +1.5).
BLENDER.pair1.camL = "Camera1";
BLENDER.pair1.camR = "Camera2";
BLENDER.pair1.camBaseX = 0.0;
BLENDER.pair1.camBaseY = -13.0;
BLENDER.pair1.camBaseZ = 10.0;
BLENDER.pair1.B_m    = 2.0;
BLENDER.pair1.camPitch = 30;
BLENDER.pair1.camRoll  = 0;
BLENDER.pair1.camYaw   = 0;

BLENDER.pair2.camL = "Camera3";
BLENDER.pair2.camR = "Camera4";
BLENDER.pair2.camBaseX = 0.0;
BLENDER.pair2.camBaseY = -2.0;
BLENDER.pair2.camBaseZ = 9.0;
BLENDER.pair2.B_m    = 2.0;
BLENDER.pair2.camPitch = 38;
BLENDER.pair2.camRoll  = 0;
BLENDER.pair2.camYaw   = 0;

% Auto: yBall < courtDividerY -> pair 1 (Y~-13), else pair 2 (Y~-2). Divider default -7.5 m.
BLENDER.courtDividerY = -7.5;

% Render size
BLENDER.width  = 752;
BLENDER.height = 480;

% Camera orientation (degrees). In patched blenderServer.py, if all
% are 0, the server keeps the original camera rotation.
BLENDER.camPitch = 0;
BLENDER.camRoll  = 0;
BLENDER.camYaw   = 0;

% Ball orientation (degrees)
BLENDER.ballPitch = 0;
BLENDER.ballRoll  = 0;
BLENDER.ballYaw   = 0;

% ---------- Blender ball depth (scene placement; not used for detector sizing) ----------
defaults.Ztrue_m = 3.0;

% use f_px (pixels) and baseline (meters)
defaults.f_px = 2632;     % focal length in pixels (must match your Blender camera properties)
defaults.B_m  = 3.0;      % nominal baseline (each pair sets its own; matches rigs)
defaults.camZ = 12.0;     % nominal camera Z (pairs use camBaseZ from BLENDER.pair*)
defaults.Zoffset = 0.25;  % Ball is set to z = Ztrue - Zoffset in Blender (matches server move)

% principal point defaults (auto set to image center on render)
defaults.cx   = 376;      % overwritten by actual image width/2
defaults.cy   = 240;      % overwritten by actual image height/2

% detection defaults: YCbCr tennis ball (excludes white lines + court green)
defaults.method = "YCbCrTennis";
defaults.grayThresh = 0.00;       % legacy GrayThresh only
defaults.whiteChromaTol = 0.08;  % |Cb-0.5|,|Cr-0.5| below this + high Y => white line
defaults.minBlobArea = 10;
defaults.maxBlobArea = 2e4;

% ROI / crop defaults used by the final detector
defaults.yMin = 0.45;      % reserved / unused in final pipeline
defaults.tCbCr = 0.15;     % reserved / unused in final pipeline

defaults.useTopCrop = false;
defaults.topFrac    = 0.75;

% overlay
defaults.overlayRadiusPx = 16;

% Stereo: "Auto (by court Y)" | "Pair 1 (Cam1+2)" | "Pair 2 (Cam3+4)"
defaults.stereoMode      = "Auto (by court Y)";
defaults.courtDividerY   = -7.5;  % Auto: yBall < this uses pair 1 (Y~-13), else pair 2 (Y~-2)
% 4-view live path: (1) compare YCbCr blob radii on pair1 vs pair2 left — closer rig -> larger r;
% (2) tie-break: ball row v vs net line vLine = courtNetVFrac*H on pair-1 left (else pair-2 left)
defaults.courtNetVFrac   = 0.42;
defaults.courtSideRadiusDiffPx = 2.0;  % min |r1-r2| to trust radius-based side (pixels)

%% -------------------- GUI Setup --------------------
ui = uifigure('Name','Lab 4 - Stereo Ball Localization (Blender)','Position',[100 100 1200 700]);
ui.CloseRequestFcn = @(~,~) onClose();

client = [];

busy = false;                 % prevents overlapping Blender calls
pendingUpdate = false;        % UI changed while busy -> run one more update

% Keep last-good centroids to stabilize tracking and prevent L/R flips
lastCL = [];
lastCR = [];
lastC1L = []; lastC1R = [];
lastC2L = []; lastC2R = [];
lastBallWorldXY = [NaN NaN];

autoTimer = timer( ...
    'ExecutionMode','singleShot', ...
    'StartDelay', 0.15, ...    % debounce delay
    'TimerFcn', @(~,~) safeUpdateOnce() );

main = uigridlayout(ui,[3 3]);
main.ColumnWidth = {430,'1x','1x'};
main.RowHeight   = {190, 300, '1x'};
main.Padding = [10 10 10 10];
main.RowSpacing = 10;
main.ColumnSpacing = 10;

% Panels for left and right images
pL = uipanel(main,'Title','Left Image');
pL.Layout.Row = 1; pL.Layout.Column = 2;
axL = uiaxes(pL);
axL.Position = [10 10 pL.Position(3)-20 pL.Position(4)-40];
axis(axL,'image'); axis(axL,'off');

pR = uipanel(main,'Title','Right Image');
pR.Layout.Row = 1; pR.Layout.Column = 3;
axR = uiaxes(pR);
axR.Position = [10 10 pR.Position(3)-20 pR.Position(4)-40];
axis(axR,'image'); axis(axR,'off');

% 3D plot panel
p3 = uipanel(main,'Title','3D Ball Position');
p3.Layout.Row = [2 3];
p3.Layout.Column = [2 3];

g3 = uigridlayout(p3,[1 1]);
g3.Padding = [6 6 6 6];
g3.RowHeight = {'1x'};

% 3D trajectory (full panel; benchmark depth plot removed)
ax3 = uiaxes(g3);
grid(ax3,'on'); view(ax3,3);
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
title(ax3,'3D Ball Trajectory');
hold(ax3,'on');
hTraj = plot3(ax3, NaN, NaN, NaN, 'b-','LineWidth',1.5);                     % full trajectory
hPt3  = plot3(ax3, NaN, NaN, NaN, 'ro','MarkerSize',10,'LineWidth',2);       % current ball
hLand = plot3(ax3, NaN, NaN, NaN, 'gs','MarkerSize',8,'MarkerFaceColor','g');% landing point
hold(ax3,'off');
% Controls (top-left)
pCtrl = uipanel(main,'Title','Controls');
pCtrl.Layout.Row = 1;
pCtrl.Layout.Column = 1;

g = uigridlayout(pCtrl,[12 3]);
g.RowHeight = {36,28,28,36,28,28,36,28,28,28,28,34};
g.ColumnWidth = {160,'1x',70};
g.Padding = [10 10 10 10];
g.RowSpacing = 8;
g.ColumnSpacing = 8;

% Row 1: Blender only — ball height below cameras (not used for detector sizing)
uilabel(g,'Text','Blender ball depth (m)');
sZ = uislider(g,'Limits',[3 5],'Value',defaults.Ztrue_m);
sZ.ValueChangedFcn = @(src,~) onZSlider(src);
sZ.Layout.Row = 1; sZ.Layout.Column = 2;
eZ = uieditfield(g,'numeric','Value',defaults.Ztrue_m);
eZ.ValueChangedFcn = @(src,~) onEditZ(src);
eZ.Layout.Row = 1; eZ.Layout.Column = 3;

% Row 2: fixed final detection pipeline (no dropdown)
uilabel(g,'Text','Detection pipeline');
lblPipeline = uilabel(g,'Text','YCbCr tennis ball | 4 cams | pair by ball row vs net');
lblPipeline.Layout.Row = 2;
lblPipeline.Layout.Column = [2 3];

% Row 3: f_px
uilabel(g,'Text','f (pixels)');
eF = uieditfield(g,'numeric','Value',defaults.f_px);
eF.ValueChangedFcn = @(~,~) onParamChange();
eF.Layout.Row = 3; eF.Layout.Column = [2 3];

% Row 4: baseline B
uilabel(g,'Text','Baseline B (m)');
eB = uieditfield(g,'numeric','Value',defaults.B_m);
eB.ValueChangedFcn = @(~,~) onParamChange();
eB.Layout.Row = 4; eB.Layout.Column = [2 3];

% Row 5: net line (pair 1 left image: v < frac*H -> pair 1 Cam1+2, else pair 2 Cam3+4)
uilabel(g,'Text','Net line (frac H)');
eCourtNet = uieditfield(g,'numeric','Value',defaults.courtNetVFrac,'Limits',[0.05 0.95]);
eCourtNet.ValueChangedFcn = @(~,~) onParamChange();
eCourtNet.Layout.Row = 5;
eCourtNet.Layout.Column = [2 3];

% Row 6: white-line chroma (YCbCr)
uilabel(g,'Text','Line chroma tol');
eWhiteTol = uieditfield(g,'numeric','Value',defaults.whiteChromaTol,'Limits',[0.02 0.2]);
eWhiteTol.ValueChangedFcn = @(~,~) onParamChange();
eWhiteTol.Layout.Row = 6; eWhiteTol.Layout.Column = [2 3];

% Row 7: gray thresh
uilabel(g,'Text','Gray thresh (0..1)');
sGray = uislider(g,'Limits',[0 1],'Value',defaults.grayThresh);
sGray.ValueChangedFcn = @(src,~) onGraySlider(src);
sGray.Layout.Row = 7; sGray.Layout.Column = 2;
eGray = uieditfield(g,'numeric','Value',defaults.grayThresh,'Limits',[0 1]);
eGray.ValueChangedFcn = @(src,~) onGrayEdit(src);
eGray.Layout.Row = 7; eGray.Layout.Column = 3;

% Row 8: min blob area
uilabel(g,'Text','Min blob area');
eMinA = uieditfield(g,'numeric','Value',defaults.minBlobArea);
eMinA.ValueChangedFcn = @(~,~) onParamChange();
eMinA.Layout.Row = 8; eMinA.Layout.Column = [2 3];

% Row 9: max blob area
uilabel(g,'Text','Max blob area');
eMaxA = uieditfield(g,'numeric','Value',defaults.maxBlobArea);
eMaxA.ValueChangedFcn = @(~,~) onParamChange();
eMaxA.Layout.Row = 9; eMaxA.Layout.Column = [2 3];

% Row 10: YCbCr brightness min (only for YCbCrNeutral)
uilabel(g,'Text','YCbCr: Y min (0=auto)');
eYmin = uieditfield(g,'numeric','Value',0,'Limits',[0 1]);  % Value=0 means AUTO
eYmin.ValueChangedFcn = @(~,~) onParamChange();
eYmin.Layout.Row = 10; eYmin.Layout.Column = [2 3];

% Row 11: YCbCr chroma tolerance (only for YCbCrNeutral)
uilabel(g,'Text','YCbCr: chroma tol');
eTcbcr = uieditfield(g,'numeric','Value',defaults.tCbCr,'Limits',[0 0.5]);
eTcbcr.ValueChangedFcn = @(~,~) onParamChange();
eTcbcr.Layout.Row = 11; eTcbcr.Layout.Column = [2 3];

% Row 12: top checkbox + frac
cbTop = uicheckbox(g,'Text','Search only top of image','Value',defaults.useTopCrop);
cbTop.ValueChangedFcn = @(~,~) onParamChange();
cbTop.Layout.Row = 12; cbTop.Layout.Column = [1 2];
eTopFrac = uieditfield(g,'numeric','Value',defaults.topFrac,'Limits',[0.1 1]);
eTopFrac.ValueChangedFcn = @(~,~) onParamChange();
eTopFrac.Layout.Row = 12; eTopFrac.Layout.Column = 3;

% Output panel (bottom-left)
pOut = uipanel(main,'Title','Output');
pOut.Layout.Row = [2 3];
pOut.Layout.Column = 1;

go = uigridlayout(pOut,[16 1]);
go.RowHeight = {32,28,28,28,28,28,24,24,24,24,22,22,22,22,22,'1x'};
go.Padding = [10 10 10 10];
go.RowSpacing = 4;

btnCalc = uibutton(go,'Text','Single Frame Test','ButtonPushedFcn',@(~,~) safeUpdateOnce());
btnCalc.Layout.Row = 1;

btnAccBall = uibutton(go,'Text','Accuracy vs Ball Position','ButtonPushedFcn',@(~,~) runAccVsBall());
btnAccBall.Layout.Row = 2;

btnAccCam = uibutton(go,'Text','Accuracy vs Camera Motion','ButtonPushedFcn',@(~,~) runAccVsCamera());
btnAccCam.Layout.Row = 3;

btnServeVolley = uibutton(go,'Text','Serves/Volleys Playback','ButtonPushedFcn',@(~,~) runServesAndVolleys());
btnServeVolley.Layout.Row = 4;

btnReplay = uibutton(go,'Text','Instant Replay','ButtonPushedFcn',@(~,~) runInstantReplay());
btnReplay.Layout.Row = 5;

btnRest = uibutton(go,'Text','Restitution Test','ButtonPushedFcn',@(~,~) runRestitutionTest());
btnRest.Layout.Row = 6;

lblCourt = uilabel(go,'Text','Court side: --','FontWeight','bold');
lblCourt.Layout.Row = 7;

lblL = uilabel(go,'Text','Left centroid (u,v): --');  lblL.Layout.Row = 8;
lblR = uilabel(go,'Text','Right centroid (u,v): --'); lblR.Layout.Row = 9;
lblD = uilabel(go,'Text','Disparity d (px): --');     lblD.Layout.Row = 10;
lblX = uilabel(go,'Text','X (m): --');                lblX.Layout.Row = 11;
lblY = uilabel(go,'Text','Y (m): --');                lblY.Layout.Row = 12;
lblZ = uilabel(go,'Text','Z (m): --');                lblZ.Layout.Row = 13;

lblLEDGreen = uilabel(go,'Text','LED IN: OFF','FontColor',[0 0.6 0]); lblLEDGreen.Layout.Row = 14;
lblLEDRed   = uilabel(go,'Text','LED OUT: OFF','FontColor',[0.6 0 0]); lblLEDRed.Layout.Row   = 15;

msg = uitextarea(go,'Editable','off');
msg.Layout.Row = 16;

% LED state (GUI simulation of hardware LEDs)
LED.state = "idle";   % "idle" | "IN" | "OUT" | "NET"
LED.redOn = false;
ledTimer = timer('ExecutionMode','fixedRate', ...
                 'Period',0.1, ...    % 10 Hz blink
                 'TimerFcn',@(~,~) updateLEDBlink());

% Last trajectory/result for replay and restitution
lastTraj = [];
lastResult = struct();

% Bindings between slider and edit box (Ztrue)
sZ.ValueChangingFcn = @(src,evt)set(eZ,'Value',evt.Value);

% Gray thresh link
sGray.ValueChangingFcn = @(src,evt)set(eGray,'Value',evt.Value);

% TCP Client State
client = [];

%% -------------------- Core callback --------------------
    function clearAllOutputs()
        % Clears all outputs (including disparity)/3D point/3D limits
        lblCourt.Text = "Court side: --";
        lblD.Text = "Disparity d (px): --";
        lblX.Text = "X (m): --";
        lblY.Text = "Y (m): --";
        lblZ.Text = "Z (m): --";
        hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

        % Keep 3D axes limited (prevents weird autoscale issue from stale values)
        xlim(ax3, [-0.5 0.5]);
        ylim(ax3, [-0.5 0.5]);
        zlim(ax3, [0 6]);
    end

    function clearXYZPointOnly()
        % Clears only x/y/z and 3D point (keep disparity text visible)
        lblX.Text = "X (m): --";
        lblY.Text = "Y (m): --";
        lblZ.Text = "Z (m): --";
        hPt3.XData = NaN; hPt3.YData = NaN; hPt3.ZData = NaN;

        xlim(ax3, [-0.5 0.5]);
        ylim(ax3, [-0.5 0.5]);
        zlim(ax3, [0 6]);
    end

    function updateOnce()
        % Prevent overlapping calls (overlap crashes the Blender timer)
        if busy
            pendingUpdate = true;
            return;
        end
        busy = true;
        btnCalc.Enable = 'off';
        cleanupObj = onCleanup(@() setBusyFalse());

        % read params from UI
        params = defaults;
        sceneDepth = eZ.Value;              % Blender ball placement only
        params.Ztrue_m     = sceneDepth;
        params.method      = "YCbCrTennis";
        params.f_px        = eF.Value;
        params.B_m         = eB.Value;
        params.grayThresh  = eGray.Value;
        params.minBlobArea = eMinA.Value;
        params.maxBlobArea = eMaxA.Value;
        params.yMin        = eYmin.Value;
        params.tCbCr       = eTcbcr.Value;
        params.whiteChromaTol = eWhiteTol.Value;
        params.useTopCrop  = cbTop.Value;
        params.topFrac     = eTopFrac.Value;
        params.courtNetVFrac = eCourtNet.Value;
        params.courtSideRadiusDiffPx = defaults.courtSideRadiusDiffPx;

        disp("=== updateOnce() CALLED ===");
        fprintf("Blender depth=%.2f m  method=%s\n", sceneDepth, params.method);
        drawnow;

        % ---- Stage 1: Four camera views (both stereo pairs) ----
        tIO = tic;
        [I1L, I1R, I2L, I2R, infoLines, B1, B2] = getFourViewsFromBlender(sceneDepth);

        msIO = 1000*toc(tIO);
        fprintf("msIO = %.1f ms (4 renders)\n", msIO);

        if ~isempty(infoLines)
            msg.Value = infoLines;
        else
            msg.Value = "No info lines.";
        end

        if isempty(I1L) || isempty(I1R) || isempty(I2L) || isempty(I2R)
            cla(axL); cla(axR);
            clearAllOutputs();
            lblL.Text = "Left centroid (u,v): --";
            lblR.Text = "Right centroid (u,v): --";
            return;
        end

        W = size(I1L,2);
        H = size(I1L,1);

        % ---- Stage 2A: YCbCr on all four images to choose stereo pair ----
        pSide = params;
        pSide.method = "YCbCrTennis";

        p1L = pSide; p1L.refCentroid = lastC1L;
        p1R = pSide; p1R.refCentroid = lastC1R;
        p2L = pSide; p2L.refCentroid = lastC2L;
        p2R = pSide; p2R.refCentroid = lastC2R;

        [c1L_y, dbg1L_y, r1L] = detectBallCentroid(I1L, p1L);
        [c1R_y, dbg1R_y, r1R] = detectBallCentroid(I1R, p1R);
        [c2L_y, dbg2L_y, r2L] = detectBallCentroid(I2L, p2L);
        [c2R_y, dbg2R_y, r2R] = detectBallCentroid(I2R, p2R);

        [pairIdx, pairNote, courtSideStr] = chooseStereoPairByBallSide( ...
            c1L_y, c1R_y, c2L_y, c2R_y, r1L, r2L, W, H, params.courtNetVFrac, params.courtSideRadiusDiffPx);

        if pairIdx < 0
            clearAllOutputs();
            imshow(I1L,'Parent',axL); title(axL,'Pair1 L (no valid stereo)');
            imshow(I1R,'Parent',axR); title(axR,'Pair1 R');
            msg.Value = [
                infoLines(:)
                string(pairNote)
                "YCbCr dbg pair1 L:"; dbg1L_y(:); "YCbCr dbg pair1 R:"; dbg1R_y(:)
                "YCbCr dbg pair2 L:"; dbg2L_y(:); "YCbCr dbg pair2 R:"; dbg2R_y(:)
            ];
            lblL.Text = "Left centroid (u,v): --";
            lblR.Text = "Right centroid (u,v): --";
            return;
        end

        % ---- Stage 2B: choose pair, then refine exact centroid with Circles only on that pair ----
        if pairIdx == 1
            IL = I1L; IR = I1R;
            B_used = B1;

            coarseL = c1L_y;
            coarseR = c1R_y;
            coarseRadL = r1L;
            coarseRadR = r1R;

            lastC1L = c1L_y; lastC1R = c1R_y;
        else
            IL = I2L; IR = I2R;
            B_used = B2;

            coarseL = c2L_y;
            coarseR = c2R_y;
            coarseRadL = r2L;
            coarseRadR = r2R;

            lastC2L = c2L_y; lastC2R = c2R_y;
        end

        if ~isfinite(coarseRadL) || coarseRadL <= 0
            coarseRadL = 4;
        end
        if ~isfinite(coarseRadR) || coarseRadR <= 0
            coarseRadR = 4;
        end

        pCircL = params;
        pCircR = params;
        pCircL.method = "Circles";
        pCircR.method = "Circles";

        if all(isfinite(coarseL))
            pCircL.refCentroid = coarseL;
        else
            pCircL.refCentroid = lastCL;
        end
        if all(isfinite(coarseR))
            pCircR.refCentroid = coarseR;
        else
            pCircR.refCentroid = lastCR;
        end

        pCircL.expectedRadiusPx = coarseRadL;
        pCircR.expectedRadiusPx = coarseRadR;

        [cL, dbgL, ~] = detectBallCentroid(IL, pCircL);
        [cR, dbgR, ~] = detectBallCentroid(IR, pCircR);

        if any(~isfinite(cL)) && all(isfinite(coarseL))
            cL = coarseL;
            dbgL = [dbgL; "Circles failed -> fallback to YCbCr coarse centroid."];
        end
        if any(~isfinite(cR)) && all(isfinite(coarseR))
            cR = coarseR;
            dbgR = [dbgR; "Circles failed -> fallback to YCbCr coarse centroid."];
        end

        if all(isfinite(cL)), lastCL = cL; end
        if all(isfinite(cR)), lastCR = cR; end

        imshow(IL,'Parent',axL);
        title(axL, sprintf("LEFT pair %d (chosen)", pairIdx));
        imshow(IR,'Parent',axR);
        title(axR, sprintf("RIGHT pair %d (chosen)", pairIdx));

        if any(~isfinite(cL)) || any(~isfinite(cR))
            clearAllOutputs();
            msg.Value = [
                infoLines(:)
                string(pairNote)
                "Chosen pair selected by YCbCr, but final circles refine failed."
                "Left dbg:"
                dbgL(:)
                "Right dbg:"
                dbgR(:)
            ];
            return;
        end

        % overlay centroid markers (green point and circle so it's obvious)
        rPx = max(6, round(params.overlayRadiusPx));
        hold(axL,'on');
        plot(axL, cL(1), cL(2), 'g+','MarkerSize',10,'LineWidth',2);
        try viscircles(axL, cL, rPx, 'Color','g','LineWidth',1); catch, end
        hold(axL,'off');

        hold(axR,'on');
        plot(axR, cR(1), cR(2), 'g+','MarkerSize',10,'LineWidth',2);
        try viscircles(axR, cR, rPx, 'Color','g','LineWidth',1); catch, end
        hold(axR,'off');

        lblL.Text = sprintf("Left centroid (u,v): (%.1f, %.1f)", cL(1), cL(2));
        lblR.Text = sprintf("Right centroid (u,v): (%.1f, %.1f)", cR(1), cR(2));
        lblCourt.Text = sprintf('Court side: %s', courtSideStr);

        % ---- Stage 3: Stereo triangulation ----
        % principal point from image center
        w = size(IL,2);
        h = size(IL,1);
        cx = w/2;
        cy = h/2;

        f = params.f_px;
        B = B_used;

        du = cL(1) - cR(1);          % horizontal disparity
        dv = cL(2) - cR(2);          % vertical disparity (reported for debug only)

        % Use only horizontal disparity for depth
        d    = du;
        dabs = abs(d);
        lblD.Text = sprintf("Disp_u (px): %.2f   (du=%.2f, dv=%.2f)", d, du, dv);

        if ~isfinite(dabs) || dabs < 1.0
            clearXYZPointOnly();
            msg.Value = [infoLines; sprintf("Disparity too small (< 1 px): |du|=%.2f -> depth unstable. Skipping update.", dabs)];
            return;
        end

        Z = (f * B) / dabs;          % Z now scales using the correct disparity direction
        X = ((cL(1) - cx) * Z) / f;
        Y = ((cL(2) - cy) * Z) / f;

        lblX.Text = sprintf("X (m): %.3f", X);
        lblY.Text = sprintf("Y (m): %.3f", Y);

        Zdepth = Z;
        lblZ.Text = sprintf("Z (depth) (m): %.3f", Zdepth);

        lastBallWorldXY = [X Y];

        % --- Update current 3D point and maintain short trajectory tail ---
        hPt3.XData = X;
        hPt3.YData = Y;
        hPt3.ZData = Z;

        % Append to trajectory (keep recent N points for live view)
        Nkeep = 200;
        if all(isfinite(hTraj.XData))
            xHist = [hTraj.XData(:); X];
            yHist = [hTraj.YData(:); Y];
            zHist = [hTraj.ZData(:); Z];
        else
            xHist = X; yHist = Y; zHist = Z;
        end
        if numel(xHist) > Nkeep
            xHist = xHist(end-Nkeep+1:end);
            yHist = yHist(end-Nkeep+1:end);
            zHist = zHist(end-Nkeep+1:end);
        end
        hTraj.XData = xHist;
        hTraj.YData = yHist;
        hTraj.ZData = zHist;

        % Auto limits so ball is always visible
        padXY = 0.25;              % meters
        padZ  = max(0.5, 0.2*Z);   % scale with distance

        xlim(ax3, sort([X - padXY, X + padXY]));
        ylim(ax3, sort([Y - padXY, Y + padXY]));
        zlim(ax3, sort([max(0, Z - padZ), Z + padZ]));

        msg.Value = [
            infoLines
            string(pairNote)
            "Centroids OK."
            sprintf("X=%.3f  Y=%.3f  Z=%.3f", X, Y, Z)
        ];
    end

    function setBusyFalse()
        busy = false;
        % btnCalc may not be a valid UI object if construction failed earlier
        try
            if ~isempty(btnCalc) && isvalid(btnCalc)
                btnCalc.Enable = 'on';
            end
        catch
        end
        if pendingUpdate
            pendingUpdate = false;
            % run one more update with latest UI values
            scheduleUpdate();
        end
    end

%% -------------------- Debounce / UI Helper Functions --------------------
    function onParamChange()
        scheduleUpdate();
    end

    function onEditZ(src)
        % Keep Z slider in sync with numeric edit, then debounce the update
        try
            set(sZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGraySlider(src)
        try
            set(eGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onGrayEdit(src)
        try
            set(sGray,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function onZSlider(src)
        try
            set(eZ,'Value',src.Value);
        catch
        end
        scheduleUpdate();
    end

    function scheduleUpdate()
        % Restart the timer so rapid changes coalesce
        try
            if ~isempty(autoTimer) && isvalid(autoTimer)
                stop(autoTimer);
                start(autoTimer);
            else
                safeUpdateOnce();
            end
        catch
            safeUpdateOnce();
        end
    end

    function onClose()
        % Stop timer, close TCP client, then close the UI
        try
            if ~isempty(autoTimer) && isvalid(autoTimer)
                stop(autoTimer);
                delete(autoTimer);
            end
        catch
        end
        try
            if ~isempty(client) && isvalid(client)
                clear client; % closes tcpclient
            end
        catch
        end
        try
            if ~isempty(ledTimer) && isvalid(ledTimer)
                stop(ledTimer);
                delete(ledTimer);
            end
        catch
        end
        delete(ui);
    end

%% -------------------- Blender I/O --------------------
    function ok = pairStereoUsable(cL, cR, W, H)
        % True if both YCbCr centroids exist and horizontal disparity is usable (matches updateOnce).
        ok = false;
        if isempty(cL) || isempty(cR) || isempty(W) || isempty(H)
            return;
        end
        if any(~isfinite(cL)) || any(~isfinite(cR))
            return;
        end
        du = cL(1) - cR(1);
        ok = isfinite(du) && abs(du) >= 1.0;
    end

    function [pairIdx, pairNote, courtSideStr] = chooseStereoPairByBallSide( ...
            c1L, c1R, c2L, c2R, r1L, r2L, W, H, ~, radiusDiffPx)
        % Choose the stereo pair that actually has the ball closest / best seen.
        %
        % Uses stereo usability first, then larger apparent blob radius,
        % then a soft stereo score as a tie-break. This avoids using image-row
        % direction, which breaks when cameras are upside down / pitched.
        pairIdx = -1;
        pairNote = "";
        courtSideStr = "--";

        ok1 = pairStereoUsable(c1L, c1R, W, H);
        ok2 = pairStereoUsable(c2L, c2R, W, H);

        if ~ok1 && ~ok2
            pairNote = "Neither stereo pair has usable centroids/disparity.";
            return;
        end

        if ok1 && ~ok2
            pairIdx = 1;
            pairNote = "Only pair 1 has usable stereo.";
            courtSideStr = "Pair 1 side";
            return;
        elseif ok2 && ~ok1
            pairIdx = 2;
            pairNote = "Only pair 2 has usable stereo.";
            courtSideStr = "Pair 2 side";
            return;
        end

        r1ok = isfinite(r1L) && r1L > 0;
        r2ok = isfinite(r2L) && r2L > 0;

        if r1ok && r2ok
            dr = r1L - r2L;
            if abs(dr) >= radiusDiffPx
                if dr > 0
                    pairIdx = 1;
                    pairNote = sprintf("Pair 1 chosen: larger ball radius (r1=%.1f px, r2=%.1f px).", r1L, r2L);
                    courtSideStr = "Pair 1 side";
                else
                    pairIdx = 2;
                    pairNote = sprintf("Pair 2 chosen: larger ball radius (r2=%.1f px, r1=%.1f px).", r2L, r1L);
                    courtSideStr = "Pair 2 side";
                end
                return;
            end
        end

        s1 = stereoPairSoftScore(c1L, c1R, r1L, r1L, W, H);
        s2 = stereoPairSoftScore(c2L, c2R, r2L, r2L, W, H);

        if s1 >= s2
            pairIdx = 1;
            pairNote = sprintf("Radii similar; pair 1 wins soft stereo score (s1=%.2f, s2=%.2f).", s1, s2);
            courtSideStr = "Pair 1 side";
        else
            pairIdx = 2;
            pairNote = sprintf("Radii similar; pair 2 wins soft stereo score (s2=%.2f, s1=%.2f).", s2, s1);
            courtSideStr = "Pair 2 side";
        end
    end

    function s = ternaryStr(cond, a, b)
        if cond, s = a; else, s = b; end
    end

    function s = stereoPairSoftScore(cL, cR, rL, rR, W, H)
        % Higher is better: margin from borders, similar radii, horizontal disparity, small |dv|.
        s = -1e10;
        if any(~isfinite(cL)) || any(~isfinite(cR))
            return;
        end
        du = cL(1) - cR(1);
        dv = cL(2) - cR(2);
        if abs(du) < 1.5
            return;
        end
        mL = min([cL(1), cL(2), W - cL(1), H - cL(2)]);
        mR = min([cR(1), cR(2), W - cR(1), H - cR(2)]);
        margin = max(10, 0.055 * min(W, H));
        penB = max(0, margin - mL) + max(0, margin - mR);

        dvTol = max(16, 0.055 * H);
        penV = max(0, abs(dv) - dvTol) * 8;

        if ~isfinite(rL), rL = NaN; end
        if ~isfinite(rR), rR = NaN; end
        rs = max(max(rL, rR), 6);
        if isfinite(rL) && isfinite(rR)
            penR = abs(rL - rR) / rs;
        else
            penR = 0.25;
        end
        rMax = 0.34 * min(W, H);
        if (isfinite(rL) && (rL < 3 || rL > rMax)) || (isfinite(rR) && (rR < 3 || rR > rMax))
            penR = penR + 0.5;
        end

        s = 2 * min(mL, mR) + log1p(abs(du)) + 22 * (1 - min(1, penR)) - 3 * penB - penV;
    end

    function [I1L, I1R, I2L, I2R, infoLines, B1, B2] = getFourViewsFromBlender(sceneDepth_m)
        I1L = []; I1R = []; I2L = []; I2R = [];
        infoLines = string.empty(0,1);
        B1 = BLENDER.pair1.B_m;
        B2 = BLENDER.pair2.B_m;

        if ~ensureClient()
            infoLines = "No TCP client (ensureClient failed).";
            return;
        end

        c = client;
        flushFour(c);

        function out = blenderLinkRetry4(varargin)
            try
                out = blenderLink(varargin{:});
            catch ME
                isTimeout = contains(ME.message,"Timeout","IgnoreCase",true) || contains(ME.message,"timed out","IgnoreCase",true);
                isReadErr = contains(ME.message,"read","IgnoreCase",true) || contains(ME.message,"connection","IgnoreCase",true);
                if isTimeout || isReadErr
                    try clear client; catch, end
                    if ~ensureClient()
                        rethrow(ME);
                    end
                    varargin{1} = client;
                    out = blenderLink(varargin{:});
                else
                    rethrow(ME);
                end
            end
        end

        function flushFour(c2)
            try
                n = c2.NumBytesAvailable;
                if n > 0
                    read(c2, n, "uint8");
                end
            catch
            end
        end

        xBall = 0;
        yBall = -7.5;
        zBall = BLENDER.pair1.camBaseZ - sceneDepth_m;

        msg.Value = string(sprintf("Rendering 4 views | ball world z=%.2f (depth=%.2f m)", zBall, sceneDepth_m));
        drawnow;

        try
            blenderLinkRetry4(c, BLENDER.width, BLENDER.height, ...
                xBall, yBall, zBall, ...
                BLENDER.ballPitch, BLENDER.ballRoll, BLENDER.ballYaw, ...
                BLENDER.ballName);
        catch ME
            infoLines = [
                "Error moving ball in Blender."
                string(ME.message)
            ];
            return;
        end

        p1 = BLENDER.pair1;
        p2 = BLENDER.pair2;
        xL1 = p1.camBaseX - p1.B_m/2;
        xR1 = p1.camBaseX + p1.B_m/2;
        xL2 = p2.camBaseX - p2.B_m/2;
        xR2 = p2.camBaseX + p2.B_m/2;

        try
            I1L = blenderLinkRetry4(c, BLENDER.width, BLENDER.height, ...
                xL1, p1.camBaseY, p1.camBaseZ, ...
                p1.camPitch, p1.camRoll, p1.camYaw, p1.camL);
            I1R = blenderLinkRetry4(c, BLENDER.width, BLENDER.height, ...
                xR1, p1.camBaseY, p1.camBaseZ, ...
                p1.camPitch, p1.camRoll, p1.camYaw, p1.camR);
            I2L = blenderLinkRetry4(c, BLENDER.width, BLENDER.height, ...
                xL2, p2.camBaseY, p2.camBaseZ, ...
                p2.camPitch, p2.camRoll, p2.camYaw, p2.camL);
            I2R = blenderLinkRetry4(c, BLENDER.width, BLENDER.height, ...
                xR2, p2.camBaseY, p2.camBaseZ, ...
                p2.camPitch, p2.camRoll, p2.camYaw, p2.camR);
        catch ME
            infoLines = ["4-view render error."; string(ME.message)];
            I1L = []; I1R = []; I2L = []; I2R = [];
            return;
        end

        infoLines = [
            "Blender 4-view OK."
            sprintf("Pair1 %s | %s  B=%.2f m", p1.camL, p1.camR, p1.B_m)
            sprintf("Pair2 %s | %s  B=%.2f m", p2.camL, p2.camR, p2.B_m)
            sprintf("Ball z=%.2f  sceneDepth=%.2f m", zBall, sceneDepth_m)
        ];
    end

    function [IL, IR, infoLines, B_used, pairIdx] = getStereoFromBlender(Ztrue_m, params, ballWorldXY)
        % Render left/right from ONE stereo pair (Cam1+2 or Cam3+4).
        % ballWorldXY = [x y] in Blender world; used in Auto mode to pick pair.
        if nargin < 3 || isempty(ballWorldXY)
            ballWorldXY = [0 0];
        end

        infoLines = string.empty(0,1);
        IL = []; IR = [];
        B_used = params.B_m;
        pairIdx = 1;

        % Ensure a client exists (reconnect only here)
        if ~ensureClient()
            infoLines = ["No TCP client (ensureClient failed)."];
            return;
        end

        c = client;
        flushClient(c);

        function out = blenderLinkRetry(varargin)
            try
                out = blenderLink(varargin{:});
            catch ME
                isTimeout = contains(ME.message,"Timeout","IgnoreCase",true) || contains(ME.message,"timed out","IgnoreCase",true);
                isReadErr = contains(ME.message,"read","IgnoreCase",true) || contains(ME.message,"connection","IgnoreCase",true);
                if isTimeout || isReadErr
                    try clear client; catch, end
                    if ~ensureClient()
                        rethrow(ME);
                    end
                    varargin{1} = client;
                    out = blenderLink(varargin{:});
                else
                    rethrow(ME);
                end
            end
        end

        function flushClient(c2)
            try
                n = c2.NumBytesAvailable;
                if n > 0
                    read(c2, n, "uint8");
                end
            catch
            end
        end

        pairIdx = chooseActiveStereoPair(params, ballWorldXY(1), ballWorldXY(2));
        if pairIdx == 1
            pair = BLENDER.pair1;
        else
            pair = BLENDER.pair2;
        end

        B_used = pair.B_m;
        camXL = -B_used/2;
        camXR = +B_used/2;

        msg.Value = string(sprintf( ...
            "Rendering... Ztrue=%.2f m | Stereo pair %d (%s + %s) B=%.3f", ...
            Ztrue_m, pairIdx, pair.camL, pair.camR, B_used));
        drawnow;

        zBall = pair.camBaseZ - Ztrue_m;

        try
            disp("STEP 1/3: move ball");
            drawnow;
            blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
                0, -5, 1, ...
                BLENDER.ballPitch, BLENDER.ballRoll, BLENDER.ballYaw, ...
                BLENDER.ballName);
            disp("STEP 1 DONE");
        catch ME
            infoLines = [
                "Error moving/reading Ball from Blender."
                "Check BLENDER.ballName matches the ball object name in Blender."
                " "
                string(ME.message)
            ];
            return;
        end

        try
            disp("STEP 2/3: render LEFT");
            drawnow;
            IL = blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
                pair.camBaseX+camXL, pair.camBaseY, pair.camBaseZ, ...
                pair.camPitch, pair.camRoll, pair.camYaw, ...
                pair.camL);
            disp("STEP 2 DONE");
        catch ME
            infoLines = [
                "Error rendering Left image from Blender."
                "Check pair camera names (Cam1/Cam3) match Blender."
                " "
                string(ME.message)
            ];
            IL = [];
            return;
        end

        try
            disp("STEP 3/3: render RIGHT");
            drawnow;
            IR = blenderLinkRetry(c, BLENDER.width, BLENDER.height, ...
                pair.camBaseX+camXR, pair.camBaseY, pair.camBaseZ, ...
                pair.camPitch, pair.camRoll, pair.camYaw, ...
                pair.camR);
            disp("STEP 3 DONE");
        catch ME
            infoLines = [
                "Error rendering Right image from Blender."
                "Check pair camera names (Cam2/Cam4) match Blender."
                " "
                string(ME.message)
            ];
            IR = [];
            return;
        end

        infoLines = [
            "Blender renders OK."
            sprintf("Stereo pair %d | %s | %s", pairIdx, pair.camL, pair.camR)
            sprintf("Ball (world) approx (0,0,%.2f)  [Ztrue=%.2f]", zBall, Ztrue_m)
            sprintf("CamL=(%.3f,%.3f,%.3f)  CamR=(%.3f,%.3f,%.3f)", ...
                pair.camBaseX+camXL, pair.camBaseY, pair.camBaseZ, ...
                pair.camBaseX+camXR, pair.camBaseY, pair.camBaseZ)
        ];
    end

    function idx = chooseActiveStereoPair(params, xBall, yBall)
        % Manual: force pair 1 or 2. Auto: ball Y vs divider (pair1 @ Y~-13, pair2 @ Y~-2).
        % More negative Y -> pair 1; less negative -> pair 2.
        mode = string(params.stereoMode);
        divY = params.courtDividerY;
        if contains(mode, "Pair 1")
            idx = 1;
        elseif contains(mode, "Pair 2")
            idx = 2;
        else
            if ~isfinite(yBall)
                yBall = 0;
            end
            if ~isfinite(xBall)
                xBall = 0;
            end
            if yBall < divY
                idx = 1;
            else
                idx = 2;
            end
        end
    end

%% -------------------- Detection --------------------
function [centroid, dbg, rEq] = detectBallCentroid(I, params)

dbg = strings(0,1);
centroid = [NaN NaN];
rEq = NaN;

if isempty(I)
    dbg(end+1) = "Empty image.";
    return;
end

function y = normalize01(x)
    x = double(x);
    if isempty(x) || all(~isfinite(x))
        y = zeros(size(x));
        return;
    end
    x(~isfinite(x)) = min(x(isfinite(x)));
    mn = min(x); mx = max(x);
    if mx <= mn
        y = ones(size(x));
    else
        y = (x - mn) / (mx - mn);
    end
end

function [xc,yc,r,ok] = fitCircleLS(x, y)
    % Algebraic least-squares circle fit
    x = double(x(:)); y = double(y(:));
    ok = false; xc=NaN; yc=NaN; r=NaN;
    if numel(x) < 3, return; end

    A = [2*x, 2*y, ones(size(x))];
    b = x.^2 + y.^2;

    p = A\b;              % [xc; yc; c] where c = r^2 - xc^2 - yc^2
    xc = p(1); yc = p(2);
    c  = p(3);
    r2 = c + xc^2 + yc^2;
    if ~isfinite(r2) || r2 <= 0, return; end
    r = sqrt(r2);
    ok = isfinite(xc) && isfinite(yc) && isfinite(r);
end

function [cent, bestScore, dbgLocal, rEq] = pickBestFromBW(BW0, label, seLineKill, areaExp, Iwork, params)
    % Select best 'ball-shaped' blob from BW0 using expected-size + shape + proximity scoring.
    % Then refine the center with a perimeter circle-fit (more accurate than region centroid).
    dbgLocal = strings(0,1);
    cent = [NaN NaN];
    bestScore = -Inf;
    rEq = NaN;
    if ~any(BW0(:))
        dbgLocal(end+1) = label + ": BW0 empty";
        return;
    end

    BW1 = imopen(BW0, seLineKill);
    BW1 = bwareaopen(BW1, max(10, round(0.05*areaExp)));
    BW1 = imfill(BW1, 'holes');

    if ~any(BW1(:))
        dbgLocal(end+1) = label + ": empty after cleanup";
        return;
    end

    st = regionprops(BW1,'Area','Centroid','Perimeter','Eccentricity','Solidity');
    if isempty(st)
        dbgLocal(end+1) = label + ": no blobs";
        return;
    end

    AA   = [st.Area];
    PP   = [st.Perimeter];
    EE   = [st.Eccentricity];
    SS   = [st.Solidity];
    CCc  = 4*pi*AA ./ max(PP.^2, eps);

    % Prefer blobs within a reasonable expected area window first
    % If none fit then fall back to scoring all blobs
    minA = 0.25 * areaExp;
    maxA = 4.00 * areaExp;
    idxCand = find((AA >= minA) & (AA <= maxA));
    if isempty(idxCand)
        idxCand = 1:numel(st);
        dbgLocal(end+1) = label + ": no blobs in expected area window; scoring all.";
    end

    AAc  = AA(idxCand);
    EEc  = EE(idxCand);
    SSc  = SS(idxCand);
    CCcc = CCc(idxCand);
    C2   = reshape([st(idxCand).Centroid],2,[])';

    areaRatio2 = AAc / areaExp;
    areaScore2 = 1 - min(abs(log(max(areaRatio2, 0.1))), 2)/2;
    circScore2 = normalize01(CCcc(:));
    solScore2  = normalize01(SSc(:));
    eccScore2  = 1 - normalize01(EEc(:));

    % Radius-from-area match (reject lines on court being read as blobs)
    rExp = sqrt(areaExp/pi);
    rA   = sqrt(AAc(:)/pi);
    sigR = max(2, 0.20*rExp);
    rMatch = exp(-((rA - rExp).^2) ./ (2*sigR^2));

    if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
        ref2 = params.refCentroid(:).';
    else
        ref2 = [size(Iwork,2)/2, size(Iwork,1)/2];
    end

    d22 = sum((C2 - ref2).^2, 2);
    prox2 = normalize01(1 ./ (1 + d22));

    % (ball is near center / previous frame)
    sc = 0.22*circScore2 + 0.18*solScore2 + 0.10*eccScore2 + 0.25*areaScore2(:) + 0.10*rMatch(:) + 0.15*prox2;
    [bestScore, kLocal] = max(sc);
    cent0 = C2(kLocal,:);
    rEq = sqrt(AAc(kLocal) / pi);

    % Circle-fit refine on the blob perimeter
    CC = bwconncomp(BW1);
    % Map local best back to component index in BW1
    % regionprops ordering matches connected-components ordering.
    k2 = idxCand(kLocal);
    if numel(CC.PixelIdxList) >= k2
        BWbest = false(size(BW1));
        BWbest(CC.PixelIdxList{k2}) = true;
        Bper = bwperim(BWbest);
        [yy, xx] = find(Bper);
        if numel(xx) >= 20
            [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
            if okFit
                cent = [xc yc];
                rEq = rc;
                dbgLocal(end+1) = string(sprintf("%s: bestScore=%.3f (circleFit r=%.1f)", label, bestScore, rc));
                return;
            end
        end
    end

    cent = cent0;
    dbgLocal(end+1) = string(sprintf("%s: bestScore=%.3f (no circleFit)", label, bestScore));
end

function centOut = refineCenterWithLocalGray(Ifull, centIn, params)
    % Make a rough centroid by fitting a circle on edges in a small
    % grayscale ROI around it. Using edges (rather than a blob)
    % keeps the fitted center very close to the visual center of the ball
    centOut = centIn;
    if any(~isfinite(centIn)) || isempty(Ifull)
        return;
    end

    H = size(Ifull,1);
    W = size(Ifull,2);

    % Expected radius from Z (if missing increase distance) 
    % used to gate which edge pixels belong to the ball
    Rball_m = 0.033;
    Zuse = params.Ztrue_m;
    if ~isfinite(Zuse) || Zuse <= 0, Zuse = 4.0; end
    fpx  = params.f_px;
    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(H,W)));

    cx = centIn(1);
    cy = centIn(2);

    % Tight ROI around expected ball radius
    % straight court edges which bias the circle fit
    roiHalf = max(16, round(1.0*rExp));
    x1 = max(1, floor(cx - roiHalf));
    x2 = min(W, ceil (cx + roiHalf));
    y1 = max(1, floor(cy - roiHalf));
    y2 = min(H, ceil (cy + roiHalf));

    if (x2-x1) < 10 || (y2-y1) < 10
        return;
    end

    Iroi = im2double(rgb2gray(Ifull(y1:y2, x1:x2, :)));
    Iroi = imgaussfilt(Iroi, 0.6);

    % Use Canny edges of the ball silhouette, less sensitive to
    % interior shading and you get a better geometric center
    E = edge(Iroi,'Canny');
    [yy, xx] = find(E);

    if numel(xx) < 20
        return;
    end

    % Keep only edge pixels whose radius from the rough center is close to
    % the expected ball radius, this should remove court lines and interior
    % edges
    cxR = cx - (x1 - 1);
    cyR = cy - (y1 - 1);
    rr = sqrt((xx - cxR).^2 + (yy - cyR).^2);
    inBand = (rr > 0.7*rExp) & (rr < 1.3*rExp);
    xx = xx(inBand);
    yy = yy(inBand);
    if numel(xx) < 20
        return;
    end

    [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
    if ~okFit
        return;
    end

    centOut = [xc + x1 - 1, yc + y1 - 1];
end

% Optional crop
if params.useTopCrop
    H = size(I,1);
    Hc = max(1, round(params.topFrac * H));
    Iwork = I(1:Hc,:,:);
else
    Iwork = I;
end

% When a reference centroid exists only search near it
% This prevents far-Z (small ball) detections from jumping to court lines
xOff = 0; yOff = 0;
method = params.method;
if ~strcmp(method,"YCbCrNeutral") && ...
        isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
    ref = params.refCentroid(:).';
    H = size(Iwork,1);
    W = size(Iwork,2);

    % ROI half-width from image size (no Z-depth assumption)
    rExpRoi = max(12, min(78, round(0.043*min(H,W))));
    if isfield(params,'roiHalfPx') && isfinite(params.roiHalfPx) && params.roiHalfPx > 0
        roiHalf = round(params.roiHalfPx);
    else
        roiHalf = max(90, round(4.0*rExpRoi));
    end

    x1 = max(1, floor(ref(1) - roiHalf));
    x2 = min(W, ceil (ref(1) + roiHalf));
    y1 = max(1, floor(ref(2) - roiHalf));
    y2 = min(H, ceil (ref(2) + roiHalf));

    % Only crop if it actually reduces the search region
    if (x1 > 1) || (y1 > 1) || (x2 < W) || (y2 < H)
        Iwork = Iwork(y1:y2, x1:x2, :);
        xOff = x1 - 1;
        yOff = y1 - 1;
        params.refCentroid = ref - [xOff yOff];
        dbg(end+1) = string(sprintf("ROI crop: [%d..%d]x[%d..%d] (off=%d,%d)", x1,x2,y1,y2,xOff,yOff));
    end
end

switch method

% ============================================================
case "YCbCrTennis"
    % Yellow ball in YCbCr; suppress white lines + court green by chroma rules
    dbg = string.empty(0,1);
    RGB = im2double(Iwork);
    YC = rgb2ycbcr(RGB);
    Y = YC(:,:,1); Cb = YC(:,:,2); Cr = YC(:,:,3);

    tW = 0.08;
    if isfield(params,'whiteChromaTol') && isfinite(params.whiteChromaTol)
        tW = params.whiteChromaTol;
    end
    isWhite = (Y >= 0.71) & (abs(Cb - 0.5) < tW) & (abs(Cr - 0.5) < tW);

    tc = 0.12;
    if isfield(params,'tCbCr') && isfinite(params.tCbCr) && params.tCbCr > 0
        tc = params.tCbCr;
    end
    isCourtGreen = (Y >= 0.07) & (Y <= 0.83) ...
        & (Cb >= (0.47 + 0.45*tc)) & (Cr <= (0.53 - 0.28*tc)) ...
        & ((Cb - Cr) > 0.022) & ~isWhite;

    crMin = 0.502 + 1.05 * tc;
    crMin = min(crMin, 0.63);
    isBall = (Y >= 0.17) & (Cr >= crMin) & (Cr > Cb + 0.007) & ~isWhite & ~isCourtGreen;

    BW0 = isBall;
    Ht = size(Iwork,1); Wt = size(Iwork,2);
    rGuess = max(6, min(round(0.038 * min(Ht, Wt)), 92));
    areaExp = pi * rGuess^2;
    seDisk = strel('disk', max(2, round(0.22 * rGuess)));

    [centroid, ~, dbg2, rEq] = pickBestFromBW(BW0, "YCbCrTennis", seDisk, areaExp, Iwork, params);
    dbg = [dbg; dbg2];
    if all(isfinite(centroid))
        centroid = centroid + [xOff, yOff];
    else
        rEq = NaN;
    end
    return;

% ============================================================
case "GrayThresh"
    % Pipeline: (1) GrayThresh coarse blob -> image side (left/right) + ROI
    %           (2) imfindcircles in ROI with Z-based radius band
    %           (3) fallback: perimeter LS circle-fit or blob centroid
    dbg = string.empty(0,1);

    Ig = im2double(rgb2gray(Iwork));
    Ig = imgaussfilt(Ig, 0.8);

    Rball_m = 0.033;
    Zuse = params.Ztrue_m;
    if ~isfinite(Zuse) || Zuse <= 0, Zuse = 4.0; end
    fpx  = params.f_px;

    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(size(Ig))));
    areaExp = pi * rExp^2;

    minAexp = 0.20 * areaExp;
    maxAexp = 6.00 * areaExp;

    if isfield(params,'minBlobArea') && isfinite(params.minBlobArea) && params.minBlobArea > 0
        minAexp = max(minAexp, params.minBlobArea);
    end
    if isfield(params,'maxBlobArea') && isfinite(params.maxBlobArea) && params.maxBlobArea > 0
        maxAexp = min(maxAexp, params.maxBlobArea);
    end

    if isfield(params,'grayThresh') && isfinite(params.grayThresh) && params.grayThresh > 0
        tDark = params.grayThresh;
        dbg(end+1) = string(sprintf("GrayThresh manual dark: t=%.3f", tDark));
    else
        tDark = graythresh(1 - Ig);
        dbg(end+1) = string(sprintf("GrayThresh otsu(inv): t=%.3f", tDark));
    end

    BW = (1 - Ig) > tDark;

    diskRad = max(2, round(0.20*rExp));
    if rExp < 25
        diskRad = min(diskRad, max(2, floor(rExp/4)));
    end
    seLineKill = strel('disk', diskRad);
    BW = imopen(BW, seLineKill);
    BW = bwareaopen(BW, max(10, round(0.05*areaExp)));
    BW = imfill(BW, 'holes');

    if ~any(BW(:))
        dbg(end+1) = "GrayThresh: BW empty after cleanup.";
        centroid = [NaN NaN];
        return;
    end

    CC = bwconncomp(BW);
    S  = regionprops(CC, 'Area','Centroid','Perimeter','Eccentricity','Solidity');
    if isempty(S)
        dbg(end+1) = "GrayThresh: no components.";
        centroid = [NaN NaN];
        return;
    end

    A = [S.Area];
    P = [S.Perimeter];
    ecc = [S.Eccentricity];
    sol = [S.Solidity];
    circ = 4*pi*A ./ max(P.^2, eps);

    idxCand = find((A >= minAexp) & (A <= maxAexp));
    if isempty(idxCand)
        idxCand = 1:numel(S);
        dbg(end+1) = "GrayThresh: no blobs in expected size window; scoring all blobs.";
    end

    Aidx   = A(idxCand);
    circC  = circ(idxCand);
    solC   = sol(idxCand);
    eccC   = ecc(idxCand);
    areaRatio = Aidx / areaExp;
    areaScore = 1 - min(abs(log(max(areaRatio, 0.1))), 2)/2;
    circScore = normalize01(circC(:));
    solScore  = normalize01(solC(:));
    eccScore  = 1 - normalize01(eccC(:));

    if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
        ref = params.refCentroid(:).';
    else
        ref = [size(Iwork,2)/2, size(Iwork,1)/2];
    end
    Ccand = reshape([S(idxCand).Centroid],2,[])';
    d2 = sum((Ccand - ref).^2, 2);
    proxScore = normalize01(1 ./ (1 + d2));

    score = 0.30*circScore + 0.25*solScore + 0.20*eccScore + 0.20*areaScore(:) + 0.05*proxScore;
    [~,kBest] = max(score);
    iBest = idxCand(kBest);

    cCoarse = S(iBest).Centroid;
    Wimg = size(Iwork,2);
    if cCoarse(1) < Wimg/2
        dbg(end+1) = "GrayThresh: ball in LEFT half of image (u < W/2).";
    else
        dbg(end+1) = "GrayThresh: ball in RIGHT half of image (u >= W/2).";
    end

    roiHalf = max(24, round(2.8*rExp));
    cxR = cCoarse(1); cyR = cCoarse(2);
    x1 = max(1, floor(cxR - roiHalf));
    x2 = min(Wimg, ceil(cxR + roiHalf));
    y1 = max(1, floor(cyR - roiHalf));
    y2 = min(size(Iwork,1), ceil(cyR + roiHalf));
    if (x2-x1) < 16 || (y2-y1) < 16
        dbg(end+1) = "ROI too small for circles; fallback.";
    else
        Iroi = Iwork(y1:y2, x1:x2, :);
        Igroi = mat2gray(rgb2gray(Iroi));
        % Tight radius band from physics: r ≈ f*R_ball / Z
        rLo = max(4, round(0.62 * rExp));
        rHi = min(round(1.38 * rExp), floor(0.48 * min(size(Igroi))));
        if rHi <= rLo + 1
            rHi = rLo + 3;
        end
        radRange = [rLo, rHi];
        dbg(end+1) = string(sprintf("imfindcircles radius band [%.0f .. %.0f] px (rExp=%.1f)", rLo, rHi, rExp));

        [centBright, radBright, metBright] = imfindcircles(Igroi, radRange, ...
            'ObjectPolarity','bright','Sensitivity',0.92,'EdgeThreshold',0.08);
        [centInv, radInv, metInv] = imfindcircles(1-Igroi, radRange, ...
            'ObjectPolarity','bright','Sensitivity',0.92,'EdgeThreshold',0.08);

        centers = [centBright; centInv];
        radii   = [radBright;  radInv];
        metric  = [metBright;  metInv];

        if ~isempty(centers)
            cenLoc = cCoarse - [x1-1, y1-1];
            d2c = sum((centers - cenLoc).^2, 2);
            [~,kc] = min(d2c);
            centroid = centers(kc,:) + [x1-1, y1-1] + [xOff, yOff];
            dbg(end+1) = string(sprintf("Circles: r=%.1f px metric=%.3f (in ROI)", radii(kc), metric(kc)));
            return;
        end
        dbg(end+1) = "imfindcircles: no circle in ROI; fallback to GrayThresh refine.";
    end

    BWbest = false(size(BW));
    BWbest(CC.PixelIdxList{iBest}) = true;
    Bper = bwperim(BWbest);
    [yy, xx] = find(Bper);
    if numel(xx) >= 20
        [xc, yc, rc, okFit] = fitCircleLS(xx, yy);
        if okFit
            centroid = [xc+xOff yc+yOff];
            dbg(end+1) = string(sprintf("Fallback: LS circle-fit (rFit=%.1f, rExp=%.1f).", rc, rExp));
            return;
        end
    end

    centroid = S(iBest).Centroid + [xOff yOff];
    dbg(end+1) = "Fallback: region centroid.";
    return;
% ============================================================
    case "YCbCrNeutral"

    % Combined bright/dark blob detector
    % Works when the ball is brighter or darker than the background.

    Ig = im2double(rgb2gray(Iwork));
    Iinv = 1 - Ig;

    % --- Bright-ball mask ---
    if isfield(params,'yMin') && params.yMin > 0
        tBright = params.yMin;
    else
        tBright = graythresh(Ig);
    end
    BWb = Ig > tBright;

    % --- Dark-ball mask (threshold on inverted image) ---
    tDark = graythresh(Iinv);
    BWd = Iinv > tDark;   % Ig < (1 - tDark)

    BW = BWb | BWd;

    % Remove specks and thin lines.
    BW = bwareaopen(BW, 50);
    BW = imopen(BW, strel('disk',2));
    BW = imclose(BW, strel('disk',3));
    BW = imfill(BW,'holes');

    if ~any(BW(:))
        dbg = ["YCbCrNeutral (bright+dark blob): BW empty after cleanup."];
        centroid = [NaN NaN];
        return;
    end

    stats = regionprops(BW,'Area','Centroid','Eccentricity','Solidity');
    if isempty(stats)
        dbg = ["YCbCrNeutral (bright+dark blob): no components."];
        centroid = [NaN NaN];
        return;
    end

    A   = [stats.Area];
    ecc = [stats.Eccentricity];
    sol = [stats.Solidity];

    % Prefer large, round, solid blob near image center
    ok = (ecc < 0.9) & (sol > 0.7);
    if ~any(ok)
        ok = true(size(A));
    end

    statsOK = stats(ok);
    Aok = A(ok);
    Cok = reshape([statsOK.Centroid],2,[])';
    imgCenter = [size(Iwork,2)/2, size(Iwork,1)/2];
    d2 = sum((Cok - imgCenter).^2, 2);

    % score the area and proximity to center.
    areaScore = Aok(:) / max(Aok);
    proxScore = 1 ./ (1 + d2);
    proxScore = proxScore / max(proxScore);
    score = 0.6*areaScore + 0.4*proxScore;

    [~,kBest] = max(score);
    c = Cok(kBest,:);
    centroid = c + [xOff yOff];
    dbg(end+1) = sprintf("YCbCrNeutral (bright+dark blob): tB=%.3f tD=%.3f area=%.0f", ...
                         tBright, tDark, Aok(kBest));
    return;

% ============================================================
    case "Circles"
        % Refine centroid locally using imfindcircles around a YCbCr-provided ROI.
        dbg = string.empty(0,1);

        Ig = mat2gray(rgb2gray(Iwork));
        Ig = imgaussfilt(Ig, 0.8);

        if isfield(params,'expectedRadiusPx') && isfinite(params.expectedRadiusPx) && params.expectedRadiusPx > 0
            rExp = params.expectedRadiusPx;
        else
            rExp = 4;
        end

        rExp = max(2, min(rExp, 0.25 * min(size(Ig))));
        rLo = max(2, floor(0.65 * rExp));
        rHi = max(rLo + 2, ceil(1.40 * rExp));
        rHi = min(rHi, floor(0.45 * min(size(Ig))));
        dbg(end+1) = sprintf("Circles: radius band [%d %d], rExp=%.1f", rLo, rHi, rExp);

        [centBright, radBright, metBright] = imfindcircles(Ig, [rLo rHi], ...
            'ObjectPolarity','bright','Sensitivity',0.92,'EdgeThreshold',0.08);

        IgInv = 1 - Ig;
        [centInv, radInv, metInv] = imfindcircles(IgInv, [rLo rHi], ...
            'ObjectPolarity','bright','Sensitivity',0.92,'EdgeThreshold',0.08);

        centers = [centBright; centInv];
        radii   = [radBright; radInv];
        metric  = [metBright; metInv];

        if isempty(centers)
            dbg(end+1) = "Circles: no candidates.";
            centroid = [NaN NaN];
            return;
        end

        if isfield(params,'refCentroid') && ~isempty(params.refCentroid) && all(isfinite(params.refCentroid))
            ref = params.refCentroid(:).';
            d2 = sum((centers - ref).^2, 2);
            radiusPenalty = abs(radii - rExp) ./ max(rExp,1);
            score = metric - 0.01*d2 - 0.4*radiusPenalty;
            [~,k] = max(score);
        else
            [~,k] = max(metric);
        end

        centroid = centers(k,:) + [xOff yOff];
        rEq = radii(k);
        dbg(end+1) = sprintf("Circles: picked r=%.1f metric=%.3f", radii(k), metric(k));
        return;

            otherwise
                dbg = ["Unknown method."];
                return;
end

% ============================================================
% Scoring (GrayThresh + YCbCr)
% ============================================================

stats = regionprops(BW,'Area','Centroid','Perimeter','Eccentricity','Solidity');

if isempty(stats)
    dbg(end+1) = "No blobs after filtering.";
    return;
end

A   = [stats.Area];
P   = [stats.Perimeter];
ecc = [stats.Eccentricity];
sol = [stats.Solidity];
circ = 4*pi*A ./ max(P.^2, eps);

% Shape gate: use stricter 'ball-shaped' constraints for YCbCr (helps reject court lines)
if isfield(params,'method') && params.method == "YCbCrNeutral"
    ok = (circ > 0.65) & (ecc < 0.85) & (sol > 0.80);
else
    ok = (circ > 0.35) & (ecc < 0.95) & (sol > 0.6);
end
if ~any(ok)
    ok = true(size(A));
end

idx = find(ok);
if isempty(idx)
    dbg(end+1) = "No blobs after filtering.";
    return;
end
C = reshape([stats(idx).Centroid],2,[])';
Aok = A(idx);
circOk = circ(idx);

scoreShape = circOk(:) .* sqrt(Aok(:));

if isfield(params,'refCentroid') && ~isempty(params.refCentroid)
    ref = params.refCentroid(:).';
else
    ref = [size(Iwork,2)/2, size(Iwork,1)/2];
end

d2 = sum((C - ref).^2,2);
scoreProx = 1 ./ (1 + d2);

% Prefer blobs near expected area (useful for Ycrbr at z=4 and z=5)
scoreArea = ones(size(Aok(:)));
if isfield(params,'Ztrue_m') && isfield(params,'f_px') && isfinite(params.Ztrue_m) && params.Ztrue_m > 0
    Zuse = params.Ztrue_m;
    fpx  = params.f_px;
    Rball_m = 0.033;
    rExp = (fpx * Rball_m) / Zuse;
    rExp = max(6, min(rExp, 0.45*min(size(Iwork,1), size(Iwork,2))));
    areaExp = pi * rExp^2;
    areaRatio = Aok(:) / areaExp;
    scoreArea = 1 - min(abs(log(max(areaRatio, 0.1))), 2)/2;
end

nShape = max(scoreShape);
nProx  = max(scoreProx);
if nShape < eps, nShape = 1; end
if nProx  < eps, nProx  = 1; end
score = 0.5*(scoreShape/nShape) + 0.25*(scoreProx/nProx) + 0.25*scoreArea;
[~,k] = max(score);

centroid = C(k,:) + [xOff yOff];
dbg(end+1) = "Blob scoring complete";

end

%% -------------------- TCP Helpers --------------------
    function ok = ensureClient()
        ok = true;
        try
            if isempty(client) || ~isvalid(client)
                disp("Connecting to Blender at 127.0.0.1:55001");
                client = tcpclient(BLENDER.server_ip, BLENDER.server_port, 'Timeout', 20);
            end
        catch ME
            ok = false;
            msg.Value = [
                "Failed to create tcpclient."
                string(ME.message)
                "Make sure Blender server is running (Start Server)."
            ];
        end
    end

    function safeUpdateOnce()
        try
            updateOnce();
        catch ME
            msg.Value = [
                "safeUpdateOnce error:"
                string(ME.message)
            ];
        end
    end

    %% -------------------- Final-project helpers (LEDs & playback stubs) --------------------
    function setLEDState(state)
        % state: "IN" | "OUT" | "NET" | "idle"
        LED.state = string(state);
        switch LED.state
            case "IN"
                try
                    stop(ledTimer);
                catch
                end
                LED.redOn = false;
                lblLEDGreen.Text = "LED IN: ON";
                lblLEDRed.Text   = "LED OUT: OFF";
            case "OUT"
                lblLEDGreen.Text = "LED IN: OFF";
                LED.redOn = true;
                lblLEDRed.Text   = "LED OUT: ON (blinking)";
                try
                    start(ledTimer);
                catch
                end
            case "NET"
                try
                    stop(ledTimer);
                catch
                end
                LED.redOn = true;
                lblLEDGreen.Text = "LED IN: OFF";
                lblLEDRed.Text   = "LED OUT: SOLID (NET)";
            otherwise
                try
                    stop(ledTimer);
                catch
                end
                LED.redOn = false;
                lblLEDGreen.Text = "LED IN: OFF";
                lblLEDRed.Text   = "LED OUT: OFF";
        end
    end

    function updateLEDBlink()
        if LED.state ~= "OUT"
            return;
        end
        LED.redOn = ~LED.redOn;
        if LED.redOn
            lblLEDRed.Text = "LED OUT: ON (blinking)";
        else
            lblLEDRed.Text = "LED OUT: OFF (blinking)";
        end
    end

    function runServesAndVolleys()
        % Playback and classify 5 serves and 5 volleys from .dat files
        shotFiles = [ "serve1.dat","serve2.dat","serve3.dat","serve4.dat","serve5.dat", ...
                      "volley1.dat","volley2.dat","volley3.dat","volley4.dat","volley5.dat" ];

        shotSummary = strings(0,1);

        for k = 1:numel(shotFiles)
            fname = shotFiles(k);
            traj = loadTrajectoryDat(fname);
            if isempty(traj)
                continue;
            end

            % Classify shot and estimate restitution
            [result, bounceIdx, eRest] = classifyShotAndRestitution(traj);
            lastTraj   = traj;
            lastResult = result;

            % Update LEDs
            setLEDState(result.type);

            % Replay trajectory with landing marker
            replayTrajectory(traj, sprintf("%s (%s)", fname, result.type), bounceIdx);

            % Per-shot message (column string array, accepted by uitextarea)
            msg.Value = [
                "Shot: "   + string(fname);
                "Result: " + string(result.type);
                string(sprintf("Landing (x,y) = (%.2f, %.2f) m", result.landXY(1), result.landXY(2)));
                string(sprintf("Cleared net: %d", result.clearedNet));
                string(sprintf("Estimated restitution e = %.3f", eRest));
                "";
                "Press 'Instant Replay' to view last trajectory again."
            ];

            % Append to overall summary
            shotSummary(end+1) = sprintf("%-10s : %-4s  land=(%.2f,%.2f)  e=%.3f", ...
                                         fname, result.type, result.landXY(1), result.landXY(2), eRest);

            drawnow;
        end

        if ~isempty(shotSummary)
            % Summary as column string array for uitextarea
            msg.Value = [
                "Serve/Volley sweep complete.";
                "Shot summary:";
                shotSummary(:);
                "";
                "Use 'Instant Replay' to review the last shot."
            ];
        end
    end

    function runInstantReplay()
        if isempty(lastTraj)
            msg.Value = ["No trajectory stored yet for instant replay."];
            return;
        end
        replayTrajectory(lastTraj, "Instant Replay", NaN);
        if isfield(lastResult,'type')
            setLEDState(lastResult.type);
        end
    end

    function runRestitutionTest()
        if isempty(lastTraj)
            msg.Value = [
                "Restitution test uses the last serve/volley trajectory."
                "Run 'Serves/Volleys Playback' first."
            ];
            return;
        end
        [~, bounceIdx, eRest] = classifyShotAndRestitution(lastTraj);
        msg.Value = [
            "Restitution estimate from last trajectory:"
            sprintf("e = %.3f", eRest)
        ];
        % Highlight bounce point if available
        if ~isnan(bounceIdx) && bounceIdx >= 1 && bounceIdx <= numel(lastTraj.x)
            hLand.XData = lastTraj.x(bounceIdx);
            hLand.YData = lastTraj.y(bounceIdx);
            hLand.ZData = lastTraj.z(bounceIdx);
        end
        drawnow;
    end

    function runAccVsBall()
        msg.Value = [
            "Accuracy vs Ball Position: run Single Frame Test at several Blender ball depths."
            "Log X/Y/Z vs ground truth manually or extend this callback to sweep Z in Blender."
        ];
    end

    function runAccVsCamera()
        msg.Value = [
            "Camera motion / calibration: change Baseline B or f (pixels) in Controls,"
            "then run Single Frame Test and compare X/Y/Z. (3–5 m benchmark plot removed.)"
        ];
    end

    %% ---- Trajectory, .dat IO, in/out/net classification and restitution ----
    function traj = loadTrajectoryDat(fname)
        traj = [];
        if ~isfile(fname)
            msg.Value = [sprintf("Trajectory file not found: %s", fname)];
            return;
        end
        data = readmatrix(fname);
        if size(data,2) < 3
            msg.Value = [sprintf("File %s does not have at least 3 columns.", fname)];
            return;
        end
        if size(data,2) == 3
            n = size(data,1);
            t = (0:n-1)'/60; % assume 60 Hz if no time column
            x = data(:,1); y = data(:,2); z = data(:,3);
        else
            t = data(:,1);
            x = data(:,2); y = data(:,3); z = data(:,4);
        end
        traj.t = t;
        traj.x = x;
        traj.y = y;
        traj.z = z;
    end

    function [result, bounceIdx, eRest] = classifyShotAndRestitution(traj)
        % Simple court model (meters, approximate doubles court)
        COURT.xMin = -8.97; COURT.xMax = 9.23;
        COURT.yMin = -12.79;  COURT.yMax = 12.61;
        COURT.netY = -0.091225;
        COURT.netHeight = 0.914;  % ~0.91 m

        x = traj.x; y = traj.y; z = traj.z; t = traj.t;

        % Find bounce: first sample near ground with downward->upward vz
        vz = [diff(z)./diff(t); 0];
        bounceIdx = NaN;
        for k = 2:numel(z)-1
            if z(k) <= 0.1 && vz(k-1) < 0 && vz(k+1) > 0
                bounceIdx = k;
                break;
            end
        end
        if isnan(bounceIdx)
            [~,bounceIdx] = min(z); % fallback: minimum height
        end

        xLand = x(bounceIdx);
        yLand = y(bounceIdx);

        % Net crossing: first crossing of COURT.netY
        clearedNet = true;
        yRel = y - COURT.netY;
        idxCross = find(yRel(1:end-1).*yRel(2:end) <= 0, 1, 'first');
        if ~isempty(idxCross)
            t1 = t(idxCross);   t2 = t(idxCross+1);
            z1 = z(idxCross);   z2 = z(idxCross+1);
            if t2 > t1
                zNet = z1 + (z2-z1)*(0 - yRel(idxCross))/(yRel(idxCross+1)-yRel(idxCross) + eps);
            else
                zNet = max(z1,z2);
            end
            clearedNet = zNet > COURT.netHeight;
        end

        inBounds = (xLand >= COURT.xMin) && (xLand <= COURT.xMax) && ...
                   (yLand >= COURT.yMin) && (yLand <= COURT.yMax);

        if ~clearedNet
            type = "NET";
        elseif inBounds
            type = "IN";
        else
            type = "OUT";
        end

        % Restitution from vertical velocity around bounce using small windows
        eRest = NaN;
        if numel(z) >= 6
            % Use a few samples before and after the bounce to get robust slopes
            k0 = max(2, bounceIdx-3);
            k1 = max(1,  bounceIdx-1);
            k2 = min(numel(z), bounceIdx+3);
            k3 = min(numel(z), bounceIdx+1);

            % Pre-bounce window [k0..k1]
            tb = t(k0:k1);
            zb = z(k0:k1);
            if numel(tb) >= 2
                pb = polyfit(tb, zb, 1);   % zb ≈ pb(1)*t + pb(2)
                vBefore = pb(1);
            else
                vBefore = NaN;
            end

            % Post-bounce window [k3..k2]
            ta = t(k3:k2);
            za = z(k3:k2);
            if numel(ta) >= 2
                pa = polyfit(ta, za, 1);
                vAfter = pa(1);
            else
                vAfter = NaN;
            end

            if isfinite(vBefore) && isfinite(vAfter) && abs(vBefore) > 1e-3
                eRest = abs(vAfter / vBefore);
            end
        end

        result.type      = type;
        result.landXY    = [xLand, yLand];
        result.clearedNet = clearedNet;
    end

    function replayTrajectory(traj, plotTitle, bounceIdx)
        % Safely redraw full trajectory and current point without animation.
        % If UI elements have been destroyed (e.g., figure closed), exit quietly.
        try
            if isempty(ax3) || ~isvalid(ax3) || isempty(hTraj) || ~isvalid(hTraj) || ...
               isempty(hPt3) || ~isvalid(hPt3) || isempty(hLand) || ~isvalid(hLand)
                return;
            end
        catch
            return;
        end

        x = traj.x; y = traj.y; z = traj.z;
        set(hTraj,'XData',x,'YData',y,'ZData',z);

        % Current point = last sample in trajectory
        if ~isempty(x)
            hPt3.XData = x(end);
            hPt3.YData = y(end);
            hPt3.ZData = z(end);
        end

        % Optional bounce/landing marker
        if ~isnan(bounceIdx) && bounceIdx >= 1 && bounceIdx <= numel(x)
            hLand.XData = x(bounceIdx);
            hLand.YData = y(bounceIdx);
            hLand.ZData = z(bounceIdx);
        else
            hLand.XData = NaN; hLand.YData = NaN; hLand.ZData = NaN;
        end

        title(ax3, plotTitle);
        drawnow;
    end
end