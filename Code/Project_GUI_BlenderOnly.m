function Project_GUI_BlenderOnly
%% Project GUI + Blender Only (updated)
% MATLAB handles GUI + Blender rendering.
% C++ backend handles detection, stereo-pair selection, and triangulation.
%
% Updates in this version:
%   - 1280x720 render size (closer to planned 720p cameras)
%   - corrected default backend executable path for triangulation.exe
%   - fixed output folder under Project_ESD\tracker_runs
%   - larger GUI layout so buttons remain visible
%   - lower ball Z placement (uses slider value directly as Blender world Z)
%   - config now sends rig positions / pitch to backend
%   - defaults updated to match four-camera scene comments more closely

clc; close all;

%% -------------------- Blender Server Settings --------------------
BLENDER.server_ip   = '127.0.0.1';
BLENDER.server_port = 55001;
BLENDER.ballName    = "tennisBall";

% Scene comments from original project indicated camera centers at Z=12 and
% 3 m baseline: (-1.5,+1.5) around X=0.
BLENDER.pair1.camL = "Camera1";
BLENDER.pair1.camR = "Camera2";
BLENDER.pair1.camBaseX = 0.0;
BLENDER.pair1.camBaseY = -15.0;
BLENDER.pair1.camBaseZ = 12.0;
BLENDER.pair1.B_m      = 3.0;
BLENDER.pair1.camPitch = 30.0;
BLENDER.pair1.camRoll  = 0.0;
BLENDER.pair1.camYaw   = 0.0;

BLENDER.pair2.camL = "Camera3";
BLENDER.pair2.camR = "Camera4";
BLENDER.pair2.camBaseX = 0.0;
BLENDER.pair2.camBaseY = -3.0;
BLENDER.pair2.camBaseZ = 12.0;
BLENDER.pair2.B_m      = 3.0;
BLENDER.pair2.camPitch = 38.0;
BLENDER.pair2.camRoll  = 0.0;
BLENDER.pair2.camYaw   = 0.0;

BLENDER.width  = 1280;
BLENDER.height = 720;
BLENDER.ballPitch = 0;
BLENDER.ballRoll  = 0;
BLENDER.ballYaw   = 0;

%% -------------------- Defaults --------------------
defaults.Ztrue_m = 1.5;  % interpreted as Blender world Z for ball placement
defaults.f_px = 430;     % more reasonable starting point for this scene; tune later
defaults.netVFrac = 0.42;
defaults.radiusDiffPx = 2.0;
defaults.overlayRadiusPx = 16;
if ispc
    defaults.outputRoot = 'C:\Users\coope\Downloads\Project_ESD\tracker_runs';
else
    defaults.outputRoot = fullfile(tempdir, 'tracker_runs');
end

%% -------------------- State --------------------
client = [];
busy = false;
pendingUpdate = false;
trajXYZ = zeros(0,3);

if ~exist(defaults.outputRoot, 'dir')
    mkdir(defaults.outputRoot);
end

%% -------------------- GUI --------------------
ui = uifigure('Name','Ball Tracking GUI (MATLAB UI + Blender only)', ...
    'Position',[80 80 1280 760]);
ui.CloseRequestFcn = @(~,~) onClose();

main = uigridlayout(ui,[3 3]);
main.ColumnWidth = {390,'1x','1x'};
main.RowHeight   = {300, 260, '1x'};
main.Padding = [10 10 10 10];
main.RowSpacing = 10;
main.ColumnSpacing = 10;

pCtrl = uipanel(main,'Title','Controls');
pCtrl.Layout.Row = 1;
pCtrl.Layout.Column = 1;

g = uigridlayout(pCtrl,[8 3]);
g.RowHeight = {28,28,28,28,28,32,32,24};
g.ColumnWidth = {120,'1x',80};
g.Padding = [8 8 8 8];
g.RowSpacing = 4;
g.ColumnSpacing = 6;

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

statusLabel = uilabel(g,'Text','Idle.');
statusLabel.Layout.Row = 8; statusLabel.Layout.Column = [1 3];

pL = uipanel(main,'Title','Chosen Left Image');
pL.Layout.Row = 1; pL.Layout.Column = 2;
axL = uiaxes(pL); axL.Position = [10 10 400 180]; axis(axL,'image'); axis(axL,'off');

pR = uipanel(main,'Title','Chosen Right Image');
pR.Layout.Row = 1; pR.Layout.Column = 3;
axR = uiaxes(pR); axR.Position = [10 10 400 180]; axis(axR,'image'); axis(axR,'off');

pOut = uipanel(main,'Title','Output');
pOut.Layout.Row = [2 3]; pOut.Layout.Column = 1;

go = uigridlayout(pOut,[12 1]);
go.RowHeight = {30,24,24,24,24,24,24,24,24,24,24,'1x'};
go.Padding = [10 10 10 10];

lblPair   = uilabel(go,'Text','Chosen pair: --','FontWeight','bold');
lblCourt  = uilabel(go,'Text','Court side: --');
lblL      = uilabel(go,'Text','Left centroid: --');
lblR      = uilabel(go,'Text','Right centroid: --');
lblDisp   = uilabel(go,'Text','Disparity: --');
lblX      = uilabel(go,'Text','X: --');
lblY      = uilabel(go,'Text','Y: --');
lblZ      = uilabel(go,'Text','Z: --');
lblMsg1   = uilabel(go,'Text','Backend: --');
lblMsg2   = uilabel(go,'Text','Blender: --');
msg       = uitextarea(go,'Editable','off');
msg.Layout.Row = 12;

p3 = uipanel(main,'Title','3D Trajectory');
p3.Layout.Row = [2 3]; p3.Layout.Column = [2 3];
ax3 = uiaxes(p3); ax3.Position = [10 10 840 460];
grid(ax3,'on'); view(ax3,3);
xlabel(ax3,'X (m)'); ylabel(ax3,'Y (m)'); zlabel(ax3,'Z (m)');
title(ax3,'Tracked 3D Position (backend coordinates)');
hold(ax3,'on');
hTraj = plot3(ax3, NaN, NaN, NaN, 'b-','LineWidth',1.5);
hPt   = plot3(ax3, NaN, NaN, NaN, 'ro','MarkerSize',10,'LineWidth',2);
hold(ax3,'off');

sZ.ValueChangingFcn = @(~,evt) set(eZ,'Value',evt.Value);
sZ.ValueChangedFcn  = @(src,~) set(eZ,'Value',src.Value);
eZ.ValueChangedFcn  = @(src,~) set(sZ,'Value',src.Value);

%% -------------------- Main update --------------------
    function updateOnce()
        if busy
            pendingUpdate = true;
            return;
        end
        busy = true;
        btnRun.Enable = 'off';
        c = onCleanup(@() releaseBusy()); %#ok<NASGU>

        ballZ = eZ.Value;
        f_px = eF.Value;
        netFrac = eNet.Value;
        radiusDiff = eRad.Value;
        backendExe = strtrim(exeField.Value);

        if ~isfile(backendExe)
            statusLabel.Text = 'Backend executable not found.';
            msg.Value = ["Backend executable not found:"; string(backendExe)];
            return;
        end

        statusLabel.Text = 'Rendering from Blender...';
        drawnow;

        [I1L, I1R, I2L, I2R, blenderInfo] = getFourViewsFromBlender(ballZ);
        if isempty(I1L) || isempty(I1R) || isempty(I2L) || isempty(I2R)
            statusLabel.Text = 'Blender render failed.';
            msg.Value = blenderInfo(:);
            cla(axL); cla(axR);
            clearReadouts();
            return;
        end

        statusLabel.Text = 'Writing images + calling backend...';
        drawnow;

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
        if ~ok
            statusLabel.Text = 'Backend failed.';
            msg.Value = [blenderInfo(:); " "; backendInfo(:)];
            clearReadouts();
            return;
        end

        if result.pair_index == 1
            IL = I1L; IR = I1R;
        else
            IL = I2L; IR = I2R;
        end

        imshow(IL,'Parent',axL); title(axL, sprintf('Left | Pair %d', result.pair_index));
        hold(axL,'on');
        plot(axL, result.left_u, result.left_v, 'g+','MarkerSize',10,'LineWidth',2);
        try
            viscircles(axL,[result.left_u result.left_v],defaults.overlayRadiusPx,'Color','g','LineWidth',1);
        catch
        end
        hold(axL,'off');

        imshow(IR,'Parent',axR); title(axR, sprintf('Right | Pair %d', result.pair_index));
        hold(axR,'on');
        plot(axR, result.right_u, result.right_v, 'g+','MarkerSize',10,'LineWidth',2);
        try
            viscircles(axR,[result.right_u result.right_v],defaults.overlayRadiusPx,'Color','g','LineWidth',1);
        catch
        end
        hold(axR,'off');

        lblPair.Text  = sprintf('Chosen pair: %d', result.pair_index);
        lblCourt.Text = sprintf('Court side: %s', result.court_side);
        lblL.Text     = sprintf('Left centroid: (%.2f, %.2f)', result.left_u, result.left_v);
        lblR.Text     = sprintf('Right centroid: (%.2f, %.2f)', result.right_u, result.right_v);
        lblDisp.Text  = sprintf('Disparity: du=%.3f px  dv=%.3f px', result.disparity_u, result.disparity_v);
        lblX.Text     = sprintf('X: %.4f m', result.X);
        lblY.Text     = sprintf('Y: %.4f m', result.Y);
        lblZ.Text     = sprintf('Z: %.4f m', result.Z);
        lblMsg1.Text  = sprintf('Backend: %s', result.message);
        lblMsg2.Text  = sprintf('Blender: %s', strjoin(blenderInfo, ' | '));

        trajXYZ(end+1,:) = [result.X result.Y result.Z]; %#ok<AGROW>
        hTraj.XData = trajXYZ(:,1);
        hTraj.YData = trajXYZ(:,2);
        hTraj.ZData = trajXYZ(:,3);
        hPt.XData = result.X;
        hPt.YData = result.Y;
        hPt.ZData = result.Z;

        autoScale3D();

        statusLabel.Text = 'Done.';
        msg.Value = [blenderInfo(:); " "; backendInfo(:)];
    end

    function autoScale3D()
        if isempty(trajXYZ)
            xlim(ax3,[-1 1]); ylim(ax3,[-15 1]); zlim(ax3,[0 15]);
            return;
        end
        x = trajXYZ(:,1); y = trajXYZ(:,2); z = trajXYZ(:,3);
        px = max(0.25, 0.15*max(1,rangeSafe(x)));
        py = max(0.25, 0.15*max(1,rangeSafe(y)));
        pz = max(0.50, 0.15*max(1,rangeSafe(z)));
        xlim(ax3,[min(x)-px, max(x)+px]);
        ylim(ax3,[min(y)-py, max(y)+py]);
        zlim(ax3,[max(0,min(z)-pz), max(z)+pz]);
    end

    function r = rangeSafe(v)
        if isempty(v)
            r = 1;
        else
            r = max(v)-min(v);
            if r <= 0, r = 1; end
        end
    end

    function clearTrajectory()
        trajXYZ = zeros(0,3);
        hTraj.XData = NaN; hTraj.YData = NaN; hTraj.ZData = NaN;
        hPt.XData = NaN; hPt.YData = NaN; hPt.ZData = NaN;
        autoScale3D();
        statusLabel.Text = 'Trajectory cleared.';
    end

    function clearReadouts()
        lblPair.Text  = 'Chosen pair: --';
        lblCourt.Text = 'Court side: --';
        lblL.Text     = 'Left centroid: --';
        lblR.Text     = 'Right centroid: --';
        lblDisp.Text  = 'Disparity: --';
        lblX.Text     = 'X: --';
        lblY.Text     = 'Y: --';
        lblZ.Text     = 'Z: --';
        lblMsg1.Text  = 'Backend: --';
        lblMsg2.Text  = 'Blender: --';
    end

    function [ok, infoLines, result] = callTrackerBackend(backendExe, p1L, p1R, p2L, p2R, cfg, res)
        ok = false;
        infoLines = string.empty(0,1);
        result = struct('pair_index',0,'court_side','--','left_u',NaN,'left_v',NaN, ...
            'right_u',NaN,'right_v',NaN,'disparity_u',NaN,'disparity_v',NaN,'X',NaN,'Y',NaN,'Z',NaN,'message','--');

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
            % still try to parse result file if it exists; backend uses exit 1 for algorithm failure
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
        fprintf(fid,'f_px=%.12g\n', f_px);
        fprintf(fid,'pair1_baseline_m=%.12g\n', BL.pair1.B_m);
        fprintf(fid,'pair2_baseline_m=%.12g\n', BL.pair2.B_m);
        fprintf(fid,'net_v_frac=%.12g\n', netFrac);
        fprintf(fid,'radius_diff_px=%.12g\n', radiusDiff);
        fprintf(fid,'image_width=%d\n', BL.width);
        fprintf(fid,'image_height=%d\n', BL.height);
        fprintf(fid,'cx=%.12g\n', BL.width/2);
        fprintf(fid,'cy=%.12g\n', BL.height/2);
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

    function [I1L, I1R, I2L, I2R, infoLines] = getFourViewsFromBlender(ballZ_m)
        I1L = []; I1R = []; I2L = []; I2R = [];
        infoLines = string.empty(0,1);

        if ~ensureClient()
            infoLines = "No TCP client (ensureClient failed).";
            return;
        end

        flushClient();

        try
            blenderLink(client, BLENDER.width, BLENDER.height, ...
                0, -7.5, ballZ_m, ...
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
            sprintf('Ball world target = (0, -7.5, %.3f) m', ballZ_m)
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
            msg.Value = ["Failed to create tcpclient."; string(ME.message); "Make sure Blender server is running."];
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
            statusLabel.Text = 'MATLAB-side error.';
            msg.Value = ["MATLAB-side error:"; string(getReport(ME,'extended','hyperlinks','off'))];
        end
    end

    function releaseBusy()
        busy = false;
        try
            btnRun.Enable = 'on';
        catch
        end
        if pendingUpdate
            pendingUpdate = false;
            safeUpdateOnce();
        end
    end

    function onClose()
        try
            if ~isempty(client) && isvalid(client)
                clear client
            end
        catch
        end
        delete(ui);
    end
end
