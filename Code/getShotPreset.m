function preset = getShotPreset(presetName, groundZ)
    if nargin < 2
        groundZ = 0.35951;
    end

    switch lower(string(presetName))
        case "serve_default"
            preset.name = "serve_default"; preset.type = "serve";
            preset.x0 = -0.6; preset.y0 = -11.8; preset.z0 = groundZ + 1.84;
            preset.vx = 0.55; preset.vy = 9.40; preset.vz = 5.20;
            preset.g = 9.81; preset.bounceT = 0.72; preset.eApprox = 0.72;

        case "serve_t"
            preset.name = "serve_t"; preset.type = "serve";
            preset.x0 = -0.2; preset.y0 = -11.8; preset.z0 = groundZ + 1.84;
            preset.vx = 0.10; preset.vy = 9.80; preset.vz = 5.30;
            preset.g = 9.81; preset.bounceT = 0.72; preset.eApprox = 0.72;

        case "serve_wide"
            preset.name = "serve_wide"; preset.type = "serve";
            preset.x0 = -1.1; preset.y0 = -11.8; preset.z0 = groundZ + 1.84;
            preset.vx = 1.10; preset.vy = 9.00; preset.vz = 5.00;
            preset.g = 9.81; preset.bounceT = 0.70; preset.eApprox = 0.72;

        case "serve_body"
            preset.name = "serve_body"; preset.type = "serve";
            preset.x0 = -0.7; preset.y0 = -11.8; preset.z0 = groundZ + 1.84;
            preset.vx = 0.35; preset.vy = 9.20; preset.vz = 5.10;
            preset.g = 9.81; preset.bounceT = 0.71; preset.eApprox = 0.72;

        case "volley_default"
            preset.name = "volley_default"; preset.type = "volley";
            preset.x0 = 0.35; preset.y0 = -4.2; preset.z0 = groundZ + 1.64;
            preset.vx = -0.40; preset.vy = 7.60; preset.vz = 2.80;
            preset.g = 9.81; preset.bounceT = 0.58; preset.eApprox = 0.67;

        case "volley_short"
            preset.name = "volley_short"; preset.type = "volley";
            preset.x0 = 0.50; preset.y0 = -3.9; preset.z0 = groundZ + 1.50;
            preset.vx = -0.20; preset.vy = 6.80; preset.vz = 2.30;
            preset.g = 9.81; preset.bounceT = 0.50; preset.eApprox = 0.67;

        case "volley_deep"
            preset.name = "volley_deep"; preset.type = "volley";
            preset.x0 = -0.10; preset.y0 = -4.4; preset.z0 = groundZ + 1.72;
            preset.vx = 0.15; preset.vy = 8.40; preset.vz = 3.00;
            preset.g = 9.81; preset.bounceT = 0.64; preset.eApprox = 0.67;

        case "volley_cross"
            preset.name = "volley_cross"; preset.type = "volley";
            preset.x0 = -0.55; preset.y0 = -4.1; preset.z0 = groundZ + 1.66;
            preset.vx = 0.75; preset.vy = 7.50; preset.vz = 2.70;
            preset.g = 9.81; preset.bounceT = 0.57; preset.eApprox = 0.67;

        otherwise
            error('Unknown shot preset: %s', string(presetName));
    end
end