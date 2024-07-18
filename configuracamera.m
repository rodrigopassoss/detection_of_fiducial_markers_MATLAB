camList = webcamlist;
cam = webcam(2);

%% minha sala na UFS
% cam.Contrast = 10; %entre 0 e 10
% cam.Brightness = 30; %entre 30 e 255
% cam.Saturation = 200; %entre 0 e 200
% cam.resolution = '1280x720';
% cam.FocusMode = 'manual';
% cam.Exposure = -8;
% se = strel('disk',5);

% %% minha casa de noite
% cam.resolution = '1280x720';
% % cam.FocusMode = 'manual';
% % cam.Focus = 0;
% cam.ExposureMode = 'manual';
% cam.Exposure = -8;
% cam.Contrast = 10; %entre 0 e 10
% cam.Saturation = 100; %entre 0 e 100
% cam.Brightness = 30; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 100; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 30; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 100; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 30; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 100; %entre 30 e 255
% pause(0.1)
% cam.Brightness = 30; %entre 30 e 255
% see = strel('disk',5);

%% teto da robótica (lifecam vx-800)
% cam.Contrast = 10; %entre 0 e 10
% cam.Brightness = 10; %entre -10 a 10
% cam.Saturation = 10; %entre 0 e 10
% cam.resolution = '640x480';
% % cam.FocusMode = 'manual';
% cam.Exposure = -6;
% se = strel('disk',5);