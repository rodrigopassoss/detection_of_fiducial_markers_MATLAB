 % Create a set of calibration images.
      images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
          'calibration', 'slr'));
   
      % Detect the checkerboard corners in the images.
      [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);
   
      % Generate the world coordinates of the checkerboard corners in the
      % pattern-centric coordinate system, with the upper-left corner at (0,0).
      squareSize = 29; % in millimeters
      worldPoints = generateCheckerboardPoints(boardSize, squareSize);
   
      % Calibrate the camera.
      I = readimage(images,1); 
      imageSize = [size(I, 1), size(I, 2)];
      cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                      'ImageSize', imageSize);
   
      % Load image at new location.
      imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
            'calibration', 'slr', 'image9.jpg'));
      figure 
      imshow(imOrig);
      title('Input Image');
   
      % Undistort image.
      [im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
   
      % Find reference object in new image.
      [imagePoints, boardSize] = detectCheckerboardPoints(im);
   
      % Compensate for image coordinate system shift.
      imagePoints = [imagePoints(:,1) + newOrigin(1), ...
                     imagePoints(:,2) + newOrigin(2)];
   
      % Compute new extrinsics.
      [rotationMatrix, translationVector] = extrinsics(...
        imagePoints, worldPoints, cameraParams);
 
      % Compute camera pose.
      [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
          translationVector);
      figure
      plotCamera('Location', location, 'Orientation', orientation, 'Size', 20);
      hold on
      pcshow([worldPoints, zeros(size(worldPoints,1), 1)], ...
          'VerticalAxisDir', 'down', 'MarkerSize', 40);