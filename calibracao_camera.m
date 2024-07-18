      
      clc
      clear all
      close all
      cnt = 1;
      for i = [1:9]
%           I = imread(['Calibration_target/imagem',num2str(i),'.jpg']);
          I = imread(['Calibration_target/imagem',num2str(i),'.jpg']);
          %       Igray = rgb2gray(I);


          % Detect the checkerboard corners in the images.
          [imagePoints_, boardSize] = detectCheckerboardPoints(I);
          imagePoints(:,:,cnt) = imagePoints_;cnt = cnt + 1;
          imshow(I)
          hold on
          plot(imagePoints_(:,1),imagePoints_(:,2),'.r','MarkerSize',30)
          plot(imagePoints_(:,1),imagePoints_(:,2),'-y','linewidth',2)
          drawnow
      end
load('camera_params2.mat');


   
      % Generate the world coordinates of the checkerboard corners in the
      % pattern-centric coordinate system, with the upper-left corner at (0,0).
      squareSize = 29; % in millimeters
      worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%    
%       % Calibrate the camera.
%       imageSize = [size(I, 1), size(I, 2)];
%       cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
%                                       'ImageSize', imageSize);
   
      % Load image at new location.
      imOrig = imread(['Calibration_target/imagem',num2str(1),'.jpg']);

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
      tic
      [rotationMatrix, translationVector] = extrinsics(...
        imagePoints, worldPoints, cameraParams);
      toc
      
      
     worldPoints(:,3)=0;
     u = rotationMatrix'*worldPoints' - rotationMatrix'*translationVector';
%      u1 = u./u(3,:);
     u2 = (cameraParams.IntrinsicMatrix'*u);
     u2 = u2./u2(3,:);
     hold on
     plot(u2(1,:),u2(2,:),'.r','MarkerSize',25)
     plot(imagePoints(:,1),imagePoints(:,2),'.y','MarkerSize',25)
 
      % Compute camera pose.
      [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
          translationVector);
      figure
      plotCamera('Location', location, 'Orientation', orientation, 'Size', 20);
      hold on
      pcshow([worldPoints, zeros(size(worldPoints,1), 1)], ...
          'VerticalAxisDir', 'down', 'MarkerSize', 40);