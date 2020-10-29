format long

load('Subject4-Session3-Take4_mocapJoints.mat')
load('vue2CalibInfo.mat')
load('vue4CalibInfo.mat')

dimMocap = size(mocapJoints)
dimVue2 = size(vue2)
dimVue4 = size(vue4)

frame_1 = mocapJoints(1,:,:);


%Reading the 3D data
mocapFnum = 1000; %frame number 1000
x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
y = mocapJoints(mocapFnum,:,2); % Y Coordinates
z = mocapJoints(mocapFnum,:,3); % Z Coordinates
conf = mocapJoints(mocapFnum,:,4); %Confidence values

%Figure out what vue2 fields stand for on the camera calibration
%parameteres
vue2

%Intristic: orientation & position
%extristic(Rotation & translation): foclen, 


%For all frames
allX = mocapJoints(1:26214,:,1);
allY = mocapJoints(1:26214,:,2);
allZ = mocapJoints(1:26214,:,3);
allConf = mocapJoints(1:26214,:,4); 


filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';

vue2video = VideoReader(filenamevue2mp4);
vue4video = VideoReader(filenamevue4mp4);

%now we can read in the video for any mocap frame mocapFnum
%the (50/100) factor is here to account for the difference in frame
%rates between video (50 fps) and mocap (100 fps)

vue2video.CurrentTime = (mocapFnum -1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);

%Display the color image
image(vid2Frame);

RightShoulderX = double(x(1));
RightShoulderY = y(1);
RightShoulderZ = z(1);


%%%main-ish()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
worldCoord3DPoints = [RightShoulderX ; RightShoulderY; RightShoulderZ;1];

resultCam1 = project3DTo2D(vue2,worldCoord3DPoints);
resultCam2 = project3DTo2D(vue4,worldCoord3DPoints);

Res3Dto2DCam1 = [0;0;0];
Res3Dto2DCam2 = [0;0;0];

for i = 1:12
    worldCoord3DPoints = [x(i);y(i);z(i);1];
    temp = project3DTo2D(vue2, worldCoord3DPoints);
    Res3Dto2DCam1 = [Res3Dto2DCam1, temp];
    temp = project3DTo2D(vue4, worldCoord3DPoints);
    Res3Dto2DCam2 = [Res3Dto2DCam2, temp];
end
Res3Dto2DCam1(:,1) = []
Res3Dto2DCam2(:,1) = []

%results3D = reconstruct3DFrom2D(vue2, resultCam1, vue4, resultCam2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function projected2DPoints = project3DTo2D(cam, worldCoord3DPoints)

cam.

end

function recovered3DPoints = reconstruct3DFrom2D(cam1, cam1PixelCoords, cam2, cam2PixelCoords)

recovered3DPoints = cam1*cam1PixelCoords + cam2*cam2PixelCoords;
end


function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)

EpipolarLines1 = 1;
EpipolarLines2 =2 ;
end






