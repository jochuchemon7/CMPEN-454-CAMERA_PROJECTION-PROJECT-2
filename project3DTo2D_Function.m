
function projected2DPoints = project3DTo2D(cam, worldCoord3DPoints)

%Solve for perspectiveProjection matrix
perspectiveProjection = cam.Kmat * cam.Pmat * worldCoord3DPoints;

%Prepare 2D points vector
projected2DPointsX = [0];
projected2DPointsY = [0];

for i = 1:12
    projected2DPointsX(i) = perspectiveProjection(1,i) ./ perspectiveProjection(3,i); 
    projected2DPointsY(i) = perspectiveProjection(2,i) ./ perspectiveProjection(3,i); 
end

%Combine points into a matrix
projected2DPoints = [projected2DPointsX; projected2DPointsY; ones(1,12)];
end