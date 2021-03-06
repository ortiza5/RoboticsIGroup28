function closest = ClosestWorkspace(current, binary_image)
% finds viable joint angles (in range -pi to pi) given p0T
% INPUTS:
%   current - current pixel location
%   binary_image - image where 0 is workspace and 1 is not
% OUTPUTS:
%   closest - closest workspace pixel location

image_width = size(binary_image,2);
image_height = size(binary_image,1);

assert(length(current) == 2, 'Error: You need to give desired uv pixel locations')

u = current(1);
v = current(2);

% checks in a circle to see if any pixel is set to 1, then if not,
% increases radius (distance), until some is found. Increases by 5 pixels
% for efficiency
for distance = 0:5:length(binary_image)
    for angle = 0:360
        test_u = u + round(distance*cosd(angle));
        test_v = v + round(distance*sind(angle));
        
        % make sure test [u,v] are in image boundary
        if test_u >= 1 && test_u <= image_width
            if test_v >= 1 && test_v <= image_height
                if binary_image(test_v, test_u) == 0
                    % found valid solution
                    closest = [test_u; test_v];
                    return;
                end
            end
        end
    end
end
end