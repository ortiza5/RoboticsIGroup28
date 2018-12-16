function direction = ClosestWorkspace(current)
% finds viable joint angles (in range -pi to pi) given p0T
% INPUTS:
%   current - current pixel location
% OUTPUTS:
%   direction - direction to closes workspace location

assert(length(current) == 2, 'Error: You need to give desired uv pixel locations')

end