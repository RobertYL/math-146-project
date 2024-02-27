classdef Case
%CASE Problem case object
%   Object that stores all parameters for a problem case
%
%   Properties
%       N       - number of robots
%       x (N,2) - robot initial positions
%       y (N,2) - robot target positions
%       M       - number of circular obstacles
%       R       - radius of obstacles
%       c (M,2) - obstacle center positions
%       opts    - optional arguments
    
properties
    N {mustBeInteger,mustBePositive}
    x (:,2) {mustBeNumeric}
    y (:,2) {mustBeNumeric}
    M {mustBeInteger,mustBePositive}
    R {mustBeNumeric,mustBePositive}
    c (:,2) {mustBeNumeric}
    opts
end

methods
function obj = Case(N,M,R,opts)
    %CASE Default constructor

    arguments
        N (1,1) {mustBeInteger,mustBePositive} = 5
        M (1,1) {mustBeInteger,mustBePositive} = 5
        R (1,1) {mustBeNumeric,mustBePositive} = 20
        opts.MaxPos {mustBeNumeric} = 100
        opts.ObsOverlap logical = true
    end
    
    obj.N = N;
    obj.M = M;
    obj.R = R;
    obj.opts = opts;

    obj.x = (rand(N,2)-0.5)*2*obj.opts.MaxPos;
    obj.y = (rand(N,2)-0.5)*2*obj.opts.MaxPos;

    if obj.opts.ObsOverlap
        obj.c = (rand(M,2)-0.5)*2*(obj.opts.MaxPos-obj.R);
    else
        obj.c = zeros(M,2);
        for i = 1:M
            while 1
                obj.c(i,:) = (rand(1,2)-0.5)*2*obj.opts.MaxPos;
                if i == 1 || min(vecnorm(obj.c(1:i-1,:)-obj.c(i,:),2,2)) ...
                        > obj.R*2
                    break
                end
            end
        end
    end

    obj.x = zeros(N,2);
    obj.y = zeros(N,2);
    for i = 1:N
        while 1
            obj.x(i,:) = (rand(1,2)-0.5)*2*obj.opts.MaxPos;
            if i == 1 || min(vecnorm(obj.c-obj.x(i,:),2,2)) ...
                    > obj.R*2
                break
            end
        end
        while 1
            obj.y(i,:) = (rand(1,2)-0.5)*2*obj.opts.MaxPos;
            if i == 1 || min(vecnorm(obj.c-obj.y(i,:),2,2)) ...
                    > obj.R*2
                break
            end
        end
    end
end
end
end

