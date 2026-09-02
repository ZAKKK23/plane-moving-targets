%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RAYPOLYLINEINTERSECT
% Drop-in replacement for the single intersection point that
% voronoiProjectToBoundary.m needs from polyxpoly, without requiring
% the Mapping Toolbox.
%
% Finds the intersection of the ray from (rayx(1),rayy(1)) through
% (rayx(2),rayy(2)) with the polyline defined by (px,py), restricted to
% the segment t in [0,1] along the ray (i.e. only intersections between
% the ray's two given endpoints, same convention polyxpoly uses here
% since the ray is drawn out to a point L units away, far outside the
% bounding box).
%
% Returns the single closest intersection point (xi,yi) to the ray's
% start point. If no intersection is found, returns (NaN,NaN).

function [xi,yi] = rayPolylineIntersect(rayx, rayy, px, py)

xi = NaN; yi = NaN;
best_t = Inf;

x1 = rayx(1); y1 = rayy(1);
x2 = rayx(2); y2 = rayy(2);
rdx = x2 - x1;
rdy = y2 - y1;

n = numel(px);
for k = 1:(n-1)
    x3 = px(k);   y3 = py(k);
    x4 = px(k+1); y4 = py(k+1);

    sdx = x4 - x3;
    sdy = y4 - y3;

    denom = rdx*sdy - rdy*sdx;
    if abs(denom) < 1e-12
        continue;  % parallel (or degenerate) segment, skip
    end

    t = ((x3-x1)*sdy - (y3-y1)*sdx) / denom;
    u = ((x3-x1)*rdy - (y3-y1)*rdx) / denom;

    if t >= -1e-9 && t <= 1+1e-9 && u >= -1e-9 && u <= 1+1e-9
        if t < best_t
            best_t = t;
            xi = x1 + t*rdx;
            yi = y1 + t*rdy;
        end
    end
end

end
