"""Geometry-only dust integration and position-evaluation contracts.

Concentration values describe respirable dust in mg/m3. This module does not
infer physical dust effects and does not change the existing Mock interface.
"""
import math


def segment_box_fraction(origin, endpoint, lower, upper):
    """Fraction of a finite ray segment inside an axis-aligned box."""
    enter, leave = 0.0, 1.0
    for a, b, lo, hi in zip(origin, endpoint, lower, upper):
        delta = b - a
        if abs(delta) < 1e-12:
            if a < lo or a >= hi:
                return 0.0
        else:
            t0, t1 = sorted(((lo - a) / delta, (hi - a) / delta))
            enter, leave = max(enter, t0), min(leave, t1)
            if leave <= enter:
                return 0.0
    return max(0.0, leave - enter)


def validate_zones(zones):
    bounds = []
    names = set()
    for z in zones:
        if z['zone_id'] in names:
            raise ValueError('duplicate zone_id')
        names.add(z['zone_id'])
        points = z['polygon_xy_m']
        xs, ys = sorted(set(p[0] for p in points)), sorted(set(p[1] for p in points))
        if len(xs) != 2 or len(ys) != 2 or set(map(tuple, points)) != {(x,y) for x in xs for y in ys}:
            raise ValueError('runtime supports axis-aligned rectangular zones only')
        lo, hi = [xs[0],ys[0],z['z_min_m']], [xs[1],ys[1],z['z_max_m']]
        c = z['respirable_concentration_mg_m3']
        if not math.isfinite(c) or c < 0 or any(b <= a for a,b in zip(lo,hi)):
            raise ValueError('invalid dust zone')
        for oldlo, oldhi in bounds:
            if all(min(b,d)>max(a,c) for a,b,c,d in zip(lo,hi,oldlo,oldhi)):
                raise ValueError('overlapping zones would double-count concentration')
        bounds.append((lo,hi))
    return bounds


def path_concentration(origin, endpoint, zones, background=0.0, *, prepared_bounds=None):
    if len(origin)!=3 or len(endpoint)!=3 or not all(math.isfinite(x) for x in origin+endpoint):
        raise ValueError('finite xyz endpoints required')
    distance = math.dist(origin, endpoint)
    if distance <= 0 or not math.isfinite(background) or background < 0:
        raise ValueError('positive path length and nonnegative concentration required')
    bounds = validate_zones(zones) if prepared_bounds is None else prepared_bounds
    pieces = []
    fractions = []
    for zone,(lo,hi) in zip(zones,bounds):
        fraction = segment_box_fraction(origin,endpoint,lo,hi)
        if fraction > 0:
            pieces.append({'zone_id':zone['zone_id'],'length_m':distance*fraction,
                           'concentration_mg_m3':zone['respirable_concentration_mg_m3']})
            fractions.append(fraction)
    outside = max(0.0, 1.0-sum(fractions))
    avg = sum(p['length_m']*p['concentration_mg_m3'] for p in pieces)/distance + outside*background
    return {'path_average_concentration_mg_m3':avg,'zone_id':'|'.join(p['zone_id'] for p in pieces) or 'NONE',
            'segments':pieces,'background_length_m':outside*distance}


def route_pose(elapsed_s, route):
    segments=route['segments']
    segment=next((s for s in segments if s['start_s']<=elapsed_s<s['end_s']),segments[-1] if elapsed_s>=segments[-1]['end_s'] else segments[0])
    f=max(0.,min(1.,(elapsed_s-segment['start_s'])/(segment['end_s']-segment['start_s'])))
    return segment['start_y_m']+f*(segment['end_y_m']-segment['start_y_m']), segment['work_mode']


def position_error(estimate, truth, max_time_difference_s=0.025):
    """Input contract only; no localization algorithm. Both poses in map."""
    if estimate['frame_id']!='map' or truth['frame_id']!='map':raise ValueError('map frame required')
    if not all(math.isfinite(t) for t in (estimate['stamp_s'], truth['stamp_s'], max_time_difference_s)) or max_time_difference_s < 0:
        raise ValueError('finite timestamps and nonnegative tolerance required')
    if abs(estimate['stamp_s']-truth['stamp_s'])>max_time_difference_s:raise ValueError('timestamps not aligned')
    a,b=estimate['xyz_m'],truth['xyz_m']
    if len(a)!=3 or len(b)!=3 or not all(math.isfinite(x) for x in a+b):raise ValueError('finite xyz required')
    delta=[x-y for x,y in zip(a,b)]
    return {'dx_m':delta[0],'dy_m':delta[1],'dz_m':delta[2],'position_error_m':math.dist(a,b)}


if __name__=='__main__':
    z=[dict(zone_id='a',polygon_xy_m=[[-1,0],[1,0],[1,5],[-1,5]],z_min_m=0,z_max_m=2,respirable_concentration_mg_m3=2),
       dict(zone_id='b',polygon_xy_m=[[-1,5],[1,5],[1,10],[-1,10]],z_min_m=0,z_max_m=2,respirable_concentration_mg_m3=10)]
    assert abs(path_concentration([0,0,1],[0,10,1],z)['path_average_concentration_mg_m3']-6)<1e-12
    assert abs(path_concentration([0,10,1],[0,0,1],z)['path_average_concentration_mg_m3']-6)<1e-12
    assert abs(path_concentration([0,-5,1],[0,5,1],z)['path_average_concentration_mg_m3']-1)<1e-12
    assert path_concentration([2,0,1],[2,10,1],z)['path_average_concentration_mg_m3']==0
    assert path_concentration([0,5,1],[0,10,1],z)['path_average_concentration_mg_m3']==10
    try:validate_zones(z+[dict(z[0],zone_id='overlap')])
    except ValueError:pass
    else:raise AssertionError('overlap not rejected')
    assert position_error(dict(frame_id='map',stamp_s=1,xyz_m=[3,4,0]),dict(frame_id='map',stamp_s=1,xyz_m=[0,0,0]))['position_error_m']==5
    print('PASS: weighted paths, reverse direction, background, boundary, outside, overlap, position-error contract')
