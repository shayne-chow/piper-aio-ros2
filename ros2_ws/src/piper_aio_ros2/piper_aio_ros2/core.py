"""Hardware-independent motion validation; angles in rad, gripper in m."""
import math

# J2/J3 firmware stops are 0°, but an unpowered arm rests a few degrees past zero.
# Those two edges are widened so gravity sag can pass the start check. Far stops stay.
LOWER = [math.radians(x) for x in (-150, -10, -170, -100, -70, -180)]+[-0.002]
UPPER = [math.radians(x) for x in (150, 180, 10, 100, 70, 180)]+[0.08]
# Commands must stay inside the firmware stops. Feedback may rest a few degrees outside.
COMMAND_LOWER = [math.radians(x) for x in (-150, 0, -170, -100, -70, -180)]+[0.0]
COMMAND_UPPER = [math.radians(x) for x in (150, 180, 0, 100, 70, 180)]+[0.08]


def commandable(q, lower=COMMAND_LOWER, upper=COMMAND_UPPER):
    return [min(hi, max(lo, x)) for x, lo, hi in zip(q, lower, upper)]


def validate(q, lower=LOWER, upper=UPPER):
    if len(q) != 7 or not all(math.isfinite(x) for x in q):
        raise ValueError('Expected seven finite joint/gripper positions')
    if any(x < lo or x > hi for x, lo, hi in zip(q, lower, upper)):
        raise ValueError('Position outside configured joint/gripper limits')
    return list(q)


def alignment_errors(leader, follower, angle=0.15, gripper=0.015, lower=LOWER, upper=UPPER):
    leader = validate(leader, lower, upper)
    follower = validate(follower, lower, upper)
    errors = []
    for i, (a, b) in enumerate(zip(leader, follower)):
        limit = angle if i < 6 else gripper
        delta = abs(a - b)
        if delta > limit:
            errors.append((i, delta, limit))
    return errors


def format_alignment(side, errors):
    parts = []
    for i, delta, limit in errors:
        if i < 6:
            parts.append(f'J{i+1} {math.degrees(delta):.1f}°>{math.degrees(limit):.1f}°')
        else:
            parts.append(f'gripper {delta*1000:.1f}mm>{limit*1000:.1f}mm')
    return f'{side}: align arms before starting ({", ".join(parts)})'


def aligned(leader, follower, angle=0.15, gripper=0.015, lower=LOWER, upper=UPPER):
    return not alignment_errors(leader, follower, angle, gripper, lower, upper)


def limited_step(previous, target, dt, angular_speed=0.3, gripper_speed=0.02, lower=LOWER, upper=UPPER):
    validate(previous,lower,upper); validate(target,lower,upper)
    if not math.isfinite(dt) or dt <= 0 or dt > 0.1:
        raise ValueError('Control loop missed deadline')
    return [p + max(-v*dt, min(v*dt, t-p)) for p,t,v in
            zip(previous, target, [angular_speed]*6+[gripper_speed])]


def fresh(stamps, ids, now, timeout):
    return all(i in stamps and 0 <= now-stamps[i] <= timeout for i in ids)
