def calc_goal_heading(current_heading, by):
    goal_heading = (current_heading + by) % 360
    # print(f"Current: {current_heading}\tby {by}\tGoal: {goal_heading}")
    return goal_heading


def rotate_to_go(current_heading, goal_heading):
    deg = goal_heading - current_heading
    if deg > 180:
        deg = deg - 360
    elif deg < -180:
        deg = deg + 360
    else:
        pass
    return deg


if __name__ == "__main__":
    import argparse

    headings = [0, 10, 170, 270, 350]
    for heading in headings:
        calc_goal_heading(heading, 30)

    for heading in headings:
        calc_goal_heading(heading, -30)
    print("\n\n")
    ap = argparse.ArgumentParser()
    ap.add_argument("--heading", type=int, default=0, help="current heading reading")
    ap.add_argument("--by", type=int, default=30, help="degree to turn")
    args = vars(ap.parse_args())
    # print(args)
    current_heading = int(args.get("heading", 0))
    by = int(args.get("by", 30))

    goal = calc_goal_heading(current_heading, by)

    ranges = []
    start = current_heading
    end = current_heading + by
    if by > 0:
        if end < 360:
            ranges.append((start, end))
        else:
            ranges.append((start, 360))
            ranges.append((0, end % 360))
    elif by < 0:
        if end >= 0:
            ranges.append((start, end))
        else:
            ranges.append((start, 0))
            ranges.append((360, end % 360))
    else:
        pass

    print(ranges)

    for r in ranges:
        (start, end) = r
        inc = 1 if end >= start else -1
        for c in range(start, end, inc):
            deg = rotate_to_go(c, goal)
            print(f"{c}\tto\t{goal}\t=\t{deg}")
