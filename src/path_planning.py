import math

HALF_WIDTH = 1.0         # m: half of the track width (track assumed ~2 m wide)
PATH_LENGTH = 8.0        # m: total length of the returned path (allowed: 5-10)
STEP = 0.25              # m: distance between path points (allowed: <= 0.5)
SMOOTHING_PASSES = 15    # how many times we average the path to round corners


def distance(a, b):
    """Distance between two points a and b."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(dx, dy):
    """Turn the vector (dx, dy) into length 1 (a pure direction)."""
    length = math.hypot(dx, dy)
    if length > 1e-9:
        return (dx / length, dy / length)
    return (1.0, 0.0)


def sort_by_number(points, number_of):
    """Sort points from the smallest to the biggest number.

    number_of is a function that gives one number for a point.
    """
    pairs = []
    for p in points:
        pairs.append((number_of(p), p))
    pairs.sort()                     
    result = []
    for number, p in pairs:
        result.append(p)
    return result


class PathPlanning:
    """Simple cone-based path planner.

    Idea:  1) find a few points that lie on the CENTRE of the track ("anchors")
           2) draw a line: car -> anchors -> straight ahead
           3) round the corners and space the points evenly
    """

    def __init__(self, car_pose, cones):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self):
        car = (self.car_pose.x, self.car_pose.y)
        heading = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))

        # Split the cones by colour.
        blue = []      # blue   = LEFT
        yellow = []    # yellow = RIGHT
        for c in self.cones:
            if c.color == 1:
                blue.append((c.x, c.y))
            elif c.color == 0:
                yellow.append((c.x, c.y))

        def distance_to_car(p):
            return distance(p, car)

        blue = sort_by_number(blue, distance_to_car)
        yellow = sort_by_number(yellow, distance_to_car)

        if len(blue) > 0 and len(yellow) > 0:
            anchors, end_dir = self._both_sides(blue, yellow)
        elif len(blue) > 0:
            anchors, end_dir = self._one_side(blue, True, heading)
        elif len(yellow) > 0:
            anchors, end_dir = self._one_side(yellow, False, heading)
        else:
            anchors = []                # no cones: go straight
            end_dir = heading

        return self._make_path(car, anchors, end_dir)

    def _both_sides(self, blue, yellow):
        """Blue AND yellow cones: connect them in a zig-zag, take the midpoints.

        Edges:  blue1-yellow1, blue2-yellow1, blue2-yellow2, blue3-yellow2, ...
        (each step we move forward on the side that gives the SHORTER edge)
        """
        b0 = blue[0]
        y0 = yellow[0]
        best = distance(b0, y0)
        for b in blue:
            for y in yellow:
                if distance(b, y) < best:
                    best = distance(b, y)
                    b0 = b
                    y0 = y
        track_dir = unit(-(y0[1] - b0[1]), y0[0] - b0[0])

   
        def along(p):
            return p[0] * track_dir[0] + p[1] * track_dir[1]

        blue = sort_by_number(blue, along)
        yellow = sort_by_number(yellow, along)

        i = 0                            
        j = 0                              
        edges = [(blue[0], yellow[0])]    
        while i < len(blue) - 1 or j < len(yellow) - 1:
            if i == len(blue) - 1:
                j += 1                    
            elif j == len(yellow) - 1:
                i += 1                     
            elif distance(blue[i + 1], yellow[j]) <= distance(blue[i], yellow[j + 1]):
                i += 1                    
            else:
                j += 1                    
            edges.append((blue[i], yellow[j]))

        anchors = []
        for b, y in edges:
            anchors.append(((b[0] + y[0]) / 2, (b[1] + y[1]) / 2))

        if len(anchors) >= 2:
            last = anchors[-1]
            before_last = anchors[-2]
            end_dir = unit(last[0] - before_last[0], last[1] - before_last[1])
        else:
            b, y = edges[0]
            end_dir = unit(-(y[1] - b[1]), y[0] - b[0])
        return anchors, end_dir

    def _one_side(self, cones, is_blue, heading):
        """1, 2 or 3 cones of ONE colour: move each cone sideways by half the track width.

        Blue cones are on the left  -> the centre is to the RIGHT of them.
        Yellow cones are on the right -> the centre is to the LEFT of them.
        'Sideways' = perpendicular to the direction the cones run in, which we get
        from the neighbouring cones. With 3 cones this follows a bend.
        """
        if is_blue:
            shift = -HALF_WIDTH
        else:
            shift = HALF_WIDTH

        anchors = []
        direction = heading
        for k in range(len(cones)):
            c = cones[k]
            if len(cones) == 1:
                direction = heading                
            else:
                k_before = max(k - 1, 0)
                k_after = min(k + 1, len(cones) - 1)
                before = cones[k_before]
                after = cones[k_after]
                direction = unit(after[0] - before[0], after[1] - before[1])
            left_normal = (-direction[1], direction[0])   
            anchors.append((c[0] + left_normal[0] * shift,
                            c[1] + left_normal[1] * shift))
        return anchors, direction

    def _make_path(self, car, anchors, end_dir):
        if len(anchors) > 0:
            last = anchors[-1]
        else:
            last = car
        far_point = (last[0] + end_dir[0] * PATH_LENGTH,
                     last[1] + end_dir[1] * PATH_LENGTH)
        rough = [car] + anchors + [far_point]

        points = self._resample(rough)

        for _ in range(SMOOTHING_PASSES):
            new_points = [points[0]]
            for n in range(1, len(points) - 1):
                x = (points[n - 1][0] + points[n][0] + points[n + 1][0]) / 3
                y = (points[n - 1][1] + points[n][1] + points[n + 1][1]) / 3
                new_points.append((x, y))
            new_points.append(points[-1])
            points = new_points
        return points

    @staticmethod
    def _resample(line):
        """Walk along the line and drop a point every STEP metres."""
        out = [line[0]]
        need = STEP                  
        max_points = int(PATH_LENGTH / STEP) + 1

        for n in range(len(line) - 1):
            a = line[n]
            b = line[n + 1]
            seg = distance(a, b)       
            walked = 0.0
            while seg - walked >= need:
                walked += need
                t = walked / seg
                out.append((a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t))
                need = STEP
                if len(out) >= max_points:
                    return out
            need -= seg - walked      
        return out
