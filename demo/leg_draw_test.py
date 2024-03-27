import pygame as pg
import math


GEOM_ARC = 0
GEOM_LINE = 1  # Straight line
LINE_WIDTH = 1
TEST_OFFSET = 3
WND_WIDTH = 600
WND_HEIGHT = 600


C_MAGENTA = (200, 0, 167)

class Point:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        return str(self.x) + " " + str(self.y)
    
    def normalise(self):
        l = (self.x ** 2 + self.y ** 2) ** 0.5
        self.x /= l
        self.y /= l

class Line:
    def __init__(self, p1, p2) -> None:
        self.coords = [p1, p2]
        self.ka = 0
        self.kb = 0
        self.kc = 0

    def __str__(self):
        return str(self.ka) + " " + str(self.kb) + " " + str(self.kc)

    def calc_coeffs(self):
        self.ka = self.coords[1].y - self.coords[0].y
        self.kb = self.coords[0].x - self.coords[1].x
        self.kc = self.coords[0].y * self.coords[1].x - \
            self.coords[0].x * self.coords[1].y

    def get_intersection(self, other):
        check_term = self.kb * other.ka - self.ka * other.kb
        if check_term:
            x_intersect = (self.kc * other.kb - self.kb * other.kc) / check_term
            y_intersect = 0
            if self.kb:
                y_intersect = -(self.ka * x_intersect + self.kc) / self.kb
            else:
                y_intersect = -(other.ka * x_intersect + other.kc) / other.kb
            return Point(x_intersect, y_intersect)
        return Point(None, None)


def get_rect(c, r):
    return (c.x-r, WND_HEIGHT - c.y - r, 2 * r, 2 * r)

def get_dist(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5

def get_mid_point(p1, p2):
    v = Point(p2.x - p1.x, p2.y - p1.y)
    midpoint = Point(p1.x + 0.5 * v.x, p1.y + 0.5 * v.y)

    return midpoint

def get_point_angle_deg(c, p):
    v = Point(p.x - c.x, p.y - c.y)
    v.normalise()
    arc = math.atan2(v.y, v.x)

    return arc

def dist_to_line(ln, x):
    s_a = get_dist(ln.coords[0], x)
    s_b = get_dist(ln.coords[1], x)
    s_c = get_dist(ln.coords[0], ln.coords[1])

    p = (s_a + s_b + s_c) / 2
    area = math.sqrt(p * (p - s_a) * (p - s_b) * (p - s_c))
    dist = area * 2 / s_c
    return dist

def get_points(ln1, ln2, r, x):
    a = Point(ln1.coords[1].x - ln1.coords[0].x, ln1.coords[1].y - ln1.coords[0].y)
    d = Point(ln2.coords[1].x - ln2.coords[0].x, ln2.coords[1].y - ln2.coords[0].y)

    a.normalise()
    d.normalise()

    b = Point(a.y, -a.x)
    e = Point(-d.y, d.x)

    ln1.calc_coeffs()
    ln2.calc_coeffs()

    p_intc = ln1.get_intersection(ln2)

    p1 = Point(p_intc.x + e.x * r, p_intc.y + e.y * r)
    p3 = Point(p_intc.x + d.x * r, p_intc.y + d.y * r)
    p2 = Point(p3.x + e.x * r, p3.y + e.y * r)
    test_line = Line(p1, p2)

    c1 = Point(x.x + b.x * r, x.y + b.y * r)

    dist_c1_ln2 = dist_to_line(ln2, c1)
    #print(dist_c1_ln2)
    
    dist_tl = dist_to_line(test_line, c1)

    y_off = (4*r*r - dist_tl ** 2) ** 0.5

    dist_c1_intc = get_dist(c1, p_intc)

    x_off = y_off + (dist_c1_intc ** 2 - dist_c1_ln2 ** 2) ** 0.5
    
    p_intm_1 = Point(c1.x + e.x * dist_tl, c1.y + e.y * dist_tl)
    c2 = Point(p_intm_1.x + y_off * d.x, p_intm_1.y + y_off * d.y)
    #c2 = p_intm_1

    p_start2 = Point(p_intc.x + d.x * x_off, p_intc.y + d.y * x_off)

    #print(get_dist(c1, c2))

    return c1, c2, p_start2

def draw_smooth_con(pg_sfc, c1, c2, s2, ln1, ln2, x, r):
    mp = get_mid_point(c1, c2)

    ang_x = get_point_angle_deg(c1, x)
    ang_mp1 = get_point_angle_deg(c1, mp)
    ang_mp2 = get_point_angle_deg(c2, mp)
    ang_s2 = get_point_angle_deg(c2, s2)

    #print(ang_x, ang_mp1)
    pg.draw.arc(pg_sfc, C_MAGENTA, get_rect(c1, r), ang_mp1, ang_x, LINE_WIDTH)
    pg.draw.arc(pg_sfc, C_MAGENTA, get_rect(c2, r), ang_mp2, ang_s2, LINE_WIDTH)
    #print(ln1.coords[1].x, ln1.coords[1].y, s2.x, s2.y)
    pg.draw.line(pg_sfc, C_MAGENTA, (ln1.coords[0].x, WND_HEIGHT - ln1.coords[0].y), \
                 (x.x, WND_HEIGHT - x.y), LINE_WIDTH)
    
    pg.draw.line(pg_sfc, C_MAGENTA, (ln2.coords[1].x, WND_HEIGHT - ln2.coords[1].y), \
                 (s2.x, WND_HEIGHT - s2.y), LINE_WIDTH)
    
    #pg.draw.circle(pg_sfc, C_MAGENTA, (c1.x, WND_HEIGHT - c1.y), r, LINE_WIDTH)
    #pg.draw.circle(pg_sfc, C_MAGENTA, (c2.x, WND_HEIGHT - c2.y), r, LINE_WIDTH)

def draw_smooth_lines(pg_sfc, ln1, ln2, r, x):
    c1, c2, p_start2 = get_points(ln1, ln2, r, x)

    draw_smooth_con(pg_sfc, c1, c2, p_start2, ln1, ln2, x, r)


pg.init()
screen = pg.display.set_mode((WND_WIDTH, WND_HEIGHT))
scale = 100
p1l1 = Point(4 * scale, 1.9 * scale)
p2l1 = Point(1 * scale, 2 * scale)
p1l2 = Point(0.4 * scale, 5.5 * scale)
p2l2 = Point(0.2 * scale, 2 * scale)

l1 = Line(p1l1, p2l1)
l2 = Line(p2l2, p1l2)

while 1:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            exit()
    
    screen.fill((0,0,0))

    draw_smooth_lines(screen, l1, l2, 89, p2l1)

    pg.display.update()
