from tkinter import *
import math
import heapq
from sympy import Point, Polygon
# from shapely.geometry import Point, Polygon

'''================= Your classes and methods ================='''

ids = []

class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)

def rotate(points, angle, center):
    angle = math.radians(angle)
    cos_val = math.cos(angle)
    sin_val = math.sin(angle)
    cx, cy = center
    new_points = []

    for x_old, y_old in points:
        x_old -= cx
        y_old -= cy
        x_new = x_old * cos_val - y_old * sin_val
        y_new = x_old * sin_val + y_old * cos_val
        new_points.append((x_new + cx, y_new + cy))

    return new_points


def get_polygon_from_position(position):
    x, y, yaw = position
    points = [(x - 50, y - 100), (x + 50, y - 100), (x + 50, y + 100), (x - 50, y + 100)]
    # Заменил new_points по предложению из чата https://discord.com/channels/681511874449244211/884816389809004584/910841777181831168
    new_points = rotate(points, yaw * 180 / math.pi, (x, y))
    return Polygon(*list(map(Point, new_points)))

def get_polygon_from_obstacle(obstacle):
    points = [(obstacle[0], obstacle[1]), (obstacle[2], obstacle[3]), (obstacle[4], obstacle[5]),
              (obstacle[6], obstacle[7])]
    return Polygon(*list(map(Point, points)))

def collides(position, obstacle, block_diag):
    x1, y1, _ = position
    x2 = sum([x for i, x in enumerate(obstacle) if i % 2 == 0]) / 4
    y2 = sum([x for i, x in enumerate(obstacle) if i % 2 == 1]) / 4
    if ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 > block_diag:
        return False
    return get_polygon_from_position(position).intersection(get_polygon_from_obstacle(obstacle))

path_list = []

class Window():
    '''================= Your Main Function ================='''

    def go(self, event):

        print("Start position:", self.get_start_position())
        print("Target position:", self.get_target_position())
        print("Obstacles:", self.get_obstacles())

        global path_list
        for id in path_list:
            self.canvas.delete(id)
        path = self.star_search(self.naive_heuristic, [self.get_start_position()], 60)
        path = self.star_search(self.custom_heuristic, path, 40)
        global ids
        for point in path:
            id = self.canvas.create_oval(point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2, fill='yellow')
            ids.append(id)
        self.root.update()
        path_list = ids

    def draw_path(self, path):
        ids_path = []
        for point in path:
            id = self.canvas.create_oval(point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2, fill='yellow')
            ids_path.append(id)
        self.root.update()
        for id in ids_path:
            self.canvas.delete(id)

    def star_search(self, heuristic, path, step_size):
        p_queue = PriorityQueue()
        history = set()
        heur = heuristic(path[-1])
        p_queue.push((path[-1], path), heur)
        block_diag = (100 ** 2 + 200 ** 2) ** 0.5

        while not p_queue.isEmpty():
            node = p_queue.pop()
            if self.target_reached(node[0], heuristic):
                print('Route found')
                break

            for step in self.get_steps(node[0], step_size, block_diag):
                if (round(step[0], 1), round(step[1], 1), round(step[2], 1)) in history:
                    continue
                path = node[1][:]
                path.append(step)
                history.add((round(step[0], 1), round(step[1], 1), round(step[2], 1)))
                prior = (len(path) + 1) * step_size

                heur = heuristic(step)
                p_queue.push((step, path), prior + heur)

            self.draw_path(path)
        return path

    def path_parameters(self, position):
        target_x, target_y, target_yaw = self.get_target_position()
        x, y, yaw = position
        dist_x = target_x - x
        dist_y = target_y - y
        distance = (dist_x ** 2 + dist_y ** 2) ** 0.5

        path_yaw = math.atan(abs(dist_x / dist_y))
        if (dist_x >= 0) and (dist_y >= 0):
            path_yaw = math.pi - path_yaw
        elif (dist_x <= 0) and (dist_y >= 0):
            path_yaw = path_yaw - math.pi
        elif (dist_x >= 0) and (dist_y <= 0):
            path_yaw = path_yaw
        elif (dist_x <= 0) and (dist_y <= 0):
            path_yaw = -path_yaw

        rotation_1 = abs(path_yaw - target_yaw)
        if rotation_1 > math.pi:
            rotation_1 = abs(rotation_1 - 2 * math.pi)
        rotation_2 = abs(target_yaw - yaw)
        if rotation_2 > math.pi:
            rotation_2 = abs(rotation_2 - 2 * math.pi)
        return distance, rotation_1, rotation_2

    def naive_heuristic(self, position):
        distance, rotation_1, rotation_2 = self.path_parameters(position)
        rotation = rotation_1 + rotation_2
        return 2000 * rotation

    def custom_heuristic(self, position):
        distance, rotation_1, rotation_2 = self.path_parameters(position)
        rotation = rotation_1 + rotation_2
        return distance + 450 * rotation + 100 * rotation

    def target_reached(self, position, heuristic):
        return heuristic(position) < 50

    def get_steps(self, position, step_size, block_diag):
        x, y, yaw = position
        view_width = math.pi * step_size / 180
        var_number = 3

        add_yaw = [view_width / var_number * i for i in range(var_number)]
        add_yaw = [x - view_width / var_number * ((var_number - 1) / 2) for x in add_yaw]
        results = []
        for angl in add_yaw:
            result_yaw = yaw + angl
            result_x = x + step_size * math.sin(result_yaw)
            result_y = y - step_size * math.cos(result_yaw)

            number_of_collisions = 0
            for obstacle in self.get_obstacles():
                if collides([result_x, result_y, result_yaw], obstacle, block_diag):
                    number_of_collisions += 1
            if number_of_collisions > 0:
                continue

            results.append([result_x, result_y, result_yaw])
        return results

    '''================= Interface Methods ================='''

    def get_obstacles(self):
        obstacles = []
        potential_obstacles = self.canvas.find_all()
        for i in potential_obstacles:
            if (i > 2):
                coords = self.canvas.coords(i)
                if coords:
                    obstacles.append(coords)
        return obstacles

    def get_start_position(self):
        x, y = self.get_center(2)  # Purple block has id 2
        yaw = self.get_yaw(2)
        return x, y, yaw

    def get_target_position(self):
        x, y = self.get_center(1)  # Green block has id 1
        yaw = self.get_yaw(1)
        return x, y, yaw

    def get_center(self, id_block):
        coords = self.canvas.coords(id_block)
        center_x, center_y = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)
        return [center_x, center_y]

    def get_yaw(self, id_block):
        center_x, center_y = self.get_center(id_block)
        first_x = 0.0
        first_y = -1.0
        second_x = 1.0
        second_y = 0.0
        points = self.canvas.coords(id_block)
        end_x = (points[0] + points[2]) / 2
        end_y = (points[1] + points[3]) / 2
        direction_x = end_x - center_x
        direction_y = end_y - center_y
        length = math.hypot(direction_x, direction_y)
        unit_x = direction_x / length
        unit_y = direction_y / length
        cos_yaw = unit_x * first_x + unit_y * first_y
        sign_yaw = unit_x * second_x + unit_y * second_y
        if (sign_yaw >= 0):
            return math.acos(cos_yaw)
        else:
            return -math.acos(cos_yaw)

    def get_vertices(self, id_block):
        return self.canvas.coords(id_block)

    '''=================================================='''

    def rotate(self, points, angle, center):
        angle = math.radians(angle)
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        cx, cy = center
        new_points = []

        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append(x_new + cx)
            new_points.append(y_new + cy)

        return new_points

    def start_block(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def in_rect(self, point, rect):
        x_start, x_end = min(rect[::2]), max(rect[::2])
        y_start, y_end = min(rect[1::2]), max(rect[1::2])

        if x_start < point[0] < x_end and y_start < point[1] < y_end:
            return True

    def motion_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                break

        res_cords = []
        try:
            coords
        except:
            return

        for ii, i in enumerate(coords):
            if ii % 2 == 0:
                res_cords.append(i + event.x - widget.start_x)
            else:
                res_cords.append(i + event.y - widget.start_y)

        widget.start_x = event.x
        widget.start_y = event.y
        widget.coords(id, res_cords)
        widget.center = ((res_cords[0] + res_cords[4]) / 2, (res_cords[1] + res_cords[5]) / 2)

    def draw_block(self, points, color):
        x = self.canvas.create_polygon(points, fill=color)
        return x

    def distance(self, x1, y1, x2, y2):
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

    def set_id_block(self, event):
        widget = event.widget

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                coords = widget.coords(i)
                id = i
                widget.id_block = i
                break

        widget.center = ((coords[0] + coords[4]) / 2, (coords[1] + coords[5]) / 2)

    def rotate_block(self, event):
        angle = 0
        widget = event.widget

        if widget.id_block == None:
            for i in range(1, 10):
                if widget.coords(i) == []:
                    break
                if self.in_rect([event.x, event.y], widget.coords(i)):
                    coords = widget.coords(i)
                    id = i
                    widget.id_block == i
                    break
        else:
            id = widget.id_block
            coords = widget.coords(id)

        wx, wy = event.x_root, event.y_root
        try:
            coords
        except:
            return

        block = coords
        center = widget.center
        x, y = block[2], block[3]

        cat1 = self.distance(x, y, block[4], block[5])
        cat2 = self.distance(wx, wy, block[4], block[5])
        hyp = self.distance(x, y, wx, wy)

        if wx - x > 0:
            angle = math.acos((cat1 ** 2 + cat2 ** 2 - hyp ** 2) / (2 * cat1 * cat2))
        elif wx - x < 0:
            angle = -math.acos((cat1 ** 2 + cat2 ** 2 - hyp ** 2) / (2 * cat1 * cat2))

        new_block = self.rotate([block[0:2], block[2:4], block[4:6], block[6:8]], angle, center)
        self.canvas.coords(id, new_block)

    def delete_block(self, event):
        widget = event.widget.children["!canvas"]

        for i in range(1, 10):
            if widget.coords(i) == []:
                break
            if self.in_rect([event.x, event.y], widget.coords(i)):
                widget.coords(i, [0, 0])
                break

    def create_block(self, event):
        block = [[0, 100], [100, 100], [100, 300], [0, 300]]

        id = self.draw_block(block, "black")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def make_draggable(self, widget):
        widget.bind("<Button-1>", self.drag_start)
        widget.bind("<B1-Motion>", self.drag_motion)

    def drag_start(self, event):
        widget = event.widget
        widget.start_x = event.x
        widget.start_y = event.y

    def drag_motion(self, event):
        widget = event.widget
        x = widget.winfo_x() - widget.start_x + event.x + 200
        y = widget.winfo_y() - widget.start_y + event.y + 100
        widget.place(rely=0.0, relx=0.0, x=x, y=y)

    def create_button_create(self):
        button = Button(
            text="New",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=0.0, x=200, y=100, anchor=SE, width=200, height=100)
        button.bind("<Button-1>", self.create_block)

    def create_green_block(self, center_x):
        block = [[center_x - 50, 100],
                 [center_x + 50, 100],
                 [center_x + 50, 300],
                 [center_x - 50, 300]]

        id = self.draw_block(block, "green")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_purple_block(self, center_x, center_y):
        block = [[center_x - 50, center_y - 300],
                 [center_x + 50, center_y - 300],
                 [center_x + 50, center_y - 100],
                 [center_x - 50, center_y - 100]]

        id = self.draw_block(block, "purple")

        self.canvas.tag_bind(id, "<Button-1>", self.start_block)
        self.canvas.tag_bind(id, "<Button-3>", self.set_id_block)
        self.canvas.tag_bind(id, "<B1-Motion>", self.motion_block)
        self.canvas.tag_bind(id, "<B3-Motion>", self.rotate_block)

    def create_button_go(self):
        button = Button(
            text="Go",
            bg="#555555",
            activebackground="blue",
            borderwidth=0
        )

        button.place(rely=0.0, relx=1.0, x=0, y=200, anchor=SE, width=100, height=200)
        button.bind("<Button-1>", self.go)

    def run(self):
        root = self.root

        self.create_button_create()
        self.create_button_go()
        self.create_green_block(self.width / 2)
        self.create_purple_block(self.width / 2, self.height)

        root.bind("<Delete>", self.delete_block)

        root.mainloop()

    def __init__(self):
        self.root = Tk()
        self.root.title("")
        self.width = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.root.geometry('{}x{}'.format(self.width, self.height))
        self.canvas = Canvas(self.root, bg="#777777", height=self.height, width=self.width)
        self.canvas.pack()
        # self.points = [0, 500, 500/2, 0, 500, 500]

if __name__ == "__main__":
    run = Window()
    run.run()