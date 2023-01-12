import pygame, sys
from queue import PriorityQueue
from Button import Button

# https://www.youtube.com/watch?v=JtiK0DOeI4A&t=4718s&ab_channel=TechWithTim

pygame.init()
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
map = pygame.image.load("map.png")
pygame.display.set_caption("PathFinding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


# A spot is one of the cubes on the grid. Spot = nodes
class Spot:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():  # moving DOWN a row
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # moving UP a row
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():  # moving RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # moving LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    # lt mean less than. What happens when we compare spots
    def __lt__(self, other):
        return False


# Importing Font
def get_font(size):
    return pygame.font.Font('font.ttf', size)


def main_menu():
    run = True

    while run:

        WIN.blit(map, (0, 0))

        menu_text = get_font(75).render("Which algorithm?", True, "BLUE")
        menu_rect = menu_text.get_rect(center=(400, 100))

        WIN.blit(menu_text, menu_rect)

        main_menu_pos = pygame.mouse.get_pos()

        button_1 = Button(image=pygame.image.load('Play Rect.png'), pos=(400, 300), text_input='A-star',
                          font=get_font(75), base_color="ORANGE", hovering_color='WHITE')
        button_2 = Button(image=pygame.image.load('Play Rect.png'), pos=(400, 450), text_input='Dijkstra',
                          font=get_font(75), base_color="PURPLE", hovering_color='WHITE')
        button_3 = Button(image=pygame.image.load('Play Rect.png'), pos=(400, 600), text_input='QUIT',
                          font=get_font(75), base_color="RED", hovering_color='GREEN')

        for button in [button_1, button_2, button_3]:
            button.changeColor(main_menu_pos)
            button.update(WIN)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                if button_1.checkinput(main_menu_pos):
                    main_A(WIN, WIDTH)
                if button_2.checkinput(main_menu_pos):
                    main_D(WIN, WIDTH)
                if button_3.checkinput(main_menu_pos):
                    pygame.quit()
                    sys.exit

        pygame.display.update()
    pygame.quit()


# Heuristic Function
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def a_star(draw, grid, start, end):  # A* pathfinder algo
    count = 0
    open_set = PriorityQueue()  # gives us the min element
    open_set.put(
        (0, count, start))  # count used to break ties if 2 fscores are the same. start = start node. 0 current fscore
    came_from = {}  # keeps track of what nodes came from where
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    # Complicated data structure stuff
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # Indexing at 2 bc: our open_set will store fscore, count, node. We want just the node.
        # if two fscores are the same we look at the count and take the first
        current = open_set.get()[2]
        open_set_hash.remove(current)

        # If this is true we found the path and need to make path
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        # otherwise continue
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1  # it takes one to travel from node to node

            if temp_g_score < g_score[neighbor]:  # if we found a better to reach neigh update and keep track
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))  # we add this neighbor bc better path
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False


def dijkstra(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}  # keeps track of what nodes came from where
    g_score = {spot: float("inf") for row in grid for spot in row}
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row}
    f_score[start] = 0

    # Complicated data structure stuff
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        # If this is true we found the path and need to make path
        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        # otherwise continue
        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1  # it takes one to travel from node to node

            if temp_g_score < g_score[neighbor]:  # if we found a better to reach neigh update and keep track
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))  # we add this neighbor bc better path
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw()

        if current != start:
            current.make_closed()

    return False


def make_grid(rows, width):
    grid = []
    # width of the cubes in the grid
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)  # calling the class we created above
            grid[i].append(spot)
    return grid


def draw_grid(win, rows, width):
    gap = width // rows
    # Creates horizontal lines across grid.
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
        # Creates vertical lines across grid.
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
    # Feels the whole screen with one color
    WIN.fill(WHITE)
    # all spots are stored in the grid and the grid is stored in the rows. Looping through that
    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()


# Mouse position function - to fig out which cube you are clicking on (and its location)
def get_clicked_position(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


# Main function. All checks we have to do. Plus starting
# A-star - Main Function
def main_A(win, width):
    rows = 50
    grid = make_grid(rows, width)

    start = None
    end = None

    run = True
    while run:
        draw(win, grid, rows, width)
        # looping through all "events". ie did something click on a mouse? a barrier was put up?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            # Left mouse click
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, rows, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            # right mouse click
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, rows, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            # Starting the algo
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    a_star(lambda: draw(win, grid, rows, width), grid, start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(rows, width)

                if event.key == pygame.K_b:
                    main_menu()

    pygame.quit()


# Dijkstra - MAIN Funtion
def main_D(win, width):
    rows = 50
    grid = make_grid(rows, width)

    start = None
    end = None

    run = True
    while run:
        draw(win, grid, rows, width)
        # looping through all "events". ie did something click on a mouse? a barrier was put up?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            # Left mouse click
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, rows, width)
                spot = grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()

                elif not end and spot != start:
                    end = spot
                    end.make_end()

                elif spot != end and spot != start:
                    spot.make_barrier()

            # right mouse click
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_position(pos, rows, width)
                spot = grid[row][col]
                spot.reset()
                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            # Starting the algo
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid)

                    dijkstra(lambda: draw(win, grid, rows, width), grid, start, end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(rows, width)

                if event.key == pygame.K_b:
                    main_menu()

    pygame.quit()


main_menu()
