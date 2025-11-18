import numpy as np
import matplotlib.pyplot as plt

MAP_PATH = 'maz_map.txt'
SAVE_PATH = 'map.png'
START = None # 시작위치 
GOAL = None # 목표위치

NEIGHBORS = [(1,0), (-1,0), (0,1), (0,-1)]

# state = (row, col)
# neighbors = [(row+1,col), (row-1,col), (row,col+1), (row,col-1)]

def grid_to_numeric(grid):
    """
    그리드를 숫자로 변환
    """
    h = len(grid)
    w = len(grid[0])
    arr = np.zeros((h, w))

    for y in range(h):
        for x in range(w):
            if grid[y][x] == '1': arr[y, x] = 1 # wall
            elif grid[y][x] == '0': arr[y, x] = 0  # path
            elif grid[y][x] == 'S': arr[y, x] = 2  # start
            elif grid[y][x] == 'G': arr[y, x] = 3  # goal
            elif grid[y][x] == 'X': arr[y, x] = 4 # visited
            else: arr[y, x] = 5 # unknown

    return arr

def save_map_image(arr, save_path=SAVE_PATH):
    """
    숫자로 변환된 그리드를 이미지로 저장
    """
    cmap = {
        0: (1,1,1),      # path: white
        1: (0,0,0),      # wall: black
        2: (0.2,0.4,1),  # start: blue
        3: (1,0.2,0.2),  # goal: red
        4: (0,1,0),      # visited: green
        5: (0.5,0.5,0.5), # unknown: gray
    }

    rgb = np.zeros((arr.shape[0], arr.shape[1], 3))
    for y in range(arr.shape[0]):
        for x in range(arr.shape[1]):
            rgb[y, x] = cmap[arr[y, x]]

    plt.imshow(rgb)
    plt.axis('off')

    plt.savefig(save_path, dpi=300, bbox_inches='tight', pad_inches=0)
    plt.close()

def print_grid(grid):
    """
    그리드를 콘솔에 출력
    """
    print("Printing grid:")
    print("Grid Size: ", len(grid), "x", len(grid[0]))
    for row in grid:
        print(" ".join(row))

def is_valid(grid, row, col):
    """
    이동할 수 있는 여부 확부
    """
    if row < 0 or row >= len(grid): return False
    if col < 0 or col >= len(grid[0]): return False
    if grid[row][col] == "1": return False
    return True

def neighbors(grid, row, col):
    """
    이동할 수 있는 이웃 좌표 반환
    """
    for n_row, n_col in NEIGHBORS:
        if is_valid(grid, row+n_row, col+n_col):
            yield (row+n_row, col+n_col)

def read_map(map_path):
    """
    파일에서 그리드를 읽어옴
    """
    with open(map_path, 'r', encoding='utf-8') as f:
        grid = []
        start = None
        goal = None
        for row, line in enumerate(f):
            single_row = line.strip()
            for col, cell in enumerate(single_row):
                if cell == "S":
                    start = (row, col)
                elif cell == "G":
                    goal = (row, col)
            grid.append(list(single_row))
    return grid, start, goal


def simple_example_dfs(grid, start, goal):
    """
    DFS를 이용한 경로 탐색 예제
    """
    visited = set()
    stack = [(start, [start])]  # (현재 위치, 지금까지의 경로)
    
    while stack:
        current, path = stack.pop()
        
        if current == goal:
            return path
        
        if current in visited:
            continue
        
        visited.add(current)
        
        for neighbor in neighbors(grid, current[0], current[1]):
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor]))
    

# 맵 파일에서 그리드와 시작/목표 위치 읽기
grid, START, GOAL = read_map(MAP_PATH)
# 그리드 출력 (2차원 배열)
print_grid(grid)
print("START: ", START)
print("GOAL: ", GOAL)

# DFS 알고리즘으로 시작점에서 목표점까지의 경로 탐색
dfs_path = simple_example_dfs(grid, START, GOAL)

# 찾은 경로를 그리드에 'X'로 표시 (시작점과 목표점 제외)
for i in dfs_path[1:-1]:
    grid[i[0]][i[1]] = 'X'

# 그리드를 숫자 배열로 변환 후 이미지로 저장
arr = grid_to_numeric(grid)
save_map_image(arr)
print("Simple Example DFS done")