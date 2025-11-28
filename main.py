# -*- coding: utf-8 -*-
"""
미로 탐색 알고리즘 통합 프로그램
- DFS, BFS, A*, Greedy 알고리즘 지원
"""

import heapq
import collections
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import os

# 상수 정의
MAP_PATH = 'maze_map.txt'
SAVE_PATH = 'map.png'

"""유틸리티 함수들"""

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

def get_neighbors(maze, current_node):
    """현재 노드에서 이동 가능한 (벽이 아닌) 인접 노드를 반환합니다."""
    rows = len(maze)
    cols = len(maze[0])
    r, c = current_node

    # 상, 하, 좌, 우 4방향 이동
    moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []

    for dr, dc in moves:
        nr, nc = r + dr, c + dc  # 새 행/열 좌표

        # 미로 경계 확인 및 벽('1')이 아닌지 확인
        if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] != '1':
            neighbors.append((nr, nc))

    return neighbors

def print_grid(grid):
    """
    그리드를 콘솔에 출력
    """
    print("Printing grid:")
    print("Grid Size: ", len(grid), "x", len(grid[0]))
    for row in grid:
        print(" ".join(row))

def reconstruct_path(parent, start, goal):
    """부모 포인터(parent) 딕셔너리를 이용해 목표점에서 시작점까지 경로를 역추적합니다."""
    path = []
    current = goal

    # 목표점에서 시작점까지 역순으로 이동
    while current != start:
        path.append(current)
        if current in parent:
            current = parent[current]
        else:
            return []  # 경로를 찾지 못한 경우
    path.append(start)
    path.reverse()
    return path

"""휴리스틱 함수들"""

def manhattan_distance_heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def manhattan_tie_breaking_heuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    # 기본 멘해튼 거리에 0.1%의 가중치를 더함 -> 같은 경로가 많아 탐색이 느려지는 것을 방지
    return (dx + dy) * 1.001

def euclidean_heuristic(a, b):
    # 피타고라스 정리: sqrt((x1-x2)^2 + (y1-y2)^2)
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

"""탐색 알고리즘들"""

def dfs_search(maze, start, goal):
    """
    DFS를 이용한 경로 탐색
    """
    visited = set()
    stack = [(start, [start])]  # (현재 위치, 지금까지의 경로)
    visited_history = [start]
    
    while stack:
        current, path = stack.pop()
        
        if current == goal:
            return path, len(visited), visited_history
        
        if current in visited:
            continue
        
        visited.add(current)
        visited_history.append(current)
        
        for neighbor in get_neighbors(maze, current):
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor]))
    
    return None, len(visited), visited_history

def bfs_search(maze, start, goal):
    """너비 우선 탐색 (BFS) 구현: 최단 경로 보장"""
    queue = collections.deque([start])
    visited = {start}
    parent = {start: None}
    visited_history = [start]  # 시각화용
    expanded_nodes = 0

    while queue:
        current_node = queue.popleft()
        expanded_nodes += 1

        if current_node == goal:
            path = reconstruct_path(parent, start, goal)
            return path, expanded_nodes, visited_history

        for neighbor in get_neighbors(maze, current_node):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current_node
                queue.append(neighbor)
                visited_history.append(neighbor)

    return None, expanded_nodes, visited_history

def astar(maze, start, goal, heuristic):
    """
    A* 알고리즘으로 경로 찾기
    """
    open_set = []
    heapq.heappush(open_set, (0, 0, start, [start]))

    visited = set()
    visited_history = []

    while open_set:
        f, g, current, path = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)
        visited_history.append(current)

        if current == goal:
            return path, len(visited), visited_history

        for neighbor in get_neighbors(maze, current):
            if neighbor in visited:
                continue

            new_g = g + 1
            h = heuristic(neighbor, goal)
            f_score = new_g + h

            heapq.heappush(open_set, (f_score, new_g, neighbor, path + [neighbor]))

    return None, len(visited), visited_history

def greedy(maze, start, goal, heuristic):
    """
    Greedy 알고리즘으로 경로 찾기
    """
    start_h = heuristic(start, goal)
    open_set = []
    heapq.heappush(open_set, (start_h, 0, start, [start]))

    visited = set()
    visited_history = []

    while open_set:
        h, g, current, path = heapq.heappop(open_set)

        if current in visited:
            continue
        visited.add(current)
        visited_history.append(current)

        if current == goal:
            return path, len(visited), visited_history

        for neighbor in get_neighbors(maze, current):
            if neighbor in visited:
                continue
            new_g = g + 1
            new_h = heuristic(neighbor, goal)
            heapq.heappush(open_set, (new_h, new_g, neighbor, path + [neighbor]))

    return None, len(visited), visited_history

"""시각화 함수"""

def create_frame(maze, visited_so_far, current_path, start, goal, frame_num, temp_dir):
    """
    단일 프레임을 생성하고 저장합니다.
    """
    rows = len(maze)
    cols = len(maze[0])

    # 맵 초기화: 0(길), 1(벽)
    grid = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == '1':
                grid[r][c] = 1  # 벽

    # 방문했던 노드 표시 (밝은 녹색: 2)
    for r, c in visited_so_far:
        if grid[r][c] == 0:
            grid[r][c] = 2

    # 현재 경로 표시 (노란색: 3)
    if current_path:
        for r, c in current_path:
            if (r, c) != start and (r, c) != goal:
                grid[r][c] = 3

    # 시작점과 목표점 표시 (파란색: 4, 빨간색: 5)
    grid[start[0]][start[1]] = 4
    grid[goal[0]][goal[1]] = 5

    # --- 색상 스키마 변환 (요청된 검은색 배경/흰색 길) ---
    transformed_grid = np.zeros_like(grid)
    for r in range(rows):
        for c in range(cols):
            val = grid[r][c]
            if val == 0:  # 원본 0 (길) -> 요청 1 (흰색)
                transformed_grid[r][c] = 1
            elif val == 1:  # 원본 1 (벽) -> 요청 0 (검은색)
                transformed_grid[r][c] = 0
            else:
                transformed_grid[r][c] = val

    # 색상 맵 정의 (0:검은색(벽), 1:흰색(길), 2:초록색(방문), 3:노란색(경로), 4:파란색(시작), 5:빨간색(목표))
    colors = ['black', 'white', 'lightgreen', 'yellow', 'blue', 'red']
    cmap = plt.cm.colors.ListedColormap(colors)
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    # 그림 설정 및 저장
    fig, ax = plt.subplots(figsize=(cols/10, rows/10), dpi=100)
    ax.imshow(transformed_grid, cmap=cmap, norm=norm, origin='upper')

    # 격자선 및 기타 설정
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="gray", linestyle='-', linewidth=0.1)
    ax.tick_params(which="minor", size=0)
    ax.tick_params(which="major", bottom=False, left=False, labelbottom=False, labelleft=False)

    plt.title(f"Frame {frame_num}", fontsize=10)
    plt.tight_layout()

    # 임시 파일로 저장
    frame_path = os.path.join(temp_dir, f"frame_{frame_num:05d}.png")
    plt.savefig(frame_path, dpi=100)
    plt.close(fig)  # 메모리 해제
    
    return frame_path

def visualize_maze(maze, path, visited_history, start, goal, filename):
    """
    미로 탐색 결과를 시각화하고 이미지 파일로 저장합니다.
    """
    rows = len(maze)
    cols = len(maze[0])

    # 맵 초기화: 0(길), 1(벽)
    grid = np.zeros((rows, cols))
    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == '1':
                grid[r][c] = 1  # 벽

    # 방문했던 노드 표시 (밝은 녹색: 2)
    for r, c in visited_history:
        if grid[r][c] == 0:
            grid[r][c] = 2

    # 최종 경로 표시 (노란색: 3)
    if path:
        for r, c in path:
            if (r, c) != start and (r, c) != goal:
                grid[r][c] = 3

    # 시작점과 목표점 표시 (파란색: 4, 빨간색: 5)
    grid[start[0]][start[1]] = 4
    grid[goal[0]][goal[1]] = 5

    # --- 색상 스키마 변환 (요청된 검은색 배경/흰색 길) ---
    transformed_grid = np.zeros_like(grid)
    for r in range(rows):
        for c in range(cols):
            val = grid[r][c]
            if val == 0:  # 원본 0 (길) -> 요청 1 (흰색)
                transformed_grid[r][c] = 1
            elif val == 1:  # 원본 1 (벽) -> 요청 0 (검은색)
                transformed_grid[r][c] = 0
            else:
                transformed_grid[r][c] = val

    # 색상 맵 정의 (0:검은색(벽), 1:흰색(길), 2:초록색(방문), 3:노란색(경로), 4:파란색(시작), 5:빨간색(목표))
    colors = ['black', 'white', 'lightgreen', 'yellow', 'blue', 'red']
    cmap = plt.cm.colors.ListedColormap(colors)
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)

    # 그림 설정 및 저장
    fig, ax = plt.subplots(figsize=(cols/10, rows/10), dpi=300)
    ax.imshow(transformed_grid, cmap=cmap, norm=norm, origin='upper')

    # 격자선 및 기타 설정
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="gray", linestyle='-', linewidth=0.1)
    ax.tick_params(which="minor", size=0)
    ax.tick_params(which="major", bottom=False, left=False, labelbottom=False, labelleft=False)

    plt.title(f"{filename.split('.')[0]} Result", fontsize=10)
    plt.tight_layout()

    # 이미지 파일 저장
    plt.savefig(filename, dpi=300)
    plt.close(fig)  # 메모리 해제

    return f"결과 이미지 저장 완료: {filename}"

def create_animation_gif(maze, path, visited_history, start, goal, gif_filename, frame_interval=10):
    """
    탐색 과정을 애니메이션 GIF로 생성합니다.
    frame_interval: 몇 개의 방문 노드마다 프레임을 생성할지 (작을수록 더 많은 프레임)
    """
    print(f"  애니메이션 GIF 생성 중... (총 {len(visited_history)}개 노드 방문)")
    
    # 임시 디렉토리 생성
    temp_dir = "temp_frames"
    os.makedirs(temp_dir, exist_ok=True)
    
    try:
        frames = []
        frame_count = 0
        
        # 단계별로 프레임 생성
        for i in range(0, len(visited_history), max(1, frame_interval)):
            visited_so_far = visited_history[:i+1]
            # 현재까지의 경로 추정 (간단한 버전)
            current_path = visited_so_far if i < len(visited_history) - 1 else path
            
            frame_path = create_frame(maze, visited_so_far, [], start, goal, frame_count, temp_dir)
            frames.append(frame_path)
            frame_count += 1
        
        # 최종 경로가 있는 경우 마지막 프레임 추가
        if path:
            # 경로를 단계별로 표시하는 프레임들 추가
            for i in range(0, len(path), max(1, len(path) // 20)):  # 경로를 20프레임으로 나눔
                path_so_far = path[:i+1]
                frame_path = create_frame(maze, visited_history, path_so_far, start, goal, frame_count, temp_dir)
                frames.append(frame_path)
                frame_count += 1
        
        # GIF 생성
        if frames:
            images = [Image.open(frame) for frame in frames]
            images[0].save(
                gif_filename,
                save_all=True,
                append_images=images[1:],
                duration=50,  # 각 프레임당 50ms (0.05초)
                loop=0  # 무한 반복
            )
            print(f"  GIF 저장 완료: {gif_filename} ({len(frames)} 프레임)")
        
        # 임시 파일 정리
        for frame in frames:
            if os.path.exists(frame):
                os.remove(frame)
        if os.path.exists(temp_dir):
            os.rmdir(temp_dir)
            
    except Exception as e:
        print(f"  GIF 생성 중 오류 발생: {e}")
        # 임시 파일 정리
        if os.path.exists(temp_dir):
            for file in os.listdir(temp_dir):
                os.remove(os.path.join(temp_dir, file))
            os.rmdir(temp_dir)

"""실행 함수들"""

def run_algorithm(name, algorithm, maze, start, goal, filename, heuristic=None, create_gif=False):
    """
    알고리즘을 실행하고 결과를 시각화합니다.
    create_gif: True이면 애니메이션 GIF도 생성합니다.
    """
    print(f"\n--- {name} 알고리즘 실행 ---")
    print(f"시작점: {start}, 목표점: {goal}")

    start_time = time.time()
    if heuristic:
        path, expanded_nodes, visited_history = algorithm(maze, start, goal, heuristic)
    else:
        path, expanded_nodes, visited_history = algorithm(maze, start, goal)
    end_time = time.time()

    execution_time = end_time - start_time
    path_length = len(path) - 1 if path else "경로 없음"

    print(f"탐색 완료: {'성공' if path else '실패'}")
    print(f"경로 길이: {path_length}")
    print(f"확장 노드 수: {expanded_nodes}")
    print(f"탐색 시간: {execution_time:.4f} 초")

    if path:
        visualize_maze(maze, path, visited_history, start, goal, filename)
        print(f"결과 이미지 저장: {filename}")
        
        # GIF 생성
        if create_gif:
            gif_filename = filename.replace('.png', '.gif')
            # 프레임 간격 조정 (노드 수에 따라)
            frame_interval = max(1, len(visited_history) // 100)  # 최대 100프레임
            create_animation_gif(maze, path, visited_history, start, goal, gif_filename, frame_interval)
    else:
        print("최단 경로를 찾지 못하여 시각화를 수행할 수 없습니다.")

"""메인 실행 블록"""

def main():
    # 맵 파일에서 그리드와 시작/목표 위치 읽기
    grid, start, goal = read_map(MAP_PATH)
    
    if not start or not goal:
        print("오류: 미로에서 시작점(S) 또는 목표점(G)을 찾을 수 없습니다.")
        return

    # 그리드 출력
    print_grid(grid)
    print(f"\n맵 크기: {len(grid)}x{len(grid[0])}")
    print(f"시작점: {start}, 목표점: {goal}\n")

    # 각 알고리즘 실행 (GIF 생성 여부 설정)
    CREATE_GIF = True  # False로 설정하면 GIF 생성 안 함
    
    run_algorithm("DFS", dfs_search, grid, start, goal, "dfs_result.png", create_gif=CREATE_GIF)
    run_algorithm("BFS", bfs_search, grid, start, goal, "bfs_result.png", create_gif=CREATE_GIF)
    run_algorithm("A* (Manhattan)", astar, grid, start, goal, "astar_manhattan.png", manhattan_distance_heuristic, create_gif=CREATE_GIF)
    run_algorithm("A* (Euclidean)", astar, grid, start, goal, "astar_euclidean.png", euclidean_heuristic, create_gif=CREATE_GIF)
    run_algorithm("A* (Tie-breaking)", astar, grid, start, goal, "astar_tie_breaking.png", manhattan_tie_breaking_heuristic, create_gif=CREATE_GIF)
    run_algorithm("Greedy (Manhattan)", greedy, grid, start, goal, "greedy_manhattan.png", manhattan_distance_heuristic, create_gif=CREATE_GIF)
    run_algorithm("Greedy (Euclidean)", greedy, grid, start, goal, "greedy_euclidean.png", euclidean_heuristic, create_gif=CREATE_GIF)
    run_algorithm("Greedy (Tie-breaking)", greedy, grid, start, goal, "greedy_tie_breaking.png", manhattan_tie_breaking_heuristic, create_gif=CREATE_GIF)

    print("\n모든 알고리즘 실행 완료!")

if __name__ == '__main__':
    main()
