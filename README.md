# AI Map - 미로 탐색 알고리즘 시각화

다양한 경로 탐색 알고리즘을 사용하여 미로를 탐색하고 결과를 시각화하는 프로그램입니다.

## 지원 알고리즘

- **DFS** (Depth-First Search): 깊이 우선 탐색
- **BFS** (Breadth-First Search): 너비 우선 탐색 (최단 경로 보장)
- **A\***: 휴리스틱 기반 최적 경로 탐색
  - Manhattan 거리
  - Euclidean 거리
  - Tie-breaking 휴리스틱
- **Greedy**: 탐욕 알고리즘
  - Manhattan 거리
  - Euclidean 거리
  - Tie-breaking 휴리스틱

## 준비

### 1. 의존성 설치

```bash
pip install numpy matplotlib pillow
```

또는 `pyproject.toml`이 있는 경우:

```bash
pip install -e .
```

## 실행

```bash
python main.py
```

## 출력 파일

실행 후 각 알고리즘마다 다음 파일들이 생성됩니다:

- `{algorithm}_result.png`: 최종 결과 이미지
- `{algorithm}_result.gif`: 탐색 과정 애니메이션 (GIF)

예시:
- `dfs_result.png`, `dfs_result.gif`
- `bfs_result.png`, `bfs_result.gif`
- `astar_manhattan.png`, `astar_manhattan.gif`
- `greedy_euclidean.png`, `greedy_euclidean.gif`
- 등등...

## 시각화 색상 설명

- **검은색**: 벽 (이동 불가)
- **흰색**: 길 (이동 가능)
- **초록색**: 방문한 노드 (탐색 과정에서 방문한 모든 곳)
- **노란색**: 최종 경로 (시작점에서 목표점까지의 최종 경로)
- **파란색**: 시작점 (S)
- **빨간색**: 목표점 (G)

## 설정

`main.py`의 `main()` 함수에서 GIF 생성 여부를 설정할 수 있습니다:

```python
CREATE_GIF = True  # True: GIF 생성, False: PNG만 생성
```

## 맵 파일 형식

`maze_map.txt` 파일 형식:
- `0`: 이동 가능한 길
- `1`: 벽 (이동 불가)
- `S`: 시작점
- `G`: 목표점

## 성능 지표

각 알고리즘 실행 시 다음 정보가 출력됩니다:
- 경로 길이
- 확장 노드 수 (탐색한 노드 개수)
- 탐색 시간
