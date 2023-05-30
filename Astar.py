from __future__ import annotations
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional, Any
import heapq
from getMap import Edge, Map
from functools import reduce

T = TypeVar('T')


class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []

    def empty(self):
        return not self.elements

    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))

    def get(self) -> T:
        return heapq.heappop(self.elements)[1]


def heuristic(graph, cur, goal):
    cur_start = graph.edge_dic[cur].start_node
    goal_start = graph.edge_dic[goal].start_node
    return abs(cur_start.x_cor - goal_start.x_cor) + abs(cur_start.y_cor - goal_start.y_cor)


def a_star_search(graph: Map, start, goal, del_set):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far: dict[Any, int] = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.edge_dic[current].out_edge_set:
            new_cost = cost_so_far[current] + graph.edge_dic[next].cost
            if (next not in cost_so_far or new_cost < cost_so_far[next]) and next not in del_set:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(graph, current, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        if current not in came_from.keys():
            return []
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path


def compute_cost(cur_path, cur_map):
    """
    用于计算path的长度/cost
    :param cur_path: 路线
    :param cur_map: 地图
    :return:
    """
    cost = 0
    for item in cur_path:
        cost += cur_map.edge_dic[item].cost
    return cost


def TopK_astar(graph, start, goal, k):
    """
    计算前k条最短路的A*算法
    :param k: 计算前k条
    :param graph: 地图
    :param start: 起点
    :param goal: 终点
    :return:
    """
    del_set = []
    # 先计算最短路径
    came_from_dic, _ = a_star_search(graph, start, goal, del_set)
    init_path = reconstruct_path(came_from_dic, start, goal)

    Path_set = [{'path': init_path, 'cost': compute_cost(init_path, init_map)}]
    # 遍历初始路径
    for i in range(1, len(init_path) - 1):
        cur_path = init_path[i]
        del_set = [cur_path]
        came_from_dic, _ = a_star_search(graph, start, goal, del_set)
        path_i = reconstruct_path(came_from_dic, start, goal)
        if len(path_i) == 0:
            continue
        Path_set.append({'path': path_i, 'cost': compute_cost(path_i, init_map)})

    run_function = lambda x, y: x if y in x else x + [y]
    new_list = reduce(run_function, [[], ] + Path_set)
    final_set = sorted(new_list, key=lambda x: x['cost'])
    min_k = min(k, len(final_set))
    return final_set[0:min_k]


if __name__ == '__main__':
    init_map = Map()
    TopK_astar(init_map, 7140, 56, 10)
