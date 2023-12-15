"""
多负载场景下的无冲突路径规划问题
2023年12月11日
"""
import copy
from getMap import Edge, Map
import time
import numpy as np
from Astar import PriorityQueue
from functools import reduce
import sys

sys.setrecursionlimit(30000000)
INF = 10000000000


class Mission:
    def __init__(self, idx, arrT, start_1, end_1, start_2, end_2, mission_type, init_map):
        self.map = init_map
        # idx是GroupTaskId
        self.mission_id = idx
        # 存储子任务id，默认顺序为先get后put
        self.arr_t = arrT  # 到达时间点
        self.start_arc = self.map.edge_dic[start_1]  # 起点所在边
        self.middle_arc_1 = self.map.edge_dic[start_2]  # 途径点1
        self.middle_arc_2 = self.map.edge_dic[end_1]  # 途径点2
        self.end_arc = self.map.edge_dic[end_2]  # 终点所在边
        self.start_node = self.start_arc.start_node  # 起点
        self.end_node = self.start_arc.end_node  # 终点
        self.path = []  # 总路径
        self.agv = None  # AGV
        self.finish_time = -1
        self.arc_win = {}  # 存储path上第几个arc对应的时间窗
        self.arc_win_id = {}  # 存储path上arc时间窗的插入点
        self.arc_status = []  # 存储path上第几个arc的负载状态
        self.mission_type = mission_type
        self.cost = 0

    @staticmethod
    def distance(start_node, end_node):
        return abs(start_node.x_cor - end_node.x_cor) + abs(start_node.y_cor - end_node.y_cor)

    def getAGV(self, idle_agvs):
        """
        为任务寻找最近的空闲车辆
        :param idle_agvs: 空车集合
        :return: 返回选择的agv，方便后续移除
        """
        min_dis = INF
        for agv in idle_agvs:
            # 车辆完成任务后，就长久占据终点的arc，如果出现任务以该arc为子任务节点，则分配该车给任务
            if agv.cur_loc.id == self.start_arc.id:
                self.agv = agv
                break
            else:
                if self.distance(self.start_node, agv.cur_loc.start_node) < min_dis:
                    min_dis = self.distance(self.start_node, agv.cur_loc.start_node)
                    self.agv = agv
        return self.agv

    def aStar_search(self, start, goal, del_set):
        """
        寻路方法，找到终点返回终点path,否则返回None
        :param del_set: 删除的边集合
        :param start: 起点
        :param goal: 终点
        :return:
        """
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            for next in self.map.edge_dic[current].out_edge_set:
                new_cost = cost_so_far[current] + self.map.edge_dic[next].cost
                if (next not in cost_so_far or new_cost < cost_so_far[next]) and next not in del_set:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current
        return came_from, cost_so_far

    def heuristic(self, cur, goal):
        """
        启发函数
        :param cur: 当前边
        :param goal: 目标终点
        :return:
        """
        cur_start = self.map.edge_dic[cur].start_node
        goal_start = self.map.edge_dic[goal].start_node
        return abs(cur_start.x_cor - goal_start.x_cor) + abs(cur_start.y_cor - goal_start.y_cor)

    @staticmethod
    def reconstruct_path(came_from, start, goal):
        """
        构造路径
        :param came_from: 矩阵
        :param start: 起点
        :param goal: 终点
        :return:
        """
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

    def scheduling_path(self):
        """
        函数作用: 为任务规划路径，包含两个部分，从AGV所在位置到任务起点、从起点到终点
        """
        del_set = []
        agv_location = self.agv.cur_loc.id
        came_from1, _ = self.aStar_search(agv_location, self.start_arc.id, del_set)
        path1 = self.reconstruct_path(came_from1, agv_location, self.start_arc.id)
        if not path1:
            return []
        # 添加路径1
        path1.remove(agv_location)
        self.arc_status += [0] * len(path1)

        # 规划从起点到途径点1的路径
        came_from2, _ = self.aStar_search(self.start_arc.id, self.middle_arc_1.id, del_set)
        path2 = self.reconstruct_path(came_from2, self.start_arc.id, self.middle_arc_1.id)
        if not path2:
            return []
        # 添加路径2
        path2.remove(self.start_arc.id)
        if self.mission_type == 1:
            self.arc_status += [0] * len(path2)
        else:
            self.arc_status += [1] * len(path2)

        # 规划途径点1到途径点2的路径
        came_from3, _ = self.aStar_search(self.middle_arc_1.id, self.middle_arc_2.id, del_set)
        path3 = self.reconstruct_path(came_from3, self.middle_arc_1.id, self.middle_arc_2.id)
        if not path3:
            return []
        path3.remove(self.middle_arc_1.id)
        self.arc_status += [1] * len(path3)

        # 规划途径点2到终点的路径
        came_from4, _ = self.aStar_search(self.middle_arc_2.id, self.end_arc.id, del_set)
        path4 = self.reconstruct_path(came_from4, self.middle_arc_2.id, self.end_arc.id)
        if not path4:
            return []
        path4.remove(self.middle_arc_2.id)
        self.arc_status += [0] * len(path4)
        self.path = path1 + path2 + path3 + path4
        return self.path


class AGV:
    def __init__(self, idx, loc):
        self.mission = None
        self.agv_id = idx
        self.cur_loc = loc

    def release_AGV(self):
        self.mission = None

    def __repr__(self):
        return 'agv_' + str(self.agv_id)


class MainAlgo:
    def __init__(self):
        self.init_map = Map()  # 默认地图
        self.serve_missions = list()  # 正在插时间窗或者正在下发控制路径的任务
        self.wait_missions = list()
        self.finish_missions = list()  # 已经结束的任务
        self.start_time = 0  # 程序开始的时间作为起始时间
        self.idle_agvs = list()  # 闲置的agv
        self.safe_time = 0
        self.serve_agvs = list()
        self.mission_num = 0
        self.agv_num = 0
        self.total_conflict_time = 0
        self.total_time = 0

    def mission_arrive(self):
        """
        通过rpc接口获取仿真器下发的任务
        :return:
        """
        path_file = 'mission_data/mission_1.txt'
        with open(path_file, encoding='utf-8') as file:
            content = file.readlines()
        count = 0
        for line in content:
            split_line = line.split()
            mission_id = int(split_line[0])
            arr_t = int(split_line[1]) * 100
            start_1 = int(split_line[2])
            start_2 = int(split_line[3])
            end_1 = int(split_line[4])
            end_2 = int(split_line[5])
            mission_type = int(split_line[6])
            self.wait_missions.append(Mission(mission_id, arr_t, start_1, start_2, end_1,
                                              end_2, mission_type, self.init_map))
            count += 1
            if count >= self.mission_num:
                break
        return

    def init_agv(self):
        """
        初始化agv，并设定所在位置
        :return:
        """
        # 设定无冲突位置
        # 候选位置
        path_file = 'mission_data/mission_1.txt'
        with open(path_file, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = line.split()
            agv_id = int(split_line[0])
            location = int(split_line[2])
            loc = self.init_map.edge_dic[location]
            self.idle_agvs.append(AGV(agv_id, loc))
            if len(self.idle_agvs) >= self.agv_num:
                break
        return

    def release_AGV(self, cur_mission):
        """
        释放在cur_mission前完成任务的AGV
        :param cur_mission:
        :return:
        """
        for item in self.serve_missions:
            if item.finish_time < cur_mission.arr_t:
                self.serve_missions.remove(item)
                self.finish_missions.append(item)
                self.serve_agvs.remove(item.agv)
                self.idle_agvs.append(item.agv)
                item.agv.cur_loc = item.end_arc
                self.cancel_win(item)
        return

    def schedule_mission_cycle(self):
        """
        考虑任务的延迟情况
        :return:
        """
        # 初始化AGV
        self.init_agv()
        # 任务到达
        self.mission_arrive()
        # 循环处理任务
        count = 0
        max_time = 0
        self.wait_missions.sort(key=lambda x: x.arr_t)
        while self.wait_missions:
            cur_mission = self.wait_missions[0]
            self.release_AGV(cur_mission)

            if len(self.idle_agvs) == 0:
                for item in self.wait_missions:
                    item.arr_t += Interval
            else:
                cur_agv = cur_mission.getAGV(self.idle_agvs.copy())
                # 规划路径
                cur_mission.scheduling_path()

                # 判断初始能否插入，注意before_id记录的是上一个arc是第几个arc
                before_id = self.insert_first(cur_mission)

                while before_id == -1:
                    cur_mission.arr_t += Interval
                    before_id = self.insert_first(cur_mission)
                # 计算时间窗
                self.insert_time_part(cur_mission, before_id)
                # 插计算好的时间窗到arc上
                self.insert_win(cur_mission)
                # self.window_check(cur_mission)
                # self.conflict_check(cur_mission)
                self.wait_missions.remove(cur_mission)
                self.serve_missions.append(cur_mission)
                if cur_agv in self.idle_agvs:
                    self.idle_agvs.remove(cur_agv)
                if cur_agv not in self.serve_agvs:
                    self.serve_agvs.append(cur_agv)
                count += 1
                max_time = max(max_time, cur_mission.finish_time / 100)
                print(count, max_time)

        return count

    def time_merge(self, cur_path, agv_status, cur_mission):
        """
        时间窗合并函数，根据不同状态合并不同的时间窗
        :param cur_path:
        :param agv_status:
        :return:
        """
        """
        if cur_mission.mission_id == 51 and cur_path.id == 4262:
            print(cur_path)
        """
        # 用于记录总时间窗
        temp_win = []

        # 先添加主时间窗
        for i in range(len(cur_path.main_in_time)):
            temp_win.append([cur_path.main_in_time[i], cur_path.main_out_time[i]])

        # 根据车辆当前的状态来合并时间窗
        if agv_status == 0:
            for item in cur_path.empty_win:
                temp_win.append([item[0], item[1]])
        else:
            for item in cur_path.load_win:
                temp_win.append([item[0], item[1]])

        sub_win = temp_win

        if len(sub_win) == 0:
            new_sub_win = []
        else:
            sub_win = sorted(sub_win, key=lambda x: x[0])
            new_sub_win = []

            for item in sub_win:
                # 将第一个数组元素添加到新的数组中
                if not new_sub_win:
                    new_sub_win.append(item)
                    continue

                # 如果有重合，就更新新的数组中最后一个元素的最大值
                if new_sub_win[-1][1] >= item[0]:
                    new_sub_win[-1][1] = max(new_sub_win[-1][1], item[1])
                else:
                    new_sub_win.append(item)

        total_win = new_sub_win

        # 排序
        if len(total_win) == 0:
            return [], []
        total_win = sorted(total_win, key=lambda x: x[0])

        # 分解
        total_in_time = []
        total_out_time = []

        for item in total_win:
            total_in_time.append(item[0])
            total_out_time.append(item[1])
        return total_in_time, total_out_time

    def insert_first(self, cur_mission):
        """
        初始插入函数
        :param cur_mission:
        :return:
        """
        # 当前arc为第一条arc
        cur_path = self.init_map.edge_dic[cur_mission.path[0]]
        # 每次插入时，都需要合并时间窗，怎么合并要根据AGV当前状态
        cur_path.total_in_time, cur_path.total_out_time = self.time_merge(cur_path, cur_mission.arc_status[0], cur_mission)
        total_in_time = cur_path.total_in_time
        total_out_time = cur_path.total_out_time

        # 开始尝试插时间窗
        if len(total_in_time) == 0:
            # 计算时间窗
            time_win = [cur_mission.arr_t, cur_mission.arr_t + cur_path.cost]
            # 记录时间窗
            cur_mission.arc_win[0] = time_win.copy()
            cur_mission.arc_win_id[0] = 0
            # 可以插
            return 0
        else:
            # 有时间窗，需要计算插入位置
            cur_idx = np.searchsorted(total_in_time, cur_mission.arr_t + cur_path.cost)
            if cur_idx == 0:
                time_win = [cur_mission.arr_t, cur_mission.arr_t + cur_path.cost]
                cur_mission.arc_win[0] = time_win.copy()
                cur_mission.arc_win_id[0] = 0
                return 0

            else:
                for k in range(cur_idx, len(total_in_time)):
                    if total_in_time[k] - max(total_out_time[k - 1],
                                              cur_mission.arr_t) > cur_path.cost + 2 * self.safe_time:
                        max_in_time = max(total_out_time[k - 1] + self.safe_time, cur_mission.arr_t)
                        time_win = [max_in_time, max_in_time + cur_path.cost]
                        # 要满足出发时间
                        if time_win[0] == cur_mission.arr_t:
                            # 记录时间窗
                            cur_mission.arc_win[0] = time_win.copy()
                            cur_mission.arc_win_id[0] = k
                            return 0
                        else:
                            return -1

                if total_out_time[-1] + self.safe_time <= cur_mission.arr_t:
                    time_win = [cur_mission.arr_t, cur_mission.arr_t + cur_path.cost]
                    # 记录时间窗
                    cur_mission.arc_win[0] = time_win.copy()
                    cur_mission.arc_win_id[0] = len(total_in_time)
                    return 0
        return -1

    def insert_time_part(self, cur_mission, before_id):
        """
        时间窗插入函数，要根据车辆当前状态判断冲突关系
        :param cur_mission:
        :param before_id:
        :return:
        """
        cur_id = before_id + 1
        for i in range(cur_id, len(cur_mission.path)):
            # 获取arc对象
            cur_path = self.init_map.edge_dic[cur_mission.path[i]]
            before_path = self.init_map.edge_dic[cur_mission.path[i - 1]]

            # 获取时间向量
            cur_path.total_in_time, cur_path.total_out_time = self.time_merge(cur_path, cur_mission.arc_status[i], cur_mission)
            before_path.total_in_time, before_path.total_out_time = self.time_merge(before_path, cur_mission.arc_status[i-1], cur_mission)

            total_in_time = cur_path.total_in_time
            total_out_time = cur_path.total_out_time

            # 若无时间窗，直接插
            if len(total_in_time) == 0:
                # 获取上一arc的时间窗
                # before_win = cur_mission.arc_win[before_id]
                before_win = cur_mission.arc_win[i - 1]
                # 计算这一arc的时间窗
                time_win = [before_win[1] + self.safe_time, before_win[1] + self.safe_time + cur_path.cost]
                # 判断是否能够延长
                insert_id = cur_mission.arc_win_id[i - 1]
                # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[i - 1][0])
                if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                    # 记录时间窗
                    cur_mission.arc_win[i] = time_win.copy()
                    # 延伸时间窗
                    cur_mission.arc_win[i - 1][1] = time_win[0]
                    # 更新插入点
                    cur_mission.arc_win_id[i] = 0
                    # 更新before_id
                    # before_id = i
                    continue
                else:
                    return self.reinsert_time_win(cur_mission, i - 1)

            else:
                # 有时间窗，需要判断插入位置
                # before_win = cur_mission.arc_win[before_id]
                before_win = cur_mission.arc_win[i - 1]
                cur_idx = np.searchsorted(total_in_time, before_win[1] + self.safe_time + cur_path.cost)

                if cur_idx == 0:  # 插在最前面
                    time_win = [before_win[1] + self.safe_time, before_win[1] + self.safe_time + cur_path.cost]
                    # 判断是否可以延长
                    insert_id = cur_mission.arc_win_id[i - 1]
                    # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[i - 1][0])
                    if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                        # 记录时间窗
                        cur_mission.arc_win[i] = time_win.copy()
                        # 延伸时间窗
                        cur_mission.arc_win[i - 1][1] = time_win[0]
                        # 更新插入点
                        cur_mission.arc_win_id[i] = 0
                        # 更新arc
                        # before_id = i
                        continue
                    else:
                        return self.reinsert_time_win(cur_mission, i - 1)

                else:  # 插在中间/后面
                    flag = 0
                    for k in range(cur_idx, len(total_in_time)):
                        if total_in_time[k] - max(total_out_time[k - 1],
                                                  before_win[1] + self.safe_time) > cur_path.cost + 2 * self.safe_time:
                            max_in_time = max(total_out_time[k - 1] + self.safe_time, before_win[1] + self.safe_time)
                            time_win = [max_in_time, max_in_time + cur_path.cost]

                            # 判断是否可以延长
                            insert_id = cur_mission.arc_win_id[i - 1]
                            # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[i - 1][0])
                            if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                                # 记录时间窗
                                cur_mission.arc_win[i] = time_win.copy()
                                # 延伸时间窗
                                cur_mission.arc_win[i - 1][1] = time_win[0]
                                # 更新插入点
                                cur_mission.arc_win_id[i] = k
                                # cur_mission.arc_win_id[i] = np.searchsorted(total_in_time, time_win[0])
                                # 更新arc
                                # before_id = i
                                flag = 1
                                break
                            else:
                                return self.reinsert_time_win(cur_mission, i - 1)

                    if flag == 0:
                        max_in_time = max(before_win[1] + self.safe_time, total_out_time[-1] + self.safe_time)
                        time_win = [max_in_time, max_in_time + cur_path.cost]
                        # 判断是否可以延长
                        insert_id = cur_mission.arc_win_id[i - 1]
                        # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[i - 1][0])
                        if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                            # 记录时间窗
                            cur_mission.arc_win[i] = time_win.copy()
                            # 延伸时间窗
                            cur_mission.arc_win[i - 1][1] = time_win[0]
                            # 更新插入点
                            cur_mission.arc_win_id[i] = len(cur_path.total_in_time)
                            # cur_mission.arc_win_id[i] = np.searchsorted(total_in_time, time_win[0])
                            # 更新arc
                            # before_id = i
                            continue
                        else:
                            return self.reinsert_time_win(cur_mission, i - 1)
        return

    def reinsert_time_win(self, cur_mission, cur_id):
        """
        从cur_id开始重插
        :param cur_mission:
        :param cur_id:
        :return:
        """
        # 特殊判断，看是不是第一条边
        if cur_id == 0:
            # 说明要插的是第一条边
            del cur_mission.arc_win[0]
            cur_mission.arr_t += Interval
            before_id = self.insert_first(cur_mission)
            while before_id == -1:
                cur_mission.arr_t += Interval
                before_id = self.insert_first(cur_mission)
            return self.insert_time_part(cur_mission, before_id)

        # 否则，就从这条边开始插
        cur_path = self.init_map.edge_dic[cur_mission.path[cur_id]]
        before_path = self.init_map.edge_dic[cur_mission.path[cur_id - 1]]
        cur_path.total_in_time, cur_path.total_out_time = self.time_merge(cur_path, cur_mission.arc_status[cur_id], cur_mission)
        before_path.total_in_time, before_path.total_out_time = self.time_merge(before_path, cur_mission.arc_status[cur_id - 1], cur_mission)

        # 计算时间窗
        before_win = cur_mission.arc_win[cur_id - 1]
        cur_win = cur_mission.arc_win[cur_id]
        conflict_id = cur_mission.arc_win_id[cur_id]
        # conflict_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[cur_id][0])
        # 删除时间窗
        del cur_mission.arc_win[cur_id]
        del cur_mission.arc_win_id[cur_id]

        flag = 0
        for k in range(conflict_id + 1, len(cur_path.total_in_time)):
            if cur_path.total_in_time[k] - max(cur_path.total_out_time[k - 1],
                                               before_win[1] + self.safe_time) > cur_path.cost + 2 * self.safe_time:
                max_in_time = max(cur_path.total_out_time[k - 1] + self.safe_time, before_win[1] + self.safe_time)
                time_win = [max_in_time, max_in_time + cur_path.cost]

                # 判断是否可以延长
                insert_id = cur_mission.arc_win_id[cur_id - 1]
                # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[cur_id - 1][0])
                if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                    # 记录时间窗
                    cur_mission.arc_win[cur_id] = time_win.copy()
                    # 延伸时间窗
                    cur_mission.arc_win[cur_id-1][1] = time_win[0]
                    # 更新插入点
                    cur_mission.arc_win_id[cur_id] = k
                    # cur_mission.arc_win_id[cur_id] = np.searchsorted(cur_path.total_in_time, time_win[0])
                    flag = 1
                    break
                else:
                    return self.reinsert_time_win(cur_mission, cur_id - 1)

        if flag == 0:
            time_win = [cur_path.total_out_time[-1] + self.safe_time, cur_path.total_out_time[-1] + self.safe_time + cur_path.cost]
            # 判断是否可以延长
            insert_id = cur_mission.arc_win_id[cur_id - 1]
            # insert_id = np.searchsorted(before_path.total_in_time, cur_mission.arc_win[cur_id - 1][0])
            if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                # 记录时间窗
                cur_mission.arc_win[cur_id] = time_win.copy()
                # 延伸时间窗
                cur_mission.arc_win[cur_id - 1][1] = time_win[0]
                # 更新插入点
                cur_mission.arc_win_id[cur_id] = len(cur_path.total_in_time)
                # cur_mission.arc_win_id[cur_id] = np.searchsorted(cur_path.total_in_time, time_win[0])
            else:
                return self.reinsert_time_win(cur_mission, cur_id - 1)

        return self.insert_time_part(cur_mission, cur_id)

    def insert_win(self, cur_mission):
        """
        单次插入时间窗的函数，将记录好的arc_win插到各个arc的时间向量上去
        :param cur_mission:
        :return:
        """
        empty_time = 0
        load_time = 0
        # 遍历每个arc
        for i in range(0, len(cur_mission.path)):
            # 获取arc对象
            cur_path = self.init_map.edge_dic[cur_mission.path[i]]
            # 获取时间窗
            cur_win = cur_mission.arc_win[i]
            # 获取车辆状态
            cur_status = cur_mission.arc_status[i]

            if i == len(cur_mission.path) - 1:
                cur_mission.finish_time = cur_win[1]
                cost = sum([self.init_map.edge_dic[item].cost for item in cur_mission.path])
                # print(cur_mission.mission_id, cur_mission.arr_t / 100, cur_mission.arr_t / 100 + cost / 100, cur_win[1] / 100, cur_mission.agv)
                self.total_conflict_time += cur_win[1] / 100 - (cur_mission.arr_t / 100 + cost / 100)
                self.total_time += cur_win[1] / 100 - cur_mission.arr_t / 100
                cur_mission.agv.cur_loc = cur_path

            # 插主时间窗
            main_idx = np.searchsorted(cur_path.main_in_time, cur_win[0])
            cur_path.main_in_time.insert(main_idx, cur_win[0])
            cur_path.main_out_time.insert(main_idx, cur_win[1])
            cur_path.main_idx_mission.insert(main_idx, cur_mission)

            # 处理冲突关系
            if cur_status == 0:  # 当前车辆状态为空载
                # 先处理空载-空载的情况
                for item in cur_path.empty_empty_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.empty_win.append([cur_win[0], cur_win[1]])
                        item_path.empty_mission.append(cur_mission)

                # 处理空载-负载的情况
                for item in cur_path.empty_load_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.load_win.append([cur_win[0], cur_win[1]])
                        item_path.load_mission.append(cur_mission)

            else:  # 当前车辆状态为负载
                # 处理负载-空载的情况
                for item in cur_path.load_empty_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.empty_win.append([cur_win[0], cur_win[1]])
                        item_path.empty_mission.append(cur_mission)

                # 处理负载-负载的情况
                for item in cur_path.load_load_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.load_win.append([cur_win[0], cur_win[1]])
                        item_path.load_mission.append(cur_mission)
        return empty_time / 100, load_time / 100

    def cancel_win(self, cur_mission):
        """
        取消对应时间窗
        :param cur_mission:
        :return:
        """
        # 遍历每个arc
        for i, item in enumerate(cur_mission.path):
            # 获取arc对象
            item_path = self.init_map.edge_dic[item]
            win = cur_mission.arc_win[i]

            # 删除主时间窗
            idx = item_path.main_in_time.index(win[0])
            del item_path.main_in_time[idx]
            del item_path.main_out_time[idx]
            del item_path.main_idx_mission[idx]
            """
            idx_set = [i for i, x in enumerate(item_path.main_idx_mission) if x == cur_mission]
            idx_set.reverse()
            for idx in idx_set:
                del item_path.main_in_time[idx]
                del item_path.main_out_time[idx]
                del item_path.main_idx_mission[idx]
            """

            # 处理相关辅助时间窗
            # 先获取车辆状态
            agv_status = cur_mission.arc_status[i]
            if agv_status == 0:  # 处理空载范围的时间窗
                # 处理空载-空载的情况
                for sub in item_path.empty_empty_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    """
                    idx_set = [i for i, x in enumerate(sub_path.empty_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]
                    """
                    if sub != item_path.id:
                        idx = sub_path.empty_win.index(win)
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]

                # 处理空载-负载的情况
                for sub in item_path.empty_load_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    """
                    idx_set = [i for i, x in enumerate(sub_path.load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]
                    """
                    if sub != item_path.id:
                        idx = sub_path.load_win.index(win)
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]

            else:  # 处理负载范围的时间窗
                # 处理负载-空载的情况
                for sub in item_path.load_empty_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    """
                    idx_set = [i for i, x in enumerate(sub_path.empty_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]
                    """
                    if sub != item_path.id:
                        idx = sub_path.empty_win.index(win)
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]
                # 处理负载-负载的情况
                for sub in item_path.load_load_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    """
                    idx_set = [i for i, x in enumerate(sub_path.load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]
                    """
                    if sub != item_path.id:
                        idx = sub_path.load_win.index(win)
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]
        return

    def window_check(self, cur_mission):
        """
        检查时间窗是否存在冲突
        :param cur_mission:
        :return:
        """
        for cur_id, item in enumerate(cur_mission.path):
            # 获取arc对象
            item_path = self.init_map.edge_dic[item]
            item_status = cur_mission.arc_status[cur_id]
            # item_win = cur_mission.arc_win[cur_id]

            # 检查总时间窗
            for i in range(1, len(item_path.main_in_time)):
                if item_path.main_in_time[i - 1] <= item_path.main_out_time[i - 1] <= item_path.main_in_time[i] <= \
                        item_path.main_out_time[i]:
                    continue
                else:
                    print('总时间窗出错了')

            # 检查是否与冲突边存在问题
            if item_status == 0:  # 车辆处于空载状态
                for cur in item_path.empty_empty_clashed_edge_set:
                    cur_path = self.init_map.edge_dic[cur]
                    for k in range(1, len(cur_path.main_in_time)):
                        if cur_path.main_in_time[k - 1] <= cur_path.main_out_time[k - 1] <= cur_path.main_in_time[k] <= \
                                cur_path.main_out_time[k]:
                            continue
                        else:
                            print('空载-空载出错了')

                for cur in item_path.empty_load_clashed_edge_set:
                    cur_path = self.init_map.edge_dic[cur]
                    for k in range(1, len(cur_path.main_in_time)):
                        if cur_path.main_in_time[k - 1] <= cur_path.main_out_time[k - 1] <= cur_path.main_in_time[k] <= \
                                cur_path.main_out_time[k]:
                            continue
                        else:
                            print('空载-负载出错了')
            else:
                # 车辆处于负载状态
                for cur in item_path.load_load_clashed_edge_set:
                    cur_path = self.init_map.edge_dic[cur]

                    for k in range(1, len(cur_path.main_in_time)):
                        if cur_path.main_in_time[k - 1] <= cur_path.main_out_time[k - 1] <= cur_path.main_in_time[k] <= \
                                cur_path.main_out_time[k]:
                            continue
                        else:
                            print('负载-负载出错了')

                for cur in item_path.load_empty_clashed_edge_set:
                    cur_path = self.init_map.edge_dic[cur]
                    for k in range(1, len(cur_path.main_in_time)):
                        if cur_path.main_in_time[k - 1] <= cur_path.main_out_time[k - 1] <= cur_path.main_in_time[k] <= \
                                cur_path.main_out_time[k]:
                            continue
                        else:
                            print('负载-空载出错了')
        return

    def check_win(self, item_path, conflict_path, item_win, status):
        """
        检查item_win是否与conflict_path上的主时间窗冲突
        :param status:
        :param conflict_path:
        :param item_win:
        :return:
        """
        for i in range(0, len(conflict_path.main_in_time)):
            win = [conflict_path.main_in_time[i], conflict_path.main_out_time[i]]
            conflict_mission = conflict_path.main_idx_mission[i]
            if isinstance(conflict_mission, int):
                continue
            idx = [k for k, v in conflict_mission.arc_win.items() if v == win][0]
            conflict_status = conflict_mission.arc_status[idx]
            if item_win[0] < win[1] < item_win[1] and conflict_status == status:
                print(item_path.id, conflict_path.id, item_win, win)
                print(self.wait_missions[0].mission_id, self.wait_missions[0].start_arc.id, self.wait_missions[0].end_arc.id)
                # print(conflict_mission.arc_win)
            elif item_win[0] < win[0] < item_win[1] and conflict_status == status:
                print(conflict_path.id, item_win, win)
            else:
                continue
        return

    def conflict_check(self, cur_mission):
        """
        逐个判断是否存在冲突
        :param cur_mission:
        :return:
        """
        for item_id, item in enumerate(cur_mission.path):
            # 获取arc对象
            item_path = self.init_map.edge_dic[item]
            item_win = cur_mission.arc_win[item_id]
            item_status = cur_mission.arc_status[item_id]

            if item_status == 0:
                # 说明车辆当前处于空载状态，需要判断其是否跟冲突边上时间窗冲突
                for cur in item_path.empty_empty_clashed_edge_set:
                    if cur != item:
                        cur_path = self.init_map.edge_dic[cur]
                        self.check_win(item_path, cur_path, item_win, 0)

                for cur in item_path.empty_load_clashed_edge_set:
                    if cur != item:
                        cur_path = self.init_map.edge_dic[cur]
                        self.check_win(item_path, cur_path, item_win, 1)
            else:
                # 说明车辆当前处于负载状态，需要判断其是否跟冲突边上时间窗冲突
                for cur in item_path.load_empty_clashed_edge_set:
                    if cur != item:
                        cur_path = self.init_map.edge_dic[cur]
                        self.check_win(item_path, cur_path, item_win, 0)

                for cur in item_path.load_load_clashed_edge_set:
                    if cur != item:
                        cur_path = self.init_map.edge_dic[cur]
                        self.check_win(item_path, cur_path, item_win, 1)
        return


if __name__ == '__main__':
    nums = {10: 30, 15: 45, 20: 60, 25: 75, 30: 90}
    for i in range(10, 31, 5):
        Interval = 500
        test = MainAlgo()
        test.mission_num = nums[i]
        test.agv_num = i
        start = time.time()
        count = test.schedule_mission_cycle()
        print(test.total_conflict_time / test.total_time)
        print((time.time() - start) / nums[i])
