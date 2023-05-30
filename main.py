"""
多负载场景下的AGV无冲突路径规划
Date: 2023-05-28
要注意一个问题，每个点有固定的访问时间
如果是pick up类型的访问点，需要判断一下当前的时间是否 >= 要求访问时间，如果不是则需要临时额外增加该边的cost，让车辆在该边等待
"""
import copy
from Astar import PriorityQueue
from getMap import Map
import sys
import numpy as np

sys.setrecursionlimit(30000000)
INF = 100000000
back_pos_5 = [7148, 7079, 6742, 4071, 6558]
back_pos_10 = [7293, 4135, 7304, 4121, 4109,
               4073, 6558, 6560, 6554, 6556]
back_pos_15 = [7148, 7147, 7079, 6742, 4071,
               4073, 6558, 6560, 6554, 6556,
               7293, 4135, 7304, 4121, 4109]
back_pos_20 = [7148, 7147, 7079, 6742, 4071,
               4073, 6558, 6560, 6554, 6556,
               7293, 4135, 7304, 4121, 4109,
               4115, 4127, 4111, 4113, 4117]
back_pos_25 = [7148, 7147, 7079, 6742, 4071,
               4073, 6558, 6560, 6554, 6556,
               7293, 4135, 7304, 4121, 4109,
               4115, 4127, 4111, 4113, 4117,
               4119, 4131, 4123, 4125, 4129]
back_pos_30 = [7148, 7147, 7079, 6742, 4071,
               4073, 6558, 6560, 6554, 6556,
               7293, 4135, 7304, 4121, 4109,
               4115, 4127, 4111, 4113, 4117,
               4119, 4131, 4123, 4125, 4129,
               3805, 3807, 3809, 6994, 7067]
back_pos = back_pos_30
total_opp = {7148: 7146, 7147: 7145, 7079: 7078, 6742: 6738, 4071: 4070, 4073: 4072, 6558: 6557, 6560: 6559, 6554: 6553, 6556: 6555,
             7293: 7373, 4135: 4134, 7304: 7303, 4121: 4120, 4109: 4108, 4115: 4114, 4127: 4126, 4111: 4110, 4113: 4112,
             4117: 4116, 4119: 4118, 4123: 4122, 4125: 4124, 4129: 4128, 4131: 4130, 3805: 3804, 3807: 3806, 3809: 3807,
             7067: 7066, 6994: 6993}


class Mission:
    def __init__(self, idx, loc_list, time_list, loc_type, load, init_map, agv):
        self.map = init_map
        self.mission_id = idx
        self.arr_t = time_list[0]  # 任务最开始的时间
        self.loc_list = loc_list  # 途经点位置
        self.loc_type = loc_type  # 途径点类型
        self.loc_load = load  # 在途径点需要增加还是减少的货物
        self.time_list = time_list  # 每个途径点要求
        self.load_set = [1]  # 随时变更的load集合
        self.path = []  # 路径
        self.agv = agv
        self.finish_time = -1
        self.arc_win = {}  # 存储path上第几个arc对应的时间窗
        self.arc_win_id = {}  # 存储path上arc时间窗的插入点
        self.arc_status = []  # 存储path上第几个arc的负载状态，预先计算好
        # 有三种负载状态：1 空载, 2 小负载, 3 大负载

    @staticmethod
    def distance(start_node, end_node):
        return abs(start_node.x_cor - end_node.x_cor) + abs(start_node.y_cor - end_node.y_cor)

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
                    priority = new_cost + self.heuristic(current, next)
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
        对多负载任务进行路径规划
        :return:
        """
        start = self.agv.cur_loc.id
        for idx, end in enumerate(self.loc_list):
            came_from, _ = self.aStar_search(start, end, [])
            path = self.reconstruct_path(came_from, start, end)
            if not path:
                return []
            # 添加路径1
            path.remove(start)
            # 计算货物变化
            if idx > 0:
                cur_load = self.loc_load[idx - 1]
                if cur_load < 0:
                    self.load_set.remove(abs(cur_load))
                else:
                    self.load_set.append(cur_load)
            # 计算负载状态变化
            max_load = max(self.load_set)
            self.arc_status += [max_load] * len(path)
            # 更新path集合
            self.path += copy.deepcopy(path)

            # 更新start和end
            start = end
        return

    def __repr__(self):
        return "mission_" + str(self.mission_id)


class AGV:
    def __init__(self, idx, loc, status):
        self.mission = None
        self.agv_id = idx
        self.cur_loc = loc
        self.back_loc = loc
        self.cur_status = status

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
        self.agv_set = []
        self.safe_time = 1
        self.mission_num = 0
        self.agv_num = 0
        self.back_location = list()
        self.idle_location = list()

    def init_agv(self):
        """
        初始化agv，并设定所在位置
        :return:
        """
        # 设定无冲突位置
        # 候选位置
        self.idle_location = copy.deepcopy(back_pos)
        total = []

        for i in range(len(self.idle_location)):
            location = self.idle_location[i]
            total.append(location)
            total.append(total_opp[location])

        for i in range(5):
            agv_id = i
            location = self.back_location[i]
            loc = self.init_map.edge_dic[location]
            self.agv_set.append(AGV(agv_id, loc, 0))
            loc.main_in_time.append(0)
            loc.main_out_time.append(INF)
            loc.main_time_win.append(INF)
            loc.main_idx_mission.append(0)
            loc.main_status.append(-1)
            # 移除
            self.idle_location.remove(location)

        for item in self.init_map.edge_dic.keys():
            if item in total:
                self.init_map.edge_dic[item].empty_empty_clashed_edge_set = [item]
                self.init_map.edge_dic[item].empty_load_clashed_edge_set = [item]
                self.init_map.edge_dic[item].load_empty_clashed_edge_set = [item]
                self.init_map.edge_dic[item].load_load_clashed_edge_set = [item]
            else:
                a = self.init_map.edge_dic[item].empty_empty_clashed_edge_set
                b = total
                self.init_map.edge_dic[item].empty_empty_clashed_edge_set = list(set(a).difference(set(b)))

                a = self.init_map.edge_dic[item].empty_load_clashed_edge_set
                b = total
                self.init_map.edge_dic[item].empty_load_clashed_edge_set = list(set(a).difference(set(b)))

                a = self.init_map.edge_dic[item].load_empty_clashed_edge_set
                b = total
                self.init_map.edge_dic[item].load_empty_clashed_edge_set = list(set(a).difference(set(b)))

                a = self.init_map.edge_dic[item].load_load_clashed_edge_set
                b = total
                self.init_map.edge_dic[item].load_load_clashed_edge_set = list(set(a).difference(set(b)))
        return

    def mission_arrive(self):
        """
        获取任务
        :return:
        """
        path_file = 'data/mission_split.txt'
        with open(path_file, encoding='utf-8') as file:
            content = file.readlines()
        count = 0
        while count < len(content) - 1:
            line = content[count]
            split_line = line.split()
            if count == 0:
                self.mission_num = int(split_line[1])
                self.agv_num = int(split_line[0])
                count += 1
                continue
            if split_line[0] == 'Mission':
                agv_id = int(split_line[3])
                mission_id = int(split_line[1])
                loc_num = int(split_line[2])
                count += 1
                loc_type = []
                loc_list = []
                loc_load = []
                loc_time = []
                for i in range(loc_num):
                    line = content[count]
                    split_line = line.split()
                    loc_type.append(split_line[1])
                    loc_time.append(float(split_line[2]) * 100)
                    loc_list.append(int(split_line[3]))
                    if split_line[1] == 'pick':
                        loc_load.append(int(split_line[4]))
                    else:
                        load = int(split_line[4])
                        loc_load.append(-load)
                    count += 1
                self.wait_missions.append(Mission(mission_id, loc_list, loc_time, loc_type, loc_load, self.init_map, self.agv_set[agv_id]))
        return

    def scheduling_mission(self):
        """
        规划任务主函数
        :return:
        """
        # 初始化AGV
        self.init_agv()
        # 任务到达
        self.mission_arrive()
        # 处理任务
        self.wait_missions.sort(key=lambda x: x.arr_t)
        # 循环处理
        while self.wait_missions:
            # 先排序
            self.wait_missions.sort(key=lambda x: x.arr_t)
            # 取出一个任务
            cur_mission = self.wait_missions[0]
            # 释放AGV
            self.release_AGV(cur_mission)
            if cur_mission.agv.cur_status == 1:
                cur_mission.arr_t += Interval
                continue
            else:
                cur_mission.agv.mission = cur_mission
                cur_mission.agv.cur_status = 1
                cur_mission.scheduling_path()

                # 判断初始能否插入，注意before_id记录的是上一个arc是第几个arc
                before_id = self.insert_first(cur_mission)

                # 插入不了也需要一直推迟
                while before_id == -1:
                    for item in self.wait_missions:
                        item.arr_t += Interval
                    before_id = self.insert_first(cur_mission)

                # 计算时间窗
                self.insert_time_part(cur_mission, before_id)
                # 插计算好的时间窗到arc上
                self.insert_win(cur_mission)

                # self.window_check(cur_mission)
                # self.conflict_check(cur_mission)

                # 处理任务和车
                self.wait_missions.remove(cur_mission)
                self.serve_missions.append(cur_mission)
                print(cur_mission.mission_id, cur_mission.arr_t / 100, cur_mission.finish_time / 100)
        print('所有任务规划完成')
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
                item.agv.cur_status = 0
                item.mission = None
                self.cancel_win(item)
        return

    def insert_first(self, cur_mission):
        """
        初始插入函数
        :param cur_mission:
        :return:
        """
        # 当前arc为第一条arc
        cur_path = self.init_map.edge_dic[cur_mission.path[0]]
        cur_path.total_in_time, cur_path.total_out_time = self.time_merge(cur_path, cur_mission.arc_status[0])
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
        时间窗计算函数
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
            cur_path.total_in_time, cur_path.total_out_time = self.time_merge(cur_path,cur_mission.arc_status[i])
            total_in_time = cur_path.total_in_time
            total_out_time = cur_path.total_out_time

            # 若无时间窗，直接插
            if len(total_in_time) == 0:
                # 获取上一arc的时间窗
                before_win = cur_mission.arc_win[before_id]
                # 计算这一arc的时间窗
                time_win = [before_win[1] + self.safe_time, before_win[1] + self.safe_time + cur_path.cost]
                # 判断是否能够延长
                insert_id = cur_mission.arc_win_id[i - 1]
                if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                    # 记录时间窗
                    cur_mission.arc_win[i] = time_win.copy()
                    # 延伸时间窗
                    cur_mission.arc_win[before_id][1] = time_win[0]
                    # 更新插入点
                    cur_mission.arc_win_id[i] = 0
                    # 更新before_id
                    before_id = i
                    continue
                else:
                    return self.reinsert_time_win(cur_mission, before_id)

            else:
                # 有时间窗，需要判断插入位置
                before_win = cur_mission.arc_win[before_id]
                cur_idx = np.searchsorted(total_in_time, before_win[1] + self.safe_time + cur_path.cost)

                if cur_idx == 0:  # 插在最前面
                    time_win = [before_win[1] + self.safe_time, before_win[1] + self.safe_time + cur_path.cost]
                    # 判断是否可以延长
                    insert_id = cur_mission.arc_win_id[i - 1]
                    if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[
                        insert_id]:
                        # 记录时间窗
                        cur_mission.arc_win[i] = time_win.copy()
                        # 延伸时间窗
                        cur_mission.arc_win[before_id][1] = time_win[0]
                        # 更新插入点
                        cur_mission.arc_win_id[i] = 0
                        # 更新arc
                        before_id = i
                        continue
                    else:
                        return self.reinsert_time_win(cur_mission, before_id)

                else:  # 插在中间/后面
                    flag = 0
                    for k in range(cur_idx, len(total_in_time)):
                        if total_in_time[k] - max(total_out_time[k - 1],
                                                  before_win[1] + self.safe_time) > cur_path.cost + 2 * self.safe_time:
                            max_in_time = max(total_out_time[k - 1] + self.safe_time, before_win[1] + self.safe_time)
                            time_win = [max_in_time, max_in_time + cur_path.cost]

                            # 判断是否可以延长
                            insert_id = cur_mission.arc_win_id[i - 1]
                            if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[
                                insert_id]:
                                # 记录时间窗
                                cur_mission.arc_win[i] = time_win.copy()
                                # 延伸时间窗
                                cur_mission.arc_win[before_id][1] = time_win[0]
                                # 更新插入点
                                cur_mission.arc_win_id[i] = k
                                # 更新arc
                                before_id = i
                                flag = 1
                                break
                            else:
                                return self.reinsert_time_win(cur_mission, before_id)

                    if flag == 0:
                        max_in_time = max(before_win[1] + self.safe_time, total_out_time[-1] + self.safe_time)
                        time_win = [max_in_time, max_in_time + cur_path.cost]
                        # 判断是否可以延长
                        insert_id = cur_mission.arc_win_id[i - 1]
                        if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[
                            insert_id]:
                            # 记录时间窗
                            cur_mission.arc_win[i] = time_win.copy()
                            # 延伸时间窗
                            cur_mission.arc_win[before_id][1] = time_win[0]
                            # 更新插入点
                            cur_mission.arc_win_id[i] = len(cur_path.total_in_time)
                            # 更新arc
                            before_id = i
                            continue
                        else:
                            return self.reinsert_time_win(cur_mission, before_id)

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
        cur_path.total_in_time, cur_path.total_out_time= self.time_merge(cur_path,cur_mission.arc_status[cur_id])
        # before_path.total_in_time, before_path.total_out_time, before_path.total_mission = self.time_merge(before_path, cur_mission.arc_status[cur_id - 1])

        # 计算时间窗
        before_win = cur_mission.arc_win[cur_id - 1]
        conflict_id = cur_mission.arc_win_id[cur_id]

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
                if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                    # 记录时间窗
                    cur_mission.arc_win[cur_id] = time_win.copy()
                    # 延伸时间窗
                    cur_mission.arc_win[cur_id - 1][1] = time_win[0]
                    # 更新插入点
                    cur_mission.arc_win_id[cur_id] = k
                    flag = 1
                    break
                else:
                    return self.reinsert_time_win(cur_mission, cur_id - 1)

        if flag == 0:
            time_win = [cur_path.total_out_time[-1] + self.safe_time,
                        cur_path.total_out_time[-1] + self.safe_time + cur_path.cost]
            # 判断是否可以延长
            insert_id = cur_mission.arc_win_id[cur_id - 1]
            if insert_id == len(before_path.total_in_time) or time_win[0] <= before_path.total_in_time[insert_id]:
                # 记录时间窗
                cur_mission.arc_win[cur_id] = time_win.copy()
                # 延伸时间窗
                cur_mission.arc_win[cur_id - 1][1] = time_win[0]
                # 更新插入点
                cur_mission.arc_win_id[cur_id] = len(cur_path.total_in_time)
            else:
                return self.reinsert_time_win(cur_mission, cur_id - 1)

        return self.insert_time_part(cur_mission, cur_id)

    def time_merge(self, cur_path, agv_status):
        # 用于记录总时间窗
        temp_win = []

        # 先添加主时间窗
        for i in range(len(cur_path.main_in_time)):
            temp_win.append([cur_path.main_in_time[i], cur_path.main_out_time[i]])

        # 根据车辆当前的状态来合并时间窗
        if agv_status == 1:
            sub_win = copy.deepcopy(cur_path.empty_win)
        elif agv_status == 2:
            sub_win = copy.deepcopy(cur_path.load_win)
        else:
            sub_win = copy.deepcopy(cur_path.multi_load_win)

        sub_win = sub_win + temp_win

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

        total_win = copy.deepcopy(new_sub_win)

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

    def insert_win(self, cur_mission):
        """
        将记录好的arc_win插到各个arc的时间向量上去
        :param cur_mission:
        :return:
        """
        for i in range(0, len(cur_mission.path)):
            # 获取arc对象
            cur_path = self.init_map.edge_dic[cur_mission.path[i]]
            # 获取时间窗
            cur_win = cur_mission.arc_win[i]
            # 获取车辆状态
            cur_status = cur_mission.arc_status[i]

            if i == len(cur_mission.path) - 1:
                cur_mission.finish_time = cur_win[1]
                # print(cur_mission.mission_id, cur_mission.arr_t / 100, cur_win[1] / 100, cur_mission.agv)
                cur_win[1] = INF
                cur_mission.agv.cur_loc = cur_path

            # 插主时间窗
            main_idx = np.searchsorted(cur_path.main_in_time, cur_win[0])
            cur_path.main_in_time.insert(main_idx, cur_win[0])
            cur_path.main_out_time.insert(main_idx, cur_win[1])
            cur_path.main_time_win.insert(main_idx, cur_win[1] - cur_win[0])
            cur_path.main_idx_mission.insert(main_idx, cur_mission)
            cur_path.main_status.insert(main_idx, cur_status)

            # 处理冲突边
            if cur_status == 1:  # 当前车辆状态为空载
                # 先处理空载-空载的情况
                for item in cur_path.empty_empty_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.empty_win.append(cur_win.copy())
                        item_path.empty_mission.append(cur_mission)

                # 处理空载-负载的情况
                for item in cur_path.empty_load_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.load_win.append(cur_win.copy())
                        item_path.load_mission.append(cur_mission)

                # 处理空载-超载的情况
                for item in cur_path.empty_multi_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.multi_load_win.append(cur_win.copy())
                        item_path.multi_load_mission.append(cur_mission)

            elif cur_status == 2:
                # 处理负载-空载的情况
                for item in cur_path.load_empty_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.empty_win.append(cur_win.copy())
                        item_path.empty_mission.append(cur_mission)

                # 处理负载-负载的情况
                for item in cur_path.load_load_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.load_win.append(cur_win.copy())
                        item_path.load_mission.append(cur_mission)

                # 处理负载-超载的情况
                for item in cur_path.load_multi_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.multi_load_win.append(cur_win.copy())
                        item_path.multi_load_mission.append(cur_mission)

            else:
                # 处理超载-空载的情况
                for item in cur_path.multi_empty_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.empty_win.append(cur_win.copy())
                        item_path.empty_mission.append(cur_mission)

                # 处理超载-负载的情况
                for item in cur_path.multi_load_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.load_win.append(cur_win.copy())
                        item_path.load_mission.append(cur_mission)

                # 处理负载-超载的情况
                for item in cur_path.multi_multi_clashed_edge_set:
                    if item != cur_path.id:
                        item_path = self.init_map.edge_dic[item]
                        item_path.multi_load_win.append(cur_win.copy())
                        item_path.multi_load_mission.append(cur_mission)
        return

    def cancel_win(self, cur_mission):
        """
        取消对应时间窗
        :param cur_mission:
        :return:
        """
        # 遍历每个arc
        for i in range(0, len(cur_mission.path)):
            # 获取arc对象
            item_path = self.init_map.edge_dic[cur_mission.path[i]]

            # 删除主时间窗
            idx_set = [i for i, x in enumerate(item_path.main_idx_mission) if x == cur_mission]
            idx_set.reverse()
            for idx in idx_set:
                del item_path.main_in_time[idx]
                del item_path.main_out_time[idx]
                del item_path.main_idx_mission[idx]
                del item_path.main_status[idx]

            # 处理相关辅助时间窗
            # 先获取车辆状态
            agv_status = cur_mission.arc_status[i]
            if agv_status == 1:  # 处理空载范围的时间窗
                # 处理空载-空载的情况
                for sub in item_path.empty_empty_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.empty_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]

                # 处理空载-负载的情况
                for sub in item_path.empty_load_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]

                # 处理空载-超载的情况
                for sub in item_path.empty_multi_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.multi_load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.multi_load_win[idx]
                        del sub_path.multi_load_mission[idx]

            elif agv_status == 2:  # 处理负载范围的时间窗
                # 处理负载-空载的情况
                for sub in item_path.load_empty_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.empty_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]
                # 处理负载-负载的情况
                for sub in item_path.load_load_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]

                # 处理空载-超载的情况
                for sub in item_path.load_multi_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.multi_load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.multi_load_win[idx]
                        del sub_path.multi_load_mission[idx]
            else:
                # 处理负载-空载的情况
                for sub in item_path.multi_empty_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.empty_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.empty_win[idx]
                        del sub_path.empty_mission[idx]
                # 处理负载-负载的情况
                for sub in item_path.multi_load_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.load_win[idx]
                        del sub_path.load_mission[idx]

                # 处理空载-超载的情况
                for sub in item_path.multi_multi_clashed_edge_set:
                    sub_path = self.init_map.edge_dic[sub]
                    idx_set = [i for i, x in enumerate(sub_path.multi_load_mission) if x == cur_mission]
                    idx_set.reverse()
                    for idx in idx_set:
                        del sub_path.multi_load_win[idx]
                        del sub_path.multi_load_mission[idx]
        return


if __name__ == '__main__':
    Interval = 500
    test = MainAlgo()
    test.back_location = back_pos_5
    test.scheduling_mission()






