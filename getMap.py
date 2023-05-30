import re
import cv2
import math
import pickle


class Node:  # edge的交叉点
    def __init__(self, x, y):
        self.x_cor = x
        self.y_cor = y
        # self.direction = direction  # +-180, 0, +-90
        self.start_arc = []  # 以该节点为起点的边
        self.end_arc = []  # 以该节点为终点的边
        # self.conflict_edge_set = []   # 冲突边集合
        # self.front_path = []   # 上游路径
        # self.forward_path = []  # 下游路径

    # 获取点的冲突边集合
    def getConflictEdgeSet(self):
        pass

    # 获取点的上游路径集合
    def getFrontEdgeSet(self):
        pass

    # 获取点的下游路径集合
    def getForwardEdgeSet(self):
        pass

    def __repr__(self):
        return '(' + str(self.x_cor) + ',' + str(self.y_cor) + ')'


class Edge:
    def __init__(self, idx, a: Node, b: Node):
        self.id = idx
        self.start_node = a
        self.end_node = b
        self.cost = 0

        self.in_edge_set = []  # 上游edge集合
        self.out_edge_set = []  # 下游edge集合
        self.clashed_edge_set = []  # 和当前edge有干涉的边的集合
        self.end_clashed_edge_set = []  # 和当前edge的终点有干涉的边的集合
        self.end_end_clashed_edge_set = []  # 终点和当前edge的终点有干涉的边的集合
        self.opposite_edge_idx = -1
        self.empty_empty_clashed_edge_set = []  # 空载-空载干涉集合
        self.empty_load_clashed_edge_set = []  # 空载-负载干涉集合
        self.empty_multi_clashed_edge_set = []  # 空载-超载干涉集合
        self.load_empty_clashed_edge_set = []  # 负载-空载干涉集合
        self.load_load_clashed_edge_set = []  # 负载-负载干涉集合
        self.load_multi_clashed_edge_set = []  # 负载-超载干涉集合
        self.multi_empty_clashed_edge_set = []  # 超载-空载干涉集合
        self.multi_load_clashed_edge_set = []  # 超载-负载干涉集合
        self.multi_multi_clashed_edge_set = []  # 超载-超载干涉集合

        # 主时间窗
        self.main_in_time = list()
        self.main_out_time = list()
        self.main_time_win = list()
        self.main_idx_mission = list()
        self.main_status = list()

        # 辅助时间窗, 相当于记录了其他任务导致的所有不可用时间窗
        # 记录形式是嵌套列表[[],[]]
        self.sub_win = list()
        self.sub_mission = list()

        self.empty_win = []
        self.empty_mission = []

        self.load_win = []
        self.load_mission = []

        self.multi_load_win = []
        self.multi_load_mission = []

        # 总时间窗
        self.total_in_time = list()
        self.total_out_time = list()
        self.total_mission = list()

    def getInEdgeSet(self):
        return self.in_edge_set

    def getOutEdgeSet(self):
        return self.out_edge_set

    def getClashedEdgeSet(self):
        return self.clashed_edge_set

    def getEndClashedEdgeSet(self):
        return self.end_clashed_edge_set

    def getEndEndClashedEdgeSet(self):
        return self.end_end_clashed_edge_set


# 地图信息
class Map:
    """
    地图信息文件在该类里读取
    包括:
    ClashedEdges: 记录每个edge的冲突路径
    EndClashedEdges: 记录每个edge的末端冲突路径
    EndEndClashedEdges: 记录每个edge的终点的末端冲突路径
    InEdges: 记录每条edge的上游路径
    OutEdges: 记录每条edge的下游路径
    path: 记录每条arc的cost，起点坐标，终点坐标，角度
    """
    def __init__(self):
        self.edge_dic = {}  # 字典，key为edge的idx，value为edge实例，便于索引
        self.node_set = list()  # 节点集合，地图均由边定义，用不到
        self.park_set = set()  # 库位，暂时用不到
        self.empty_empty_set = None
        self.empty_load_set = None
        self.load_empty_set = None
        self.load_load_set = None

        # 实例化map
        path_file = 'data/path.txt'
        clash_file = 'data/ClashedEdges.txt'
        end_file = 'data/EndClashedEdges.txt'
        end_end_file = 'data/EndEndClashedEdges.txt'
        in_file = 'data/InEdges.txt'
        out_file = 'data/OutEdges.txt'
        self.readPath(path_file)
        self.recompute_conflict()
        self.readClashedEdges(clash_file)
        self.readEndClashedEdges(end_file)
        self.readEndEndClashedEdges(end_end_file)
        self.readInEdges(in_file)
        self.readOutEdges(out_file)
        self.getOppositeEdge()

    def loadMap(self, filename):
        pass

    def readPath(self, path_file):
        '''
        最先运行该函数，确定了地图上所有edge的字典
        示例:
        1:0.589583;82.380000,45.460000,3.141593;82.663000,45.460000,3.141593;
        '''
        with open(path_file, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;|,', line)
            cur_edge_idx = int(split_line[0])
            info = list(map(float, split_line[1:-1]))
            cost = float(info[0])
            start_x = info[1]
            start_y = info[2]
            end_x = info[4]
            end_y = info[5]
            start_node = Node(start_x, start_y)
            end_node = Node(end_x, end_y)
            cur_edge = Edge(cur_edge_idx, start_node, end_node)
            cur_edge.cost = int(cost * 100) + 1
            self.edge_dic[cur_edge_idx] = cur_edge

    def readClashedEdges(self, filename):
        '''
        get clashed edges set for every edge
        自身与自身也是干涉关系，后面几个函数同理
        '''
        with open(filename, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;', line)[:-1]  # remove \n
            cur_edge_idx = int(split_line[0])
            clashed_set = list(map(int, split_line[1:]))
            """
            self.edge_dic[cur_edge_idx].clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].empty_empty_clashed_edge_set = self.empty_empty_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].empty_load_clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].load_empty_clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].load_load_clashed_edge_set = self.load_load_set[cur_edge_idx]
            """

            self.edge_dic[cur_edge_idx].clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].empty_empty_clashed_edge_set = self.empty_empty_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].empty_load_clashed_edge_set = self.empty_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].empty_multi_clashed_edge_set = self.empty_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].load_multi_clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].load_empty_clashed_edge_set = self.load_empty_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].load_load_clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].multi_multi_clashed_edge_set = self.load_load_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].multi_empty_clashed_edge_set = self.load_empty_set[cur_edge_idx]
            self.edge_dic[cur_edge_idx].multi_load_clashed_edge_set = self.load_load_set[cur_edge_idx]

            """
            self.edge_dic[cur_edge_idx].clashed_edge_set = clashed_set
            self.edge_dic[cur_edge_idx].empty_empty_clashed_edge_set = clashed_set
            self.edge_dic[cur_edge_idx].empty_load_clashed_edge_set = clashed_set
            self.edge_dic[cur_edge_idx].load_empty_clashed_edge_set = clashed_set
            self.edge_dic[cur_edge_idx].load_load_clashed_edge_set = clashed_set
            """

    def readEndClashedEdges(self, filename):
        '''
        get end clashed edges set for every edge
        '''
        with open(filename, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;', line)[:-1]
            cur_edge_idx = int(split_line[0])
            end_clashed_set = list(map(int, split_line[1:]))
            self.edge_dic[cur_edge_idx].end_clashed_edge_set = end_clashed_set

    def readEndEndClashedEdges(self, filename):
        '''
        get end end clashed edges set for every edge
        '''
        with open(filename, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;', line)[:-1]
            cur_edge_idx = int(split_line[0])
            end_end_clashed_set = list(map(int, split_line[1:]))
            self.edge_dic[cur_edge_idx].end_end_clashed_edge_set = end_end_clashed_set

    def readInEdges(self, filename):
        '''
        get in edges set for every edge
        '''
        with open(filename, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;', line)[:-1]
            cur_edge_idx = int(split_line[0])
            in_edge_set = list(map(int, split_line[1:]))
            self.edge_dic[cur_edge_idx].in_edge_set = in_edge_set

    def readOutEdges(self, filename):
        '''
        get in edges set for every edge
        '''
        with open(filename, encoding='utf-8') as file:
            content = file.readlines()
        for line in content:
            split_line = re.split(':|;', line)[:-1]
            cur_edge_idx = int(split_line[0])
            out_edge_set = list(map(int, split_line[1:]))
            self.edge_dic[cur_edge_idx].out_edge_set = out_edge_set

    def getOppositeEdge(self):
        res = {}  # 用于print结果
        for idx_1 in self.edge_dic:
            edge_1 = self.edge_dic[idx_1]
            x_1 = edge_1.start_node.x_cor
            y_1 = edge_1.start_node.y_cor
            x_2 = edge_1.end_node.x_cor
            y_2 = edge_1.end_node.y_cor
            if x_1 == x_2 and y_1 == y_2:  # 起终点一样，说明是终点边，跳过
                continue
            for idx_2 in edge_1.clashed_edge_set:
                edge_2 = self.edge_dic[idx_2]
                if edge_1.id == edge_2.id:  # 去掉自身
                    continue
                elif edge_1.cost != edge_2.cost:  # cost不一样的两条边一定不互为双向边
                    continue
                else:  # 两条不同的边cost一样，有可能是双向边，检查两者起终点
                    x_3 = edge_2.start_node.x_cor
                    y_3 = edge_2.start_node.y_cor
                    x_4 = edge_2.end_node.x_cor
                    y_4 = edge_2.end_node.y_cor
                    if x_1 == x_4 and x_2 == x_3 and y_1 == y_4 and y_2 == y_3:
                        edge_1.opposite_edge_idx = edge_2.id
                        res[edge_1.id] = edge_2.id
        return res

    # 这俩函数用不到
    # 使用节点id找到节点对象

    def getNodebyId(self, index):
        for node in self.node_set:
            if node.Node_id == index:
                return node
        return None
        pass

    # 使用边id找到边对象
    def getEdgebyId(self, index):
        for edge in self.edge_dic:
            if edge.id == index:
                return edge
        return None
        pass

    def dynamic_conflict(self, status):
        """
        =0，表示空载-空载
        =1，表示空载-负载
        =2，表示负载-空载
        =3，表示负载-负载
        :param status:
        :return:
        """
        # 设置超出长度与宽度
        LoadLength = 0.5
        LoadWidth = 0.4
        AgvLength = 2.9
        safeLength = 0
        AgvWidth = 1.5
        conflict_set = {}

        if status == 0:
            print('空载-空载计算')
            ExtendLength1 = safeLength
            ExtendWidth1 = 0
            ExtendLength2 = safeLength
            ExtendWidth2 = 0
        elif status == 1:
            print('空载-负载计算')
            ExtendLength1 = safeLength
            ExtendWidth1 = 0
            ExtendLength2 = LoadLength
            ExtendWidth2 = LoadWidth
        elif status == 2:
            print('负载-空载计算')
            ExtendLength1 = LoadLength
            ExtendWidth1 = LoadWidth
            ExtendLength2 = safeLength
            ExtendWidth2 = 0
        else:
            print('负载-负载计算')
            ExtendLength1 = LoadLength
            ExtendWidth1 = LoadWidth
            ExtendLength2 = LoadLength
            ExtendWidth2 = LoadWidth

        for index1, arc1 in self.edge_dic.items():
            conflict_set[index1] = []
            # 判断边是横着的还是竖着的
            if arc1.end_node.y_cor - arc1.start_node.y_cor > 0.001:
                # 竖着的
                extend_node1 = [arc1.end_node.x_cor, arc1.end_node.y_cor + ExtendLength1]
            elif arc1.end_node.x_cor - arc1.start_node.x_cor > 0.001:
                # 横着的
                extend_node1 = [arc1.end_node.x_cor + ExtendLength1, arc1.end_node.y_cor]
            else:
                extend_node1 = [arc1.end_node.x_cor + ExtendLength1, arc1.end_node.y_cor+ ExtendLength1]
            center_point1 = ((arc1.start_node.x_cor + extend_node1[0]) / 2, (arc1.start_node.y_cor + extend_node1[1]) / 2)
            # 计算矩形长度
            Length1 = math.sqrt(abs(extend_node1[0] - arc1.start_node.x_cor)**2 + abs(extend_node1[1] - arc1.start_node.y_cor)**2)
            # Length1 = max(Length1, AgvLength / 2)
            Width1 = AgvWidth + 2 * ExtendWidth1
            angle1 = math.degrees(math.atan2((arc1.start_node.x_cor - extend_node1[0]), (arc1.start_node.y_cor - extend_node1[1])))
            rect1 = (center_point1, (Length1, Width1), angle1)
            for index2, arc2 in self.edge_dic.items():
                # 判断边是横着的还是竖着的
                if arc2.end_node.y_cor - arc2.start_node.y_cor > 0.001:
                    # 竖着的
                    extend_node2 = [arc2.end_node.x_cor, arc2.end_node.y_cor + ExtendLength2]
                elif arc2.end_node.x_cor - arc2.start_node.x_cor > 0.001:
                    # 横着的
                    extend_node2 = [arc2.end_node.x_cor + ExtendLength2, arc2.end_node.y_cor]
                else:
                    extend_node2 = [arc2.end_node.x_cor + ExtendLength2, arc2.end_node.y_cor + ExtendLength2]
                center_point2 = ((arc2.start_node.x_cor + extend_node2[0]) / 2, (arc2.start_node.y_cor + extend_node2[1]) / 2)
                Length2 = math.sqrt(abs(extend_node2[0] - arc2.start_node.x_cor) ** 2 + abs(
                    extend_node2[1] - arc2.start_node.y_cor) ** 2)
                # Length2 = max(Length2, AgvLength / 2)
                Width2 = AgvWidth + 2 * ExtendWidth2
                angle2 = math.degrees(
                    math.atan2((arc2.start_node.x_cor - extend_node2[0]), (arc2.start_node.y_cor - extend_node2[1])))
                rect2 = (center_point2, (Length2, Width2), angle2)
                r1 = cv2.rotatedRectangleIntersection(rect1, rect2)  # 区分正负角度，逆时针为负，顺时针为正
                if r1[0] == 0:
                    area = 0
                else:
                    area = cv2.contourArea(r1[1])  # 求相交面积
                if area > 0:
                    conflict_set[index1].append(index2)

        for key, value in conflict_set.items():
            origin = sorted(self.edge_dic[key].clashed_edge_set)
            new = value
            # print('不在原始冲突集合中的边：', [item for item in new if item not in origin])
            # print(sorted(self.edge_dic[1].clashed_edge_set))
            # print(key, value)
            print('原始冲突边数量：', len(origin), '当前冲突边数量', len(value))
            break
        return conflict_set

    # 重新计算冲突关系
    def recompute_conflict(self):
        """
        调用opencv库计算
        关键在于通过参数调整使得负载和空载计算的冲突关系有差别
        具体准不准其实没关系，因为企业还会考虑更多实际因素
        :return:
        """
        """
        # 空载-空载
        self.empty_empty_set = self.dynamic_conflict(0)
        self.empty_load_set = self.dynamic_conflict(1)
        self.load_empty_set = self.dynamic_conflict(2)
        self.load_load_set = self.dynamic_conflict(3)

        with open('data/empty_empty.pkl', 'wb') as tf:
            pickle.dump(self.empty_empty_set, tf)
        with open('data/empty_load.pkl', 'wb') as tf:
            pickle.dump(self.empty_load_set, tf)
        with open('data/load_empty.pkl', 'wb') as tf:
            pickle.dump(self.load_empty_set, tf)
        with open('data/load_load.pkl', 'wb') as tf:
            pickle.dump(self.load_load_set, tf)
        """

        with open('data/empty_empty.pkl', 'rb') as tf:
            self.empty_empty_set = pickle.load(tf)

        with open('data/empty_load.pkl', 'rb') as tf:
            self.empty_load_set = pickle.load(tf)

        with open('data/load_empty.pkl', 'rb') as tf:
            self.load_empty_set = pickle.load(tf)

        with open('data/load_load.pkl', 'rb') as tf:
            self.load_load_set = pickle.load(tf)

        return


if __name__ == '__main__':
    test_map = Map()
