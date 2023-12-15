"""
Author: Zhuge Qinqin
Time: 2022-08-14
Purpose: 读取企业常规任务文件，生成任务
Special: 需要实现库位id-路径id的转换
Deadline: 周四
"""

import xml.dom.minidom
from functools import reduce
import operator
import pandas as pd
import random
import pickle
total_opp = {7148: 7146, 7147: 7145, 7079: 7078, 6742: 6738, 4071: 4070, 4073: 4072, 6558: 6557, 6560: 6559, 6554: 6553, 6556: 6555,
             7293: 7373, 4135: 4134, 7304: 7303, 4121: 4120, 4109: 4108, 4115: 4114, 4127: 4126, 4111: 4110, 4113: 4112,
             4117: 4116, 4119: 4118, 4123: 4122, 4125: 4124, 4129: 4128, 4131: 4130, 3805: 3804, 3807: 3806, 3809: 3807,
             7067: 7066, 6994: 6993, 8042: 8041}

total = list(total_opp.keys()) + list(total_opp.values())


def readPath(path):
    """
    读取path文件
    :param path:
    :return:
    """
    dom = xml.dom.minidom.parse(path)
    root = dom.documentElement
    itemList = root.getElementsByTagName('Path')
    item = itemList[0]
    un = item.getAttribute("No")
    Path = {}
    for item in itemList:
        un = int(item.getAttribute("No"))
        ut = item.getAttribute("Detail")
        a = [x.split(',') for x in ut.split(';')]
        b = reduce(operator.add, a)
        Path[un] = b

    p = pd.DataFrame.from_dict(Path, orient='index', columns=['ward', 'sx', 'sy', 'sr', 'dx', 'dy', 'dr'])
    return p


def readParks(path):
    """
    读取库位文件
    :param path:
    :return:
    """
    dom = xml.dom.minidom.parse(path)
    root = dom.documentElement
    itemList = root.getElementsByTagName('AGVPark')
    AGVPark = {}
    for item in itemList:
        un = int(item.getAttribute("Id"))
        ut = item.getAttribute("District")
        AGVPark[un] = ut
    return AGVPark


def readBackPark(path):
    """
    读取库位路径对应表
    :param path:
    :return:
    """
    dom = xml.dom.minidom.parse(path)
    root = dom.documentElement
    itemList = root.getElementsByTagName('Record')
    park_path_set = {}
    for item in itemList:
        ParkId = int(item.getAttribute("ParkId"))
        PathNo = int(item.getAttribute("PathNo"))
        park_path_set[ParkId] = PathNo
    return park_path_set


def readParkSet(path):
    """
    读取仓库位置集合
    :param path:
    :return:
    """
    dom = xml.dom.minidom.parse(path)
    root = dom.documentElement
    itemList = root.getElementsByTagName('ParkSet')
    ParkSet = {}
    for item in itemList:
        name = item.getAttribute("Name")
        strs = item.getAttribute("Parks").split(';')
        Parks = [int(item) for item in strs]
        ParkSet[name] = Parks
    return ParkSet


def readMission(path):
    """
    读取任务并将其转换成pathId类型的任务
    :param path:
    :return:
    """
    park_path = readBackPark('data/BackParkTable.config')
    ParkSets = readParkSet('data/ParksSet.config')
    dom = xml.dom.minidom.parse(path)
    root = dom.documentElement
    itemList = root.getElementsByTagName('Group')
    missionList = {}
    for index, item in enumerate(itemList):
        start_time = 0   # 开始时间
        num = 40   # 任务批次
        FlowDuration = float(item.getAttribute("FlowDuration")) * 60 * 60  # 持续时间
        TimeGap = 200
        taskList = item.getElementsByTagName('Task')
        mission_set = []
        random_time = 0
        for i in range(0, num):
            mission = {}
            random_time = random_time + random.randint(5, 10)
            mission['start_time'] = int(start_time + i * TimeGap + random_time)   # 开始时间
            for task in taskList:
                Type = task.getAttribute("Type")
                GoalMode = task.getAttribute("GoalMode")
                if GoalMode == "ParkId":
                    Id = int(task.getAttribute("Goal"))
                    pathId = park_path[Id]
                if GoalMode == "ParkSet":
                    Id = task.getAttribute("Goal")
                    pathId = park_path[random.choice(ParkSets[Id])]
                mission[Type] = pathId
            mission_set.append(mission)
        missionList[index] = mission_set

    for key, value in missionList.items():
        print(key, value)

    Note = open('mission_data/mission_11.txt', mode='w')
    index = 0
    idx = 1
    start_time = 0
    while index < 25:
        for key, value in missionList.items():
            mission_info = value[index]
            if mission_info['GET'] in total or mission_info['PUT'] in total:
                continue
            start_time += random.randint(15, 25)
            Note.write(str(idx) + ' ' + str(start_time) + ' '
                       + str(mission_info['GET']) + ' ' + str(mission_info['PUT']) + ' ' + str(mission_info['MOVE_TO']) + ' 1 2 3' + '\n')
            idx += 1
        index += 1
    Note.close()
    return missionList


def random_mission():
    """
    读取地图信息，随机生成任务
    :return:
    """
    park_path = readBackPark('data/BackParkTable.config')
    ParkSets = readParkSet('data/ParksSet.config')
    # 生成100个任务
    idx = 0
    start_time = 0
    Note = open('mission_data/test_mission.txt', mode='w')
    for i in range(300):
        # type1是先取小货物，再取大货物，再送大货物，再送小货物
        start_time += random.randint(25, 35)
        GET_Region = random.choice(["C1", "C2"])
        GET_1 = park_path[random.choice(ParkSets[GET_Region])]
        PUT_Region = random.choice(["C3", "C4"])
        PUT_1 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_1 in total or PUT_1 in total:
            continue

        GET_Region = random.choice(["C3", "C4"])
        GET_2 = park_path[random.choice(ParkSets[GET_Region])]
        PUT_Region = random.choice(["A1", "A2"])
        PUT_2 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_2 in total or PUT_2 in total:
            continue
        idx += 1
        Note.write(str(idx) + ' ' + str(start_time) + ' '
                   + str(GET_1) + ' ' + str(GET_2) + ' ' + str(PUT_1) + ' ' + str(PUT_2) + ' ' + '1' + '\n')

        # type2是先取大货物，再取小货物，再送大货物，再送小货物
        GET_1 = park_path[random.choice([1051, 1086])]
        PUT_Region = random.choice(["B3", "B4"])
        PUT_1 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_1 in total or PUT_1 in total:
            continue

        start_time += random.randint(25, 35)
        GET_Region = random.choice(["B1", "B2"])
        GET_2 = park_path[random.choice(ParkSets[GET_Region])]
        PUT_Region = random.choice(["C3", "C4"])
        PUT_2 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_2 in total or PUT_2 in total:
            continue
        idx += 1
        Note.write(str(idx) + ' ' + str(start_time) + ' '
                   + str(GET_1) + ' ' + str(GET_2) + ' ' + str(PUT_1) + ' ' + str(PUT_2) + ' ' + '2' + '\n')

        # type2是先取大货物，再取小货物，再送大货物，再送小货物
        GET_Region = random.choice(["B1", "B2"])
        GET_1 = park_path[random.choice(ParkSets[GET_Region])]
        PUT_Region = random.choice(["C1", "C2"])
        PUT_1 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_1 in total or PUT_1 in total:
            continue

        start_time += random.randint(30, 35)
        GET_Region = random.choice(["C1", "C2"])
        GET_2 = park_path[random.choice(ParkSets[GET_Region])]
        PUT_Region = random.choice(["C3", "C4"])
        PUT_2 = park_path[random.choice(ParkSets[PUT_Region])]
        if GET_2 in total or PUT_2 in total:
            continue
        idx += 1
        Note.write(str(idx) + ' ' + str(start_time) + ' '
                   + str(GET_1) + ' ' + str(GET_2) + ' ' + str(PUT_1) + ' ' + str(PUT_2) + ' ' + '2' + '\n')

    Note.close()
    return


if __name__ == '__main__':
    paths = readPath('data/path.config')
    parks = readParks('data/Parks.config')
    random_mission()
    # readMission('data/tasks.config')
