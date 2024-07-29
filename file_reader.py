import string
from typing import Any, Dict
import pandas as pd
import numpy as np
import math
import statistics

"""
This file contains the functions that extract the parameters and check them for instances
"""


def get_parameters(file: string, num: int=0) -> Dict[string, Any]:
    """
    Extract parameters from the instance files
    :param file: txt instance file
    :param different_dummy: whether to use dummy with different ID from original charging stations
    :param num: number of dummy for each charging station
    :return: dict storing parameters
    """

    # get the data frame cleaning the parameter description sentences in the last 5 rows
    df = pd.read_csv(file, sep='\s+')
    df_filtered = df.iloc[:-5]

    # convert the data frame to numpy array
    original_data = df_filtered.to_numpy()

    # get the copy of the depot row and change the name
    depot_copy = original_data[0].copy()
    depot_copy[0] = "D0_end"

    # read lines from text and get the general parameters from last 5 rows
    with open(file, 'r') as file:
        lines = file.readlines()

        for line in lines:
            if line.startswith('Q Vehicle fuel tank capacity'):
                Q = float(line.split('/')[1])
            elif line.startswith('C Vehicle load capacity'):
                C = float(line.split('/')[1])
            elif line.startswith('g inverse refueling rate'):
                g = float(line.split('/')[1])
            elif line.startswith('r fuel consumption rate'):
                h = float(line.split('/')[1])
            elif line.startswith('v average Velocity'):
                v = float(line.split('/')[1])


    # get the number of charging stations and the index of the final station
    final_station_index = np.sum(df_filtered["Type"] == 'f')
    stations_copy = original_data[1:final_station_index + 1].copy()
    original_stations = [str(row[0]) for count, row in enumerate(stations_copy)]

    # replicate all the stations and change the names
    replicate_stations = np.tile(stations_copy, (num,1))
    for count, value in enumerate(replicate_stations):
        value[0] = "S_dummy" + str(count)

    # concatenate all the arrays
    final_data = np.vstack((original_data, depot_copy, replicate_stations))

    # extract the client, charging stations, depots and all nodes
    clients = [str(row[0]) for count, row in enumerate(final_data) if str(row[1]) == "c"]
    stations = [str(row[0]) for count, row in enumerate(final_data) if str(row[1]) == "f"]
    all_nodes = [str(row[0]) for count, row in enumerate(final_data)]
    depot_start = ["D0"]
    depot_end = ["D0_end"]

    # extract distance, demand, ready time, due date and service time
    locations = {}
    demand = {}
    ready_time = {}
    due_date = {}
    service_time = {}
    arcs = {}
    times = {}

    for index, row in enumerate(final_data):
        locations[row[0]] = (float(row[2]), float(row[3]))
        demand[row[0]] = float(row[4])
        ready_time[row[0]] = float(row[5])
        due_date[row[0]] = float(row[6])
        service_time[row[0]] = float(row[7])

    for key1, value1 in locations.items():
        for key2, value2 in locations.items():
            arcs[(key1, key2)] = math.sqrt((value1[0] - value2[0])**2 + ((value1[1] - value2[1]))**2)
            times[(key1, key2)] = math.sqrt((value1[0] - value2[0])**2 + ((value1[1] - value2[1]))**2)/v


    travel_time_series = []
    for client in clients:
        travel_time_series.append(due_date[client] - ready_time[client])

    std = statistics.stdev(travel_time_series)

    normal_times = times






    parameters = {"Q": Q, "C": C, "g": g, "h": h, "v": v, "clients": clients, "stations": stations,
                  "all_nodes": all_nodes, "depot_start": depot_start, "depot_end": depot_end,
                  "demand": demand, "ready_time": ready_time, "due_date": due_date, "service_time": service_time,
                  "arcs": arcs, "times": times, "final_data": final_data, "original_stations": original_stations,
                  "locations": locations, "std": statistics.stdev(travel_time_series), "mean": statistics.mean(travel_time_series),
                  "time_series": travel_time_series, "normal_times": normal_times}

    return parameters
