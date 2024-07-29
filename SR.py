from mip_check import MIPCheck
from helper_function import Helper
from math import ceil, floor
from random import uniform, sample, random


class StationRemoval():
    def __init__(self, parameters):
        """
        This is a constructor to create a station removal object
        :param parameters: parameters got from a file reader from an instance
        """
        self.parameters = parameters
        self.checker = MIPCheck(self.parameters)
        self.helper = Helper(self.parameters)
        self.clients = self.parameters["clients"]
        self.stations = self.parameters["stations"]
        self.all_nodes = self.parameters["all_nodes"]
        self.depot_start = self.parameters["depot_start"]
        self.depot_end = self.parameters["depot_end"]
        self.demand = self.parameters["demand"]
        self.ready_time = self.parameters["ready_time"]
        self.due_date = self.parameters["due_date"]
        self.service_time = self.parameters["service_time"]
        self.arcs = self.parameters["arcs"]
        self.times = self.parameters["times"]
        self.final_data = self.parameters["final_data"]
        self.original_stations = self.parameters["original_stations"]
        self.Q = self.parameters["Q"]
        self.C = self.parameters["C"]
        self.g = self.parameters["g"]
        self.h = self.parameters["h"]
        self.v = self.parameters["v"]
        # define the tuned parameters before start the algorithms
        self.lower = 0.1
        self.upper = 0.4

    def random_removal(self, routes):
        """
        This is the function to randomly remove some stations in routes
        :param routes:
        :return:
        """
        # compute the sigma, number of stations having to be removed
        # first count how many stations are there in the routes
        counter_stations = 0
        for route in routes:
            for node in route:
                if node in self.original_stations:
                    counter_stations += 1

        # get the upper and lower
        removal_lower = int(min(0.1 * counter_stations, 30))
        removal_upper = int(min(0.4 * counter_stations, 60))
        sigma = ceil(uniform(removal_lower, removal_upper))

        # make a list of the index of stations
        index_stations = []
        for i in range(len(routes)):
            for j in range(len(routes[i])):
                if routes[i][j] in self.original_stations:
                    index_stations.append((i, j))

        # sample to get random removed stations indices
        removal_stations = sample(index_stations, sigma)

        # then we remove the stations
        new_routes = []
        for i in range(len(routes)):
            new_route = []
            for j in range(len(routes[i])):
                if (i, j) in removal_stations:
                    continue
                else:
                    new_route.append(routes[i][j])
            new_routes.append(new_route)

        return new_routes

    def worst_distance_removal(self, routes):
        """
        This is the function to remove the worst distance stations
        :param routes: a solution
        :return: the new routes with removing some stations
        """
        counter_stations = 0
        for route in routes:
            for node in route:
                if node in self.original_stations:
                    counter_stations += 1

        # get the upper and lower
        removal_lower = int(min(0.1 * counter_stations, 30))
        removal_upper = int(min(0.4 * counter_stations, 60))
        sigma = ceil(uniform(removal_lower, removal_upper))

        # create a dict containing the distance
        distance_stations = {}
        for i in range(len(routes)):
            for j in range(len(routes[i])):
                if routes[i][j] in self.original_stations:
                    distance_stations[(i,j)] = (self.arcs[routes[i][j-1], routes[i][j]] +
                                                self.arcs[routes[i][j+1], routes[i][j]])

        # decreasing order of the stations since we want to remove the high cost, which is long distance
        sorted_stations = sorted(distance_stations, key=distance_stations.get, reverse=True)

        # since in the paper, there is no parameters for randomly choosing the stations, we choose the first ones
        removal_stations = sorted_stations[:sigma+1]

        # then we remove the stations
        new_routes = []
        for i in range(len(routes)):
            new_route = []
            for j in range(len(routes[i])):
                if (i, j) in removal_stations:
                    continue
                else:
                    new_route.append(routes[i][j])
            new_routes.append(new_route)

        return new_routes

    def worst_charge_removal(self, routes):
        """
        This is the function to get the worst charge to remove stations
        :param routes:
        :return:
        """
        counter_stations = 0
        for route in routes:
            for node in route:
                if node in self.original_stations:
                    counter_stations += 1

        # get the upper and lower
        removal_lower = int(min(0.1 * counter_stations, 30))
        removal_upper = int(min(0.4 * counter_stations, 60))
        sigma = ceil(uniform(removal_lower, removal_upper))

        # create a dict to contain all arrival energy of the stations
        energy_cost = {}
        for i in range(len(routes)):
            arrival_energy = self.checker.energy_extractor(routes[i])
            for j in range(len(routes[i])):
                if routes[i][j] in self.original_stations:
                    energy_cost[(i, j)] = arrival_energy[j]

        # sorted the stations from high arrival energy to low, since high energy arrival means high cost
        sorted_stations = sorted(energy_cost, key=energy_cost.get, reverse=True)

        # from the paper, we remove the first ones
        removal_stations = sorted_stations[:sigma + 1]

        # then we remove the stations
        new_routes = []
        for i in range(len(routes)):
            new_route = []
            for j in range(len(routes[i])):
                if (i, j) in removal_stations:
                    continue
                else:
                    new_route.append(routes[i][j])
            new_routes.append(new_route)

        return new_routes

    def full_removal(self, routes):
        """
        This is the function to remove all the full recharge stations
        :param routes: solution
        :return: new routes without any full recharge stations
        """
        counter_stations = 0
        for route in routes:
            for node in route:
                if node in self.original_stations:
                    counter_stations += 1

        # get the upper and lower
        removal_lower = int(min(0.1 * counter_stations, 30))
        removal_upper = int(min(0.4 * counter_stations, 60))
        sigma = ceil(uniform(removal_lower, removal_upper))

        # create a list to store the index of all full recharge stations
        removal_stations = []
        for i in range(len(routes)):
            departure_energy = self.checker.energy_extractor_departure(routes[i])
            for j in range(len(routes[i])):
                if routes[i][j] in self.original_stations:
                    if departure_energy[j] == self.Q:
                        removal_stations.append((i, j))

        # then we remove the stations
        # first we test if the number of the full charge stations is greater than sigma
        # if smaller or equal, we remove them all, otherwise we randomly remove sigma
        new_routes = []
        if len(removal_stations) <= sigma:
            for i in range(len(routes)):
                new_route = []
                for j in range(len(routes[i])):
                    if (i, j) in removal_stations:
                        continue
                    else:
                        new_route.append(routes[i][j])
                new_routes.append(new_route)
        else:
            new_removal_stations = sample(removal_stations, sigma)
            for i in range(len(routes)):
                new_route = []
                for j in range(len(routes[i])):
                    if (i, j) in new_removal_stations:
                        continue
                    else:
                        new_route.append(routes[i][j])
                new_routes.append(new_route)
        return new_routes
