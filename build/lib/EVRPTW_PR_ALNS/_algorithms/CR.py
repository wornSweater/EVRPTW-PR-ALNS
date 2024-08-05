from EVRPTW_PR_ALNS.mip_check import MIPCheck
from EVRPTW_PR_ALNS.helper_function import Helper
from math import ceil, floor
from random import uniform, sample, random
from copy import deepcopy
import numpy as np


class CustomerRemoval:
    def __init__(self, parameters):
        """
        This is a constructor to create a customer removal object
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
        self.locations = self.parameters["locations"]
        self.Q = self.parameters["Q"]
        self.C = self.parameters["C"]
        self.g = self.parameters["g"]
        self.h = self.parameters["h"]
        self.v = self.parameters["v"]
        # define the tuned parameters before start the _algorithms
        self.removal_lower = int(min(0.1 * len(self.clients), 30))
        self.removal_upper = int(min(0.4 * len(self.clients), 60))
        self.worst_removal_factor = 4
        self.shaw_removal_factor = 12
        self.phi1 = 0.5
        self.phi2 = 13
        self.phi3 = 0.15
        self.phi4 = 0.25
        self.routes_number_lower = 0.1
        self.mr = 0.3
        self.increment = 5
        self.removal = []

    def reset_removal(self):
        """
        Void function, reset the removal list
        """
        self.removal = []

    def random_removal(self, routes):
        """
        This is the function to randomly remove customers from the feasible solution
        :param routes: feasible routes
        :return still a feasible solution without some customers and add the removed to the removal_list
        """
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))
        # use sample to update the removal list, containing the customers to be removed
        self.removal = sample(self.clients, gamma)
        # deep copy the routes for list change
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def random_removal_prev(self, routes):
        """
        This is the function to remove the previous station
        @param routes: solution
        @return: new routes (probably not feasible)
        """
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))
        # use sample to update the removal list, containing the customers to be removed
        self.removal = sample(self.clients, gamma)

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def random_removal_next(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))
        # use sample to update the removal list, containing the customers to be removed
        self.removal = sample(self.clients, gamma)

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_distance_removal(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        distance_cost = {}

        for route in routes:
            for i in range(len(route)):
                if route[i] in self.clients:
                    distance_cost[route[i]] = self.arcs[route[i], route[i - 1]] + self.arcs[route[i], route[i + 1]]

        # sort the dict according to the descending order
        sorted_customers = sorted(distance_cost, key=distance_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        # after get the removal list, we can remove the customers, same procedure as the random removal
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def worst_distance_removal_prev(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        distance_cost = {}

        for route in routes:
            for i in range(len(route)):
                if route[i] in self.clients:
                    distance_cost[route[i]] = self.arcs[route[i], route[i - 1]] + self.arcs[route[i], route[i + 1]]

        # sort the dict according to the descending order
        sorted_customers = sorted(distance_cost, key=distance_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_distance_removal_next(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        distance_cost = {}

        for route in routes:
            for i in range(len(route)):
                if route[i] in self.clients:
                    distance_cost[route[i]] = self.arcs[route[i], route[i - 1]] + self.arcs[route[i], route[i + 1]]

        # sort the dict according to the descending order
        sorted_customers = sorted(distance_cost, key=distance_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_time_removal(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        time_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            arrival_time = self.checker.time_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    time_cost[route[i]] = abs(arrival_time[i] - self.ready_time[route[i]])

        # sort the dict according to the descending order
        sorted_customers = sorted(time_cost, key=time_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        # after get the removal list, we can remove the customers, same procedure as the random removal
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def worst_time_removal_prev(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        time_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            arrival_time = self.checker.time_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    time_cost[route[i]] = abs(arrival_time[i] - self.ready_time[route[i]])

        # sort the dict according to the descending order
        sorted_customers = sorted(time_cost, key=time_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_time_removal_next(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        time_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            arrival_time = self.checker.time_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    time_cost[route[i]] = abs(arrival_time[i] - self.ready_time[route[i]])

        # sort the dict according to the descending order
        sorted_customers = sorted(time_cost, key=time_cost.get, reverse=True)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_energy_removal(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        energy_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            departure_energy = self.checker.energy_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    energy_cost[route[i]] = departure_energy[i]

        # sort the dict according to the increasing order, since when the smaller is the energy, more is the cost
        sorted_customers = sorted(energy_cost, key=energy_cost.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        # after get the removal list, we can remove the customers, same procedure as the random removal
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def worst_energy_removal_prev(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        energy_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            departure_energy = self.checker.energy_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    energy_cost[route[i]] = departure_energy[i]

        # sort the dict according to the increasing order, since when the smaller is the energy, more is the cost
        sorted_customers = sorted(energy_cost, key=energy_cost.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def worst_energy_removal_next(self, routes):
        # reset the removal list to be empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # define the distance cost dict to store the key and value
        energy_cost = {}

        for route in routes:
            # for each route, extract the one to one corresponding arrival time
            departure_energy = self.checker.energy_extractor(route)
            for i in range(len(route)):
                if route[i] in self.clients:
                    energy_cost[route[i]] = departure_energy[i]

        # sort the dict according to the increasing order, since when the smaller is the energy, more is the cost
        sorted_customers = sorted(energy_cost, key=energy_cost.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.worst_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def shaw_removal(self, routes, phi1=0.5, phi2=13, phi3=0.15, phi4=0.25):
        """
        This the shaw removal
        Rij = phi1 * dij + phi2 * |ei - ej| + phi3 * lij + phi4 * |ui - uj|
        lij is -1 if nodes i and j are in the same route !!!!!!!!!!!!!!!!!!
        """
        # reset the removal list to empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # choose a customer randomly from the clients using the random sample method
        # and update the self removal list
        self.removal = sample(self.clients, 1)
        chosen = self.removal[0]

        # create a similarity dict to contain all the relatedness between the chosen and the other customers
        similarity = {}

        # first traverse each route in the solution and check if the customer and the chosen one are in the same route
        # for each route in the solution
        for route in routes:
            # for each node in the route if the node is a client, check if the chosen one is also in
            # also check if they are the same node, if yes, nothing to operate
            for node in route:
                if node in self.clients and chosen in route and node != chosen:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) - phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )
                if node in self.clients and chosen not in route:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) + phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )

        # sort the dict according to the increasing order, since we want to remove similar nodes
        sorted_customers = sorted(similarity, key=similarity.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.shaw_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        # after get the removal list, we can remove the customers, same procedure as the random removal
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def shaw_removal_prev(self, routes, phi1=0.5, phi2=13, phi3=0.15, phi4=0.25):
        # reset the removal list to empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # choose a customer randomly from the clients using the random sample method
        # and update the self removal list
        self.removal = sample(self.clients, 1)
        chosen = self.removal[0]

        # create a similarity dict to contain all the relatedness between the chosen and the other customers
        similarity = {}

        # first traverse each route in the solution and check if the customer and the chosen one are in the same route
        # for each route in the solution
        for route in routes:
            # for each node in the route if the node is a client, check if the chosen one is also in
            # also check if they are the same node, if yes, nothing to operate
            for node in route:
                if node in self.clients and chosen in route and node != chosen:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) - phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )
                if node in self.clients and chosen not in route:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) + phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )

        # sort the dict according to the increasing order, since we want to remove similar nodes
        sorted_customers = sorted(similarity, key=similarity.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.shaw_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def shaw_removal_next(self, routes, phi1=0.5, phi2=13, phi3=0.15, phi4=0.25):
        # reset the removal list to empty again
        self.reset_removal()
        # uniformly choose a gamma as the number of clients to be removed
        gamma = ceil(uniform(self.removal_lower, self.removal_upper))

        # choose a customer randomly from the clients using the random sample method
        # and update the self removal list
        self.removal = sample(self.clients, 1)
        chosen = self.removal[0]

        # create a similarity dict to contain all the relatedness between the chosen and the other customers
        similarity = {}

        # first traverse each route in the solution and check if the customer and the chosen one are in the same route
        # for each route in the solution
        for route in routes:
            # for each node in the route if the node is a client, check if the chosen one is also in
            # also check if they are the same node, if yes, nothing to operate
            for node in route:
                if node in self.clients and chosen in route and node != chosen:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) - phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )
                if node in self.clients and chosen not in route:
                    similarity[node] = (
                            phi1 * self.arcs[node, chosen] + phi2 * abs(
                        self.ready_time[node] - self.ready_time[chosen]) + phi3 + phi4 * abs(
                        self.demand[node] - self.demand[chosen])
                    )

        # sort the dict according to the increasing order, since we want to remove similar nodes
        sorted_customers = sorted(similarity, key=similarity.get)

        # start to update the removal list and remove the customers
        while len(self.removal) < gamma:
            # generate a random (0, 1), and choose the indicated index, while check if this customer is in already
            random_num = random()
            index = floor((random_num ** self.shaw_removal_factor) * gamma)
            # check if the customer corresponding the index is in the removal list or not
            if sorted_customers[index] in self.removal:
                continue
            else:
                self.removal.append(sorted_customers[index])

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def proximity_removal(self, routes):
        """
        A special case of shaw removal for distance
        """
        return self.shaw_removal(routes, phi1=1, phi2=0, phi3=0, phi4=0)

    def proximity_removal_prev(self, routes):
        return self.shaw_removal_prev(routes, phi1=1, phi2=0, phi3=0, phi4=0)

    def proximity_removal_next(self, routes):
        return self.shaw_removal_next(routes, phi1=1, phi2=0, phi3=0, phi4=0)

    def time_removal(self, routes):
        """
        A special case of shaw removal for time
        """
        return self.shaw_removal(routes, phi1=0, phi2=1, phi3=0, phi4=0)

    def time_removal_prev(self, routes):
        return self.shaw_removal_prev(routes, phi1=0, phi2=1, phi3=0, phi4=0)

    def time_removal_next(self, routes):
        return self.shaw_removal_next(routes, phi1=0, phi2=1, phi3=0, phi4=0)

    def demand_removal(self, routes):
        """
        A special case of shaw removal for demand
        """
        return self.shaw_removal(routes, phi1=0, phi2=0, phi3=0, phi4=1)

    def demand_removal_prev(self, routes):
        return self.shaw_removal_prev(routes, phi1=0, phi2=0, phi3=0, phi4=1)

    def demand_removal_next(self, routes):
        return self.shaw_removal_next(routes, phi1=0, phi2=0, phi3=0, phi4=1)

    def zone_removal(self, routes):
        """
        This is the function to remove a bunch of customers in the same zone
        """
        # reset the removal list
        self.reset_removal()

        # find the bound of the all the graph (not only customers, but all nodes)
        x_lower = min(values[0] for location, values in self.locations.items())
        x_upper = max(values[0] for location, values in self.locations.items())
        y_lower = min(values[1] for location, values in self.locations.items())
        y_upper = max(values[1] for location, values in self.locations.items())

        # split into smaller zones based on the coordinates
        # first decide the increments
        x_increment = (ceil(x_upper) - floor(x_lower)) / self.increment
        y_increment = (ceil(y_upper) - floor(y_lower)) / self.increment

        zones = []

        for i in range(floor(x_lower), ceil(x_upper), ceil(x_increment)):
            for j in range(floor(y_lower), ceil(y_upper), ceil(y_increment)):
                zones.append((i, i + x_increment, j, j + y_increment))

        # randomly select a zone and remove the customers inside, if the zone has no customer, continue to next
        while not self.removal:
            removal_zone = sample(zones, 1)

            for route in routes:
                for node in route:
                    if (
                            node in self.clients and removal_zone[0][0] <= self.locations[node][0] <= removal_zone[0][
                        1] and
                            removal_zone[0][2] <= self.locations[node][1] <= removal_zone[0][3]
                    ):
                        self.removal.append(node)

        # after get the removal list, we can remove the customers, same procedure as the random removal
        routes_removal = deepcopy(routes)

        # traverse the feasible solution row by row
        # for each route in the routes, for each client in removal list, if the client in, remove
        for i in range(len(routes)):
            for client in self.removal:
                if client in routes[i]:
                    routes_removal[i].remove(client)
        # return the routes after removing all the clients
        return routes_removal

    def zone_removal_prev(self, routes):
        # reset the removal list
        self.reset_removal()

        # find the bound of the all the graph (not only customers, but all nodes)
        x_lower = min(values[0] for location, values in self.locations.items())
        x_upper = max(values[0] for location, values in self.locations.items())
        y_lower = min(values[1] for location, values in self.locations.items())
        y_upper = max(values[1] for location, values in self.locations.items())

        # split into smaller zones based on the coordinates
        # first decide the increments
        x_increment = (ceil(x_upper) - floor(x_lower)) / self.increment
        y_increment = (ceil(y_upper) - floor(y_lower)) / self.increment

        zones = []

        for i in range(floor(x_lower), ceil(x_upper), ceil(x_increment)):
            for j in range(floor(y_lower), ceil(y_upper), ceil(y_increment)):
                zones.append((i, i + x_increment, j, j + y_increment))

        # randomly select a zone and remove the customers inside, if the zone has no customer, continue to next
        while not self.removal:
            removal_zone = sample(zones, 1)

            for route in routes:
                for node in route:
                    if (
                            node in self.clients and removal_zone[0][0] <= self.locations[node][0] <= removal_zone[0][
                        1] and
                            removal_zone[0][2] <= self.locations[node][1] <= removal_zone[0][3]
                    ):
                        self.removal.append(node)

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j - 1] in self.original_stations:
                        remove_index.append(j - 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def zone_removal_next(self, routes):
        # reset the removal list
        self.reset_removal()

        # find the bound of the all the graph (not only customers, but all nodes)
        x_lower = min(values[0] for location, values in self.locations.items())
        x_upper = max(values[0] for location, values in self.locations.items())
        y_lower = min(values[1] for location, values in self.locations.items())
        y_upper = max(values[1] for location, values in self.locations.items())

        # split into smaller zones based on the coordinates
        # first decide the increments
        x_increment = (ceil(x_upper) - floor(x_lower)) / self.increment
        y_increment = (ceil(y_upper) - floor(y_lower)) / self.increment

        zones = []

        for i in range(floor(x_lower), ceil(x_upper), ceil(x_increment)):
            for j in range(floor(y_lower), ceil(y_upper), ceil(y_increment)):
                zones.append((i, i + x_increment, j, j + y_increment))

        # randomly select a zone and remove the customers inside, if the zone has no customer, continue to next
        while not self.removal:
            removal_zone = sample(zones, 1)

            for route in routes:
                for node in route:
                    if (
                            node in self.clients and removal_zone[0][0] <= self.locations[node][0] <= removal_zone[0][
                        1] and
                            removal_zone[0][2] <= self.locations[node][1] <= removal_zone[0][3]
                    ):
                        self.removal.append(node)

        new_routes = []

        for i in range(len(routes)):
            arr = np.array(routes[i])
            remove_index = []
            for j in range(len(routes[i])):
                if routes[i][j] in self.removal:
                    remove_index.append(j)
                    if routes[i][j + 1] in self.original_stations:
                        remove_index.append(j + 1)
            new_arr = np.delete(arr, remove_index)
            new_route = new_arr.tolist()
            new_routes.append(new_route)
        return new_routes

    def random_route_removal_RRR(self, routes):
        """
        This is the function to randomly remove a route
        """
        # reset the removal list
        self.reset_removal()

        # get the omega, which is the number of routes to be removed
        omega = ceil(uniform(self.routes_number_lower * len(routes), self.mr * len(routes)))
        # randomly choose omega routes and then remove them
        routes_removed = sample(routes, omega)

        # update the removal list of clients
        for route in routes_removed:
            for node in route:
                if node in self.clients:
                    self.removal.append(node)

        # remove the routes to be removed
        routes_copy = deepcopy(routes)
        for route in routes_removed:
            routes_copy.remove(route)

        return routes_copy

    def greedy_route_removal_GRR(self, routes):
        """
        This is the function to greedy remove a route
        """
        # reset the removal list
        self.reset_removal()

        # get the omega, which is the number of routes to be removed
        omega = ceil(uniform(self.routes_number_lower * len(routes), self.mr * len(routes)))

        # sort the routes according to the increasing order of the number of clients
        sorted_routes = sorted(routes, key=lambda route: sum(node in self.clients for node in route))
        routes_removed = sorted_routes[:omega]

        # update the removal list of clients
        for route in routes_removed:
            for node in route:
                if node in self.clients:
                    self.removal.append(node)

        return sorted_routes[omega:]
