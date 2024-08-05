import string
from EVRPTW_PR_ALNS.mip_check import MIPCheck


class Helper:
    def __init__(self, parameters):
        """
        Take the parameter to initiate a helper instance
        :param parameters: parameter dict of a graph instance
        """
        self.parameters = parameters
        self.checker = MIPCheck(self.parameters)
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

    def get_routes_dict(self, incidence_dict):
        """
        This the function to get all routes from the solution, a list contains lists, two-dimensional arrays
        :param incidence_dict: binary arcs with 0 and 1
        :return: list of routes
        """
        routes = []
        for i in self.all_nodes:
            if incidence_dict["D0", i] == 1:
                routes.append(self.get_route_dict(incidence_dict, ["D0"], i))
        return routes

    def get_route_dict(self, incidence_dict, list_of_nodes, node: string):
        """
        This is a recursive function to get just one route from the incidence dict matrix
        :param incidence_dict: binary arcs with 0 and 1
        :param list_of_nodes: the route list
        :param node: the current node pointer
        :return: list, representing one route
        """
        list_of_nodes.append(node)
        if node == "D0_end":
            return list_of_nodes

        for i in self.all_nodes:
            if incidence_dict[node, i] == 1:
                return self.get_route_dict(incidence_dict, list_of_nodes, i)

    def vehicle_number_dict(self, incidence_dict):
        """
        This is the function to get the number of vehicles from incidence dict matrix
        :param incidence_dict: binary arcs with 0 and 1
        :return: int, the number of vehicles in the solution
        """
        return sum(incidence_dict["D0", j] for j in self.all_nodes)

    def vehicle_number_list(self, routes):
        """
        This is the function to get the number of vehicles from list of all routes
        :param routes: the list of routes
        :return: the length of the list which is the number of vehicles
        """
        return len(routes)

    def total_distance_dict(self, incidence_dict):
        """
        This is the function to get the total distance from the incidence dict matrix
        :param incidence_dict: the binary arcs with 0 and 1
        :return: the total traveled distance
        """
        return sum(self.arcs[i, j] * incidence_dict[i, j] for i in self.all_nodes for j in self.all_nodes)

    def total_distance_list(self, routes):
        """
        This is the function to get total distance form all the routes
        :param routes: list of all routes
        :return: total distance of the graph
        """
        return sum(self.distance_one_route(route) for route in routes)

    def distance_one_route(self, route):
        """
        This is the function to get the total distance of a route
        :param route: list of nodes
        :return: distance of a route
        """
        total_distance = 0
        for i in range(len(route)):
            if route[i] == "D0_end":
                break
            else:
                total_distance += self.arcs[route[i], route[i + 1]]
        return total_distance

    def cargo_check(self, route):
        """
        This is the function to check the cargo feasibility for a route
        :param route: one route of one vehicle
        :return: if the cargo on this route is feasible
        """
        return sum(self.demand[i] for i in route) <= self.C

    def depot_check(self, route):
        """
        This is the function to check if for a route, the start is D0 and the end is D0_end
        :param route: list of nodes of a route
        :return: true if yes and false otherwise
        """
        return route[0] == "D0" and route[-1] == "D0_end" and route.count("D0") == 1 and route.count("D0_end") == 1

    def feasible_route(self, route):
        """
        This is the function to check if the route is completely feasible
        :param route: list of nodes
        :return: true if route is feasible and false otherwise
        """
        return self.checker.time_energy(route) and self.cargo_check(route) and self.depot_check(route)

    def feasible(self, routes):
        return all(self.feasible_route(route) for route in routes)
