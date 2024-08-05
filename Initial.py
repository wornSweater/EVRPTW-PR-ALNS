import string
from mip_check import MIPCheck
from _algorithms.SI import StationInsertion
from helper_function import Helper


class Heuristic:
    def __init__(self, parameters):
        """
        Take the parameter to initiate a helper instance
        :param parameters: parameter dict of a graph instance
        """
        self.parameters = parameters
        self.checker = MIPCheck(self.parameters)
        self.SI = StationInsertion(self.parameters)
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

    def initial_solution(self):
        """
        Function to get the initial feasible solution using heuristic combined with Greedy Station Insertion
        :return: list of routes for an instance
        """
        # create routes containing all feasible sub-route
        routes = []

        # create the first route and add to the routes
        route = ["D0", "D0_end"]
        routes.append(route)

        # create a route and add the nearest to the new route
        removal = self.clients[:]

        # if the removal is not empty, the iteration will not stop
        while removal:
            # define the current route as the last route in the routes list
            # hence now we are working with only one route
            current_route = routes[-1]
            # calculate the best customer with time and cargo constraints
            # define the temporary pointers
            best_insertion: string
            index_insertion: int
            min_distance = 999999
            distance_record = min_distance

            # for this part, we must find the smallest feasible if there is any
            for client in removal:
                for i in range(1, len(current_route)):
                    # calculate the difference, trying to find the best one
                    difference = self.arcs[current_route[i], client] + self.arcs[current_route[i - 1], client] - \
                                 self.arcs[
                                     current_route[i], current_route[i - 1]]
                    if difference < min_distance:
                        # find a smaller one, but check the feasibility first
                        # if feasible, we update the recorders
                        if self.helper.feasible_route(current_route[:i] + [client] + current_route[i:]):
                            min_distance = difference
                            best_insertion = client
                            index_insertion = i

            # after this smaller search, check if the recorder has updated
            # if yes, then we find a smaller feasible customer that can be added to the route and update the current
            if min_distance < distance_record:
                new_route = current_route[:index_insertion] + [best_insertion] + current_route[index_insertion:]
                routes[-1] = new_route
                removal.remove(best_insertion)
            # if after the search, there is no customer can be added to the route
            # we loop again to find the customer with a station
            else:
                # create the candidates and then compare the total distance if feasible
                candidates = []
                for client in removal:
                    for i in range(1, len(current_route)):
                        new_route = current_route[:i] + [client] + current_route[i:]
                        # keep new route with time and cargo constraint, and use greedy station insertion to repair
                        if self.helper.cargo_check(new_route) and self.checker.time(new_route) and not self.checker.energy(
                                new_route):
                            candidates.append(self.SI.greedy_station_insertion_sn(new_route))
                # when the loop finish, we proceed with the situation of all new routes
                # if the candidates are not empty:
                if candidates:
                    # if there is one feasible
                    if any(self.helper.feasible_route(candidate) for candidate in candidates):
                        add_route = min(
                            candidates,
                            key=lambda candidate: self.helper.distance_one_route(candidate) if self.helper.feasible_route(
                                candidate) else float("inf")
                        )
                        routes[-1] = add_route
                        for client in add_route:
                            if client in self.clients and client not in current_route:
                                removal.remove(client)
                    else:
                        if current_route != ["D0", "D0_end"]:
                            routes.append(["D0", "D0_end"])

                        # if the new route construction is already in, meaning the infinite loop is on
                        else:
                            routes[-1] = self.SI.supplement_station_insertion(["D0", removal[0], "D0_end"])
                            removal.remove(removal[0])
                else:
                    # this is because the candidates are empty, since the route cannot meet time or cargo constraint
                    routes.append(["D0", "D0_end"])
        return routes
