from mip_check import MIPCheck
from helper_function import Helper


class StationInsertion:
    def __init__(self, parameters):
        """
        Collection of station insertion _algorithms
        :param parameters: parameters of a graph instance
        """
        self.parameters = parameters
        self.Q = self.parameters["Q"]
        self.depot_start = self.parameters["depot_start"]
        self.original_stations = self.parameters["original_stations"]
        self.clients = self.parameters["clients"]
        self.arcs = self.parameters["arcs"]
        self.h = self.parameters["h"]
        self.checker = MIPCheck(self.parameters)
        self.helper = Helper(self.parameters)

    # find the first negative customer, backward until reaches a station or depot_start
    def greedy_station_insertion(self, route):
        """
        This is the function to repair the route using station insertion
        :param route: The route needs to be performed station insertion
        :return: return feasible or the argument original infeasible one
        """
        # track the arrival energy
        # assume arrival energy at depot_start and stations is never negative, skip to negative clients
        # assume arrival energy at depot_start and station is Q full
        # this method must return a feasible solution (not better, but must be feasible) under the assumption
        # that there is no situation of consecutive stations, and if only one station is removed

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        for i in range(len(route)):
            if route[i] in self.depot_start + self.original_stations:
                arrival_energy = self.Q
            else:
                arrival_energy -= self.h * self.arcs[route[i - 1], route[i]]

            # find the first negative energy level of a customer
            if arrival_energy < 0:
                # start the insertion process
                # track the current arc and prepare to track all arcs before a station or depot_start
                for k in range(i):
                    # check if in this arc the predecessor is depot_start or station, if yes return infeasible route
                    # in theory this will never happen because of assumption one station is sufficient to any
                    # will never happen that after one station or depot_station, the customer arrival energy is negative
                    if route[i - k - 1] in self.depot_start + self.original_stations:
                        return route
                    # else we get all possible station insertion at this arc and see whether there is feasible
                    else:
                        # here we don't directly use min since if there is no feasible, the min will give warning
                        candidates = []
                        for station in self.original_stations:
                            candidates.append(route[:i - k] + [station] + route[i - k:])

                        # check whether there is feasible, if there is, find the best insertion
                        if any(self.checker.time_energy(candidate) for candidate in candidates):
                            new_route = min(
                                candidates, key=lambda candidate: self.helper.distance_one_route(
                                    candidate) if self.checker.time_energy(candidate) else float("inf")
                            )
                            return new_route
                        # if there is no feasible station insertion at this arc, continue to the previous arc
                        else:
                            continue
                # if we find the first negative, no need to continue to the next negative, we break the loop
                break
        return route

    # find the first negative customer, compare two arcs, if neither feasible use GSI / GSI-sn instead
    def greedy_station_insertion_comparison(self, route):
        """
        Compare the arc leading to the negative customer and the previous arc
        :param route: the list of nodes
        :return: the new route after repair or the argument if infeasible
        """
        # define the arrival and energy, the initial is Q
        # same as the GSI algorithm above

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        for i in range(len(route)):
            if route[i] in self.depot_start + self.original_stations:
                arrival_energy = self.Q
            else:
                arrival_energy -= self.h * self.arcs[route[i - 1], route[i]]

            # find the first negative energy level of a customer
            if arrival_energy < 0:
                # start the insertion process
                # check first the index and the three nodes
                # there should be three nodes, the current one and previous two, so the index should be greater than 2
                # since we insert station into the two arcs, the three nodes should be all clients
                # or there should not be any depot_start or stations in the three nodes

                # if we cannot compare because impossible to insert in either arc, we use GSI above
                if not (i >= 2 and route[i - 1] in self.clients and route[i - 2] in self.clients):
                    return self.greedy_station_insertion(route)
                # else, we find the two minimum at the two arcs
                else:
                    insertion1 = min(
                        self.original_stations,
                        key=lambda station: self.arcs[station, route[i - 1]] + self.arcs[
                            station, route[i]] -
                                            self.arcs[route[i - 1], route[i]]
                    )
                    insertion2 = min(
                        self.original_stations,
                        key=lambda station: self.arcs[station, route[i - 2]] + self.arcs[
                            station, route[i - 1]] -
                                            self.arcs[route[i - 2], route[i - 1]]
                    )
                    new_route1 = route[:i] + [insertion1] + route[i:]
                    new_route2 = route[:i - 1] + [insertion2] + route[i - 1:]

                    # compare and then check the feasibility
                    # if both feasible, find the less one
                    if self.checker.time_energy(new_route1) and self.checker.time_energy(new_route2):
                        if self.helper.distance_one_route(new_route1) < self.helper.distance_one_route(new_route2):
                            return new_route1
                        else:
                            return new_route2
                    # if neither feasible, use GSI
                    elif not self.checker.time_energy(new_route1) and not self.checker.time_energy(new_route2):
                        return self.greedy_station_insertion_sn(route)
                    # if only one is feasible, return that one
                    else:
                        if self.checker.time_energy(new_route1):
                            return new_route1
                        else:
                            return new_route2
                # break the for loop if we find the first negative
                break
        return route

    # find the first negative node, compare all arcs, if infeasible use GSI / GSI-sn instead
    def greedy_station_insertion_comparison_all(self, route):
        """
        General and improved form of greedy_station_insertion_comparison
        """

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        departure_energy = self.Q

        for i in range(len(route)):
            # at this index the arrival is first to update
            if route[i] == "D0":
                arrival_energy = self.Q
            else:
                arrival_energy = departure_energy - self.h * self.arcs[route[i - 1], route[i]]

            if route[i] in self.depot_start + self.original_stations:
                departure_energy = self.Q
            else:
                departure_energy = arrival_energy

            # find the first negative energy level of a node, could be customer, could be station
            if arrival_energy < 0:
                # start the insertion process
                # find all minimals (but not necessarily feasible of each arc)
                candidates = []
                for k in range(i):
                    # for each arc, we find the min on this arc, without making sure this is the feasible one
                    best_insertion = min(
                        self.original_stations,
                        key=lambda insertion: self.arcs[route[i-k], insertion] + self.arcs[route[i-k-1], insertion] -
                                              self.arcs[route[i-k], route[i-k-1]]
                    )
                    candidates.append(route[:i-k] + [best_insertion] + route[i-k:])
                if any(self.helper.feasible_route(candidate) for candidate in candidates):
                    return min(candidates, key=lambda candidate: self.helper.distance_one_route(candidate))
                else:
                    return self.greedy_station_insertion_sn(route)
                # if we find the first negative, no need to continue to the next negative, we break the loop
                break
        return route

    # find the first negative customer, backwards until reaches a station or depot_start, find the min feasible
    def best_station_insertion(self, route):
        """
        Strong version of greedy insertion algorithm
        :param route: a list of nodes
        :return: return the repaired route or argument itself if infeasible
        """
        # define the arrival and energy, the initial is Q
        # same as the GSI
        # there is also an assumption here, all search arcs are also before any station and depot_start
        # since we also assume one station or depot_start with full energy can go to any customer

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        for i in range(len(route)):
            if route[i] in self.depot_start + self.original_stations:
                arrival_energy = self.Q
            else:
                arrival_energy -= self.h * self.arcs[route[i - 1], route[i]]

            # find the first negative energy level of a customer
            if arrival_energy < 0:
                # create a list to store all the feasible new routes
                candidates = []
                # start the insertion process
                # this is for traverse all arcs
                for k in range(i):
                    # from the definition above, the index must be larger or equal to 1, no need to check
                    # we traverse the arcs until we reach a station or the depot_start
                    if route[i - k - 1] in self.depot_start + self.original_stations:
                        break
                    # else we get the min feasible distance on this arc
                    else:
                        # create the list to contain all routes after station insertion
                        all_feasible = []
                        for station in self.original_stations:
                            all_feasible.append(route[:i - k] + [station] + route[i - k:])
                        # check if there is feasible or not
                        # if there is feasible, we find the min
                        if any(self.checker.time_energy(new_route) for new_route in all_feasible):
                            candidate = min(
                                all_feasible, key=lambda candidate: self.helper.distance_one_route(
                                    candidate) if self.checker.time_energy(candidate) else float("inf")
                            )
                            candidates.append(candidate)
                        # if there is no feasible, we continue to the next arc
                        else:
                            continue

                # after traversing all arcs backwards before any station or the depot_start
                # test if the candidates are empty
                if candidates:
                    return min(candidates, key=lambda candidate: self.helper.distance_one_route(candidate))
                # since we find the first negative, no need to continue
                break
        return route

    # find the first negative node, backwards until the start, more general way to get a feasible and include more
    def greedy_station_insertion_sn(self, route):
        """
        This is the function to repair the route using station insertion finding the first negative including stations
        :param route: The route needs to be performed station insertion
        :return: return feasible or the argument original infeasible one
        """

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        departure_energy = self.Q

        for i in range(len(route)):
            # at this index the arrival is first to update
            if route[i] == "D0":
                arrival_energy = self.Q
            else:
                arrival_energy = departure_energy - self.h * self.arcs[route[i - 1], route[i]]

            if route[i] in self.depot_start + self.original_stations:
                departure_energy = self.Q
            else:
                departure_energy = arrival_energy

            # find the first negative energy level of a node, could be customer, could be station
            if arrival_energy < 0:
                # start the insertion process
                # track the current arc and prepare to track all arcs before a station or depot_start
                for k in range(i):
                    candidates = []
                    for station in self.original_stations:
                        candidates.append(route[:i - k] + [station] + route[i - k:])

                    # check whether there is feasible, if there is, find the best insertion
                    if any(self.checker.time_energy(candidate) for candidate in candidates):
                        new_route = min(
                            candidates, key=lambda candidate: self.helper.distance_one_route(
                                candidate) if self.checker.time_energy(candidate) else float("inf")
                        )
                        return new_route
                    # if there is no feasible station insertion at this arc, continue to the previous arc
                    else:
                        continue
                # if we find the first negative, no need to continue to the next negative, we break the loop
                break
        return route

    # find the first negative node, backwards until the start, find the min feasible throughout all arcs searched
    def best_station_insertion_sn(self, route):
        """
        This is the modified version of the best station insertion, find the first negative and backwards until start
        :param route: list of nodes, to be repaired route
        :return: updated route if feasible otherwise the original infeasible route
        """

        if self.helper.feasible_route(route):
            return route

        arrival_energy = self.Q
        departure_energy = self.Q

        for i in range(len(route)):
            # at this index the arrival is first to update
            if route[i] == "D0":
                arrival_energy = self.Q
            else:
                arrival_energy = departure_energy - self.h * self.arcs[route[i - 1], route[i]]

            if route[i] in self.depot_start + self.original_stations:
                departure_energy = self.Q
            else:
                departure_energy = arrival_energy

            # find the first negative energy level of a node, could be customer, could be station
            if arrival_energy < 0:
                # create a list to store all the feasible new routes
                candidates = []
                # start the insertion process
                # this is for traverse all arcs
                for k in range(i):

                    # create the list to contain all routes after station insertion
                    all_feasible = []
                    for station in self.original_stations:
                        all_feasible.append(route[:i - k] + [station] + route[i - k:])
                    # check if there is feasible or not
                    # if there is feasible, we find the min
                    if any(self.checker.time_energy(new_route) for new_route in all_feasible):
                        candidate = min(
                            all_feasible, key=lambda candidate: self.helper.distance_one_route(
                                candidate) if self.checker.time_energy(candidate) else float("inf")
                        )
                        candidates.append(candidate)
                    # if there is no feasible, we continue to the next arc
                    else:
                        continue

                # after traversing all arcs backwards before any station or the depot_start
                # test if the candidates are empty
                if candidates:
                    return min(candidates, key=lambda candidate: self.helper.distance_one_route(candidate))
                # since we find the first negative, no need to continue
                break
            return route

    def supplement_station_insertion(self, route):
        """
        This is a station insertion function to perfectly repair a route
        :param route: a list of nodes
        :return: the perfectly repaired route (highly possible to be feasible)
        """
        # set a counter and threshold to avoid infinite loop
        original_route = route
        threshold = 5
        counter = 0
        # start the while loop, as long as the route is not feasible, continue to update
        while not self.helper.feasible_route(route):
            counter += 1
            if counter >= threshold:
                return original_route
            # assign the route to the algorithm to process, return another route
            route = self.greedy_station_insertion_sn(route)
            # if the updated route is feasible, then we return the feasible route
            if self.helper.feasible_route(route):
                return route
            # if the route is the same, meaning the algorithm can not find a feasible one, we add station manually
            else:
                arrival_energy = self.Q
                departure_energy = self.Q

                route_copy = route[:]
                for i in range(len(route_copy)):
                    # at this index the arrival is first to update
                    if route[i] == "D0":
                        arrival_energy = self.Q
                    else:
                        arrival_energy = departure_energy - self.h * self.arcs[route[i - 1], route[i]]

                    if route[i] in self.depot_start + self.original_stations:
                        departure_energy = self.Q
                    else:
                        departure_energy = arrival_energy

                    if arrival_energy < 0:
                        # if the insertion location is right after depot_start or before the depot_end
                        if route_copy[i] == "D0_end" or route_copy[i-1] == "D0":
                            stations_search = self.original_stations[:]
                            if "S0" in self.original_stations:
                                stations_search.remove("S0")
                            insertion = min(
                                stations_search,
                                key=lambda station: self.arcs[route[i], station] + self.arcs[route[i - 1], station] -
                                                    self.arcs[route[i], route[i - 1]]
                            )
                            route = route[:i] + [insertion] + route[i:]
                        else:
                            insertion = min(
                                self.original_stations,
                                key=lambda station: self.arcs[route[i], station] + self.arcs[route[i - 1], station] -
                                                    self.arcs[route[i], route[i - 1]]
                            )
                            route = route[:i] + [insertion] + route[i:]
                        # we find the first negative, so break the loop
                        break
        return original_route


