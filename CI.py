from mip_check import MIPCheck
from helper_function import Helper
from SI import StationInsertion
import string


class CustomerInsertion:
    def __init__(self, parameters):
        self.parameters = parameters
        self.Q = self.parameters["Q"]
        self.depot_start = self.parameters["depot_start"]
        self.original_stations = self.parameters["original_stations"]
        self.clients = self.parameters["clients"]
        self.arcs = self.parameters["arcs"]
        self.h = self.parameters["h"]
        self.checker = MIPCheck(self.parameters)
        self.helper = Helper(self.parameters)
        self.SI = StationInsertion(self.parameters)

    def greedy_customer_insertion(self, routes, removal):
        """
        This is the function to repair the route by adding the customers back
        :param routes: a solution
        :param removal: the customers need to be inserted
        :return: a new feasible solution
        """
        # create the route index and initiate the current route, the first route in the routes
        route_index = 0
        index_limit = len(routes) - 1

        # if the removal is not empty, the iteration will not stop
        while removal:
            # initiate the current route
            current_route = routes[route_index]
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
                # if there is customer that can be inserted, we update the current route and un-change the index
                # remove the best client from the removal list
                routes[route_index] = new_route
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
                        if self.helper.cargo_check(new_route) and self.checker.time(
                                new_route) and not self.checker.energy(
                                new_route):
                            candidates.append(self.SI.supplement_station_insertion(new_route))
                # when the loop finish, we proceed with the situation of all new routes
                # if the candidates are not empty:
                if candidates:
                    # if there is one feasible
                    if any(self.helper.feasible_route(candidate) for candidate in candidates):
                        add_route = min(
                            candidates,
                            key=lambda candidate: self.helper.distance_one_route(
                                candidate) if self.helper.feasible_route(
                                candidate) else float("inf")
                        )
                        # if after repair some can be in, we update the route, removal list and un-change the index
                        routes[route_index] = add_route
                        for client in add_route:
                            if client in self.clients and client not in current_route:
                                removal.remove(client)
                    else:
                        # if the SI can not repair the route:
                        # if this is not the last route in the routes solution, we update to the next route
                        if route_index < index_limit:
                            route_index += 1

                        # if this is the last route, we add a new route and update the route index
                        elif route_index == index_limit:
                            routes.append(["D0", "D0_end"])
                            route_index += 1

                        # if this is even an extra route
                        else:
                            # we first test if the current route is ["D0", "D0_end"]
                            # if the current route is not ["D0", "D0_end"], we add the new one and update the index
                            if current_route != ["D0", "D0_end"]:
                                routes.append(["D0", "D0_end"])
                                route_index += 1
                            # if the current route is already ["D0", "D0_end"], we start the perfect repair
                            else:
                                routes[route_index] = self.SI.supplement_station_insertion(["D0", removal[0], "D0_end"])
                                removal.remove(removal[0])
                else:
                    # this is because the candidates are empty, since the route cannot meet time or cargo constraint
                    if route_index < index_limit:
                        route_index += 1
                    else:
                        routes.append(["D0", "D0_end"])
                        route_index += 1

        return routes


    def regret_customer_insertion_2(self, routes, removal, k=2):
        """
        This is the function to perform the regret customer insertion
        @param routes: the solution needed to be repaired
        @param removal: the list of customers needed to be added to the solution
        @param k: hyper which is the first and number k insertion of a customer on a specific route
        @return: another feasible solution
        """
        # create the route index and initiate the current route, the first route in the routes
        route_index = 0
        index_limit = len(routes) - 1

        # if the removal is not empty, the iteration will not stop
        while removal:
            # initiate the current route
            current_route = routes[route_index]

            # store all the feasible new route after customer insertion
            customers_dict = {}
            for client in removal:
                customer_routes = []
                for i in range(1, len(current_route)):
                    new_route = current_route[:i] + [client] + current_route[i:]
                    if self.helper.feasible_route(new_route):
                        customer_routes.append(new_route)
                if len(customer_routes) >= k:
                    sorted_routes = sorted(customer_routes, key=self.helper.distance_one_route)
                    customers_dict[client] = sorted_routes

            # test if the dict is empty
            if customers_dict:
                # if not empty, we can do the insertion
                best_customer = max(
                    customers_dict,
                    key=lambda customer: abs(self.helper.distance_one_route(
                        customers_dict[customer][k - 1]) - self.helper.distance_one_route(customers_dict[customer][0]))
                )

                # we insert the best customer, update the removal list
                routes[route_index] = customers_dict[best_customer][0]
                removal.remove(best_customer)
            else:
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
                    # if there is customer that can be inserted, we update the current route and un-change the index
                    # remove the best client from the removal list
                    routes[route_index] = new_route
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
                            if self.helper.cargo_check(new_route) and self.checker.time(
                                    new_route) and not self.checker.energy(
                                new_route):
                                candidates.append(self.SI.supplement_station_insertion(new_route))
                    # when the loop finish, we proceed with the situation of all new routes
                    # if the candidates are not empty:
                    if candidates:
                        # if there is one feasible
                        if any(self.helper.feasible_route(candidate) for candidate in candidates):
                            add_route = min(
                                candidates,
                                key=lambda candidate: self.helper.distance_one_route(
                                    candidate) if self.helper.feasible_route(
                                    candidate) else float("inf")
                            )
                            # if after repair some can be in, we update the route, removal list and un-change the index
                            routes[route_index] = add_route
                            for client in add_route:
                                if client in self.clients and client not in current_route:
                                    removal.remove(client)
                        else:
                            # if the SI can not repair the route:
                            # if this is not the last route in the routes solution, we update to the next route
                            if route_index < index_limit:
                                route_index += 1

                            # if this is the last route, we add a new route and update the route index
                            elif route_index == index_limit:
                                routes.append(["D0", "D0_end"])
                                route_index += 1

                            # if this is even an extra route
                            else:
                                # we first test if the current route is ["D0", "D0_end"]
                                # if the current route is not ["D0", "D0_end"], we add the new one and update the index
                                if current_route != ["D0", "D0_end"]:
                                    routes.append(["D0", "D0_end"])
                                    route_index += 1
                                # if the current route is already ["D0", "D0_end"], we start the perfect repair
                                else:
                                    routes[route_index] = self.SI.supplement_station_insertion(["D0", removal[0], "D0_end"])
                                    removal.remove(removal[0])
                    else:
                        # this is because the candidates are empty, since the route cannot meet time or cargo constraint
                        if route_index < index_limit:
                            route_index += 1
                        else:
                            routes.append(["D0", "D0_end"])
                            route_index += 1
        return routes

    def regret_customer_insertion_3(self, routes, removal, k=3):
        """
        This is the function to perform the regret customer insertion
        @param routes: the solution needed to be repaired
        @param removal: the list of customers needed to be added to the solution
        @param k: hyper which is the first and number k insertion of a customer on a specific route
        @return: another feasible solution
        """
        # create the route index and initiate the current route, the first route in the routes
        route_index = 0
        index_limit = len(routes) - 1

        # if the removal is not empty, the iteration will not stop
        while removal:
            # initiate the current route
            current_route = routes[route_index]

            # store all the feasible new route after customer insertion
            customers_dict = {}
            for client in removal:
                customer_routes = []
                for i in range(1, len(current_route)):
                    new_route = current_route[:i] + [client] + current_route[i:]
                    if self.helper.feasible_route(new_route):
                        customer_routes.append(new_route)
                if len(customer_routes) >= k:
                    sorted_routes = sorted(customer_routes, key=self.helper.distance_one_route)
                    customers_dict[client] = sorted_routes

            # test if the dict is empty
            if customers_dict:
                # if not empty, we can do the insertion
                best_customer = max(
                    customers_dict,
                    key=lambda customer: abs(self.helper.distance_one_route(
                        customers_dict[customer][k - 1]) - self.helper.distance_one_route(customers_dict[customer][0]))
                )

                # we insert the best customer, update the removal list
                routes[route_index] = customers_dict[best_customer][0]
                removal.remove(best_customer)
            else:
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
                    # if there is customer that can be inserted, we update the current route and un-change the index
                    # remove the best client from the removal list
                    routes[route_index] = new_route
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
                            if self.helper.cargo_check(new_route) and self.checker.time(
                                    new_route) and not self.checker.energy(
                                new_route):
                                candidates.append(self.SI.supplement_station_insertion(new_route))
                    # when the loop finish, we proceed with the situation of all new routes
                    # if the candidates are not empty:
                    if candidates:
                        # if there is one feasible
                        if any(self.helper.feasible_route(candidate) for candidate in candidates):
                            add_route = min(
                                candidates,
                                key=lambda candidate: self.helper.distance_one_route(
                                    candidate) if self.helper.feasible_route(
                                    candidate) else float("inf")
                            )
                            # if after repair some can be in, we update the route, removal list and un-change the index
                            routes[route_index] = add_route
                            for client in add_route:
                                if client in self.clients and client not in current_route:
                                    removal.remove(client)
                        else:
                            # if the SI can not repair the route:
                            # if this is not the last route in the routes solution, we update to the next route
                            if route_index < index_limit:
                                route_index += 1

                            # if this is the last route, we add a new route and update the route index
                            elif route_index == index_limit:
                                routes.append(["D0", "D0_end"])
                                route_index += 1

                            # if this is even an extra route
                            else:
                                # we first test if the current route is ["D0", "D0_end"]
                                # if the current route is not ["D0", "D0_end"], we add the new one and update the index
                                if current_route != ["D0", "D0_end"]:
                                    routes.append(["D0", "D0_end"])
                                    route_index += 1
                                # if the current route is already ["D0", "D0_end"], we start the perfect repair
                                else:
                                    routes[route_index] = self.SI.supplement_station_insertion(["D0", removal[0], "D0_end"])
                                    removal.remove(removal[0])
                    else:
                        # this is because the candidates are empty, since the route cannot meet time or cargo constraint
                        if route_index < index_limit:
                            route_index += 1
                        else:
                            routes.append(["D0", "D0_end"])
                            route_index += 1
        return routes
