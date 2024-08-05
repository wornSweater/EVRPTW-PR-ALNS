import gurobipy as gp
from gurobipy import GRB
import numpy as np
import random


class MIPCheck:
    def __init__(self, parameters):
        self.parameters = parameters
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
        self.times = self.parameters["normal_times"]
        # self.times = self.parameters["times"]
        self.C = self.parameters["C"]
        self.Q = self.parameters["Q"]
        self.g = self.parameters["g"]
        self.h = self.parameters["h"]
        self.v = self.parameters["v"]
        self.std = self.parameters["std"]
        self.mean = self.parameters["mean"]

    def update_times(self, p, n):
        new_times = self.parameters["times"]
        for i in self.all_nodes:
            for j in self.all_nodes:
                if i != j:
                    if random.random() < p:
                        stochastic = np.random.normal(0, n*self.std, 1)[0]
                        if new_times[i,j] + stochastic > 0:
                            new_times[i,j] = new_times[i,j] + stochastic
        self.times = new_times


    def time_energy(self, route) -> bool:
        """
        This is the function to check one route time and energy constraints feasibility
        :param route: the list of nodes, one route
        :return: true if the route can be made feasible and false otherwise
        """
        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the time and energy
        # time constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.stations:
                    model.addConstr(
                        t[index]+self.times[route[index], route[index+1]]+self.g*(Y[index]-y[index])<=t[index+1]
                    )
                else:
                    model.addConstr(
                        t[index]+self.times[route[index], route[index+1]]+self.service_time[route[index]]<=t[index+1]
                    )

        # energy constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.clients:
                    model.addConstr(y[index+1]<=y[index]-self.h*self.arcs[route[index], route[index+1]])
                else:
                    model.addConstr(y[index+1]<=Y[index]-self.h*self.arcs[route[index], route[index+1]])

        # departure and arrival energy constraint
        for i in range(len(route)):
            if route[i] in self.stations + self.depot_start + self.depot_end:
                model.addConstr(y[i] <= Y[i])

        # solve the model and optimize
        model.optimize()

        if model.status == GRB.OPTIMAL:
            return True
        if model.status == GRB.INFEASIBLE or model.status == GRB.UNBOUNDED:
            return False
        return False

    def time(self, route):
        """
        This is the function to check if the time constraint can be satisfied
        :param route: list of nodes which is a route
        :return: true if time allowed and false otherwise
        """
        # only care about the time, no energy, so only consider the energy state change at stations

        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the time and energy
        # time constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.stations:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] <= t[
                            index + 1]
                    )
                else:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.service_time[route[index]] <= t[
                            index + 1]
                    )

        # solve the model and optimize
        model.optimize()

        if model.status == GRB.OPTIMAL:
            return True
        if model.status == GRB.INFEASIBLE or model.status == GRB.UNBOUNDED:
            return False
        return False

    def energy(self, route):
        """
        This is the function to check only the energy constraint
        :param route: lsit of nodes
        :return: true if energy constraint can be satisfied and false otherwise
        """
        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the energy

        # energy constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.clients:
                    model.addConstr(y[index + 1] <= y[index] - self.h * self.arcs[route[index], route[index + 1]])
                else:
                    model.addConstr(y[index + 1] <= Y[index] - self.h * self.arcs[route[index], route[index + 1]])

        # departure and arrival energy constraint
        for i in range(len(route)):
            if route[i] in self.stations + self.depot_start + self.depot_end:
                model.addConstr(y[i] <= Y[i])

        # solve the model and optimize
        model.optimize()

        if model.status == GRB.OPTIMAL:
            return True
        if model.status == GRB.INFEASIBLE or model.status == GRB.UNBOUNDED:
            return False
        return False

    # notice: when using this function, the route must be feasible
    def time_extractor(self, route):
        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the time and energy
        # time constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.stations:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.g * (Y[index] - y[index]) <= t[
                            index + 1]
                    )
                else:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.service_time[route[index]] <= t[
                            index + 1]
                    )

        # energy constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.clients:
                    model.addConstr(y[index + 1] <= y[index] - self.h * self.arcs[route[index], route[index + 1]])
                else:
                    model.addConstr(y[index + 1] <= Y[index] - self.h * self.arcs[route[index], route[index + 1]])

        # departure and arrival energy constraint
        for i in range(len(route)):
            if route[i] in self.stations + self.depot_start + self.depot_end:
                model.addConstr(y[i] <= Y[i])

        # solve the model and optimize
        model.optimize()

        if model.status == GRB.OPTIMAL:
            return [time.x for time in t]
        raise "This route is not feasible"

    def energy_extractor(self, route):
        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the time and energy
        # time constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.stations:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.g * (Y[index] - y[index]) <= t[
                            index + 1]
                    )
                else:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.service_time[route[index]] <= t[
                            index + 1]
                    )

        # energy constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.clients:
                    model.addConstr(y[index + 1] <= y[index] - self.h * self.arcs[route[index], route[index + 1]])
                else:
                    model.addConstr(y[index + 1] <= Y[index] - self.h * self.arcs[route[index], route[index + 1]])

        # departure and arrival energy constraint
        for i in range(len(route)):
            if route[i] in self.stations + self.depot_start + self.depot_end:
                model.addConstr(y[i] <= Y[i])

        # solve the model and optimize
        model.optimize()

        # here for clients the departure energy should use the arrival energy
        # departure energy is the same as arrival energy
        # and for clients the departure energy decision variable is not included in the constraints, so missing value
        if model.status == GRB.OPTIMAL:
            return [arrival_energy.x for arrival_energy in y]
        raise "This route is not feasible"

    def energy_extractor_departure(self, route):
        # create the model first
        model = gp.Model("route_check_time_energy")

        # set the output flag as 0 to avoid outcome showing
        model.setParam('OutputFlag', 0)

        # create the decision variables, times and energy states
        t = []
        y = []
        Y = []
        for i in range(len(route)):
            t.append(model.addVar(lb=self.ready_time[route[i]], ub=self.due_date[route[i]], vtype=GRB.CONTINUOUS))
            Y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))
            y.append(model.addVar(lb=0, ub=self.Q, vtype=GRB.CONTINUOUS))

        # create objective function, for checking feasibility, objective is set as 0
        model.setObjective(0, GRB.MINIMIZE)

        # create constraints to check the time and energy
        # time constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.stations:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.g * (Y[index] - y[index]) <= t[
                            index + 1]
                    )
                else:
                    model.addConstr(
                        t[index] + self.times[route[index], route[index + 1]] + self.service_time[route[index]] <= t[
                            index + 1]
                    )

        # energy constraints
        for index in range(len(route)):
            i = route[index]
            if i == "D0_end":
                break
            else:
                if i in self.clients:
                    model.addConstr(y[index + 1] <= y[index] - self.h * self.arcs[route[index], route[index + 1]])
                else:
                    model.addConstr(y[index + 1] <= Y[index] - self.h * self.arcs[route[index], route[index + 1]])

        # departure and arrival energy constraint
        for i in range(len(route)):
            if route[i] in self.stations + self.depot_start + self.depot_end:
                model.addConstr(y[i] <= Y[i])

        # solve the model and optimize
        model.optimize()

        # here for clients the departure energy should use the arrival energy
        # departure energy is the same as arrival energy
        # and for clients the departure energy decision variable is not included in the constraints, so missing value
        if model.status == GRB.OPTIMAL:
            return [arrival_energy.x for arrival_energy in Y]
        raise "This route is not feasible"
