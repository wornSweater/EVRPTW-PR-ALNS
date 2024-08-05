import gurobipy as gp
from gurobipy import GRB
from file_reader import get_parameters
from helper_function import Helper


def milp_model(file):
    # extract parameters from the instance file
    parameters = get_parameters(file, 2)

    clients = parameters["clients"]
    stations = parameters["stations"]
    all_nodes = parameters["all_nodes"]
    depot_start = parameters["depot_start"]
    depot_end = parameters["depot_end"]
    demand = parameters["demand"]
    ready_time = parameters["ready_time"]
    due_date = parameters["due_date"]
    service_time = parameters["service_time"]
    arcs = parameters["arcs"]
    times = parameters["times"]
    final_data = parameters["final_data"]
    original_stations = parameters["original_stations"]
    Q = parameters["Q"]
    C = parameters["C"]
    g = parameters["g"]
    h = parameters["h"]
    v = parameters["v"]

    # Create a new model
    model = gp.Model("mip")
    # not showing the output directly in the terminal


    # Create decision variables
    # Arcs decision variable

    # only arcs between different nodes, without from depot_start to depot_end
    X = {}
    for i in clients + stations + depot_start:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                X[i, j] = model.addVar(vtype=GRB.BINARY, name=f"x[{i},{j}]")

    # Arrival time decision variable
    t = {}
    for j in clients + stations + depot_start + depot_end:
        t[j] = model.addVar(lb=ready_time[j], ub=due_date[j], vtype=GRB.CONTINUOUS, name=f"t{j}")

    # Remaining cargo decision variable
    u = {}
    for j in clients + stations + depot_start + depot_end:
        if j == "D0":
            u[j] = model.addVar(lb=0, ub=C, vtype=GRB.CONTINUOUS, name=f"u{j}")
        else:
            u[j] = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f"u{j}")

    # Arrival and Departure energy decision variable
    # For depot_start, there is no arrival energy and for depot_end there is no departure energy level
    y = {}
    Y = {}
    for j in clients + stations + depot_start + depot_end:
        if j != "D0_end":
            Y[j] = model.addVar(lb=0, ub=Q, vtype=GRB.CONTINUOUS, name=f"Y{j}")
        if j != "D0":
            y[j] = model.addVar(lb=0, ub=Q, vtype=GRB.CONTINUOUS, name=f"y{j}")

    # Objective
    scalar = 2000
    objective_num = sum(X["D0", j] for j in clients + stations)
    objective_distance = sum(
        arcs[i, j] * X[i, j] for i in clients + stations + depot_start for j in clients + stations + depot_end
        if i != j and not (i == "D0" and j == "D0_end")
    )
    model.setObjective(objective_distance, GRB.MINIMIZE)

    # Constraints
    # The number limit to try if some of them do not work for a feasible solution
    ####################################################################
    # model.addConstr(sum(X["D0", j] for j in clients + stations) == 1)
    ####################################################################

    # The constraints to avoid the transfer from the same charging station directly to its dummy
    for i in stations:
        for j in stations:
            if arcs[i, j] == 0 and i != j:
                model.addConstr(X[i, j] == 0, name=f"no_inner_transfer{i, j}")

    # inflow and outflow equality

    # clients inflow and outflow
    for i in clients:
        model.addConstr(
            sum(X[i, j] for j in clients + stations + depot_end if i != j and not (i == "D0" and j == "D0_end")) == 1,
            name=f"con_CF{i}"
        )

    # stations inflow and outflow
    for i in stations:
        model.addConstr(
            sum(X[i, j] for j in clients + stations + depot_end if i != j and not (i == "D0" and j == "D0_end")) <= 1,
            name=f"con_CF{i}"
        )

    # inflow outflow equality
    for j in clients + stations:
        model.addConstr(
            sum(X[i, j] for i in clients + stations + depot_start if i != j and not (i == "D0" and j == "D0_end")) ==
            sum(X[j, i] for i in clients + stations + depot_end if i != j and not (i == "D0" and j == "D0_end")),
            name=f"con_In_Out{j}"
        )

    # Time
    # arrival time for clients
    for i in clients + depot_start:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                model.addConstr(
                    t[i] + (times[i, j] + service_time[i]) * X[i, j] - due_date["D0"] * (1 - X[i, j]) <= t[j],
                    name=f"con_time{i}-{j}")

    # arrival time for stations
    for i in stations:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                model.addConstr(
                    t[i] + times[i, j] * X[i, j] + g * (Y[i] - y[i]) - (due_date["D0"] + g * Q) * (1 - X[i, j]) <= t[j],
                    name=f"con_time{i}-{j}")

    # Remaining cargo
    for i in clients + stations + depot_start:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                model.addConstr(u[j] <= u[i] - demand[i] * X[i, j] + C * (1 - X[i, j]), name=f"con_cargo{i}-{j}")

    # Charge level
    # charge level on arrival at a client
    for i in clients:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                model.addConstr(y[j] <= y[i] - h * arcs[i, j] * X[i, j] + Q * (1 - X[i, j]), name=f"con_charge{i}-{j}")

    # charge level on arrival at a station
    for i in stations + depot_start:
        for j in clients + stations + depot_end:
            if i != j and not (i == "D0" and j == "D0_end"):
                model.addConstr(y[j] <= Y[i] - h * arcs[i, j] * X[i, j] + Q * (1 - X[i, j]), name=f"con_charge{i}-{j}")

    # Constraint for Y and y
    # -> depot_start not included
    for i in stations:
        model.addConstr(y[i] <= Y[i], name=f"con_AfterCharge{i}")

    # set the time limit for the model
    model.setParam('TimeLimit', 3600)

    model.optimize()

    # get the solving time
    solving_time = model.Runtime

    # if optimal solution, then extract the objective and optimal solution
    # create the data structures to get the number of vehicles as well as the routes
    helper = Helper(parameters)

    binary_arcs = {}
    for i in all_nodes:
        for j in all_nodes:
            binary_arcs[i, j] = 0

    objective_value = 0

    if model.status == GRB.OPTIMAL:
        objective_value = model.ObjVal
        for i in all_nodes:
            for j in all_nodes:
                if (i, j) in X:
                    if X[i, j].x > 0.5:
                        binary_arcs[i, j] = 1

    elif model.status == gp.GRB.Status.TIME_LIMIT:
        if model.SolCount > 0:
            objective_value = model.ObjVal
            for i in all_nodes:
                for j in all_nodes:
                    if (i, j) in X:
                        if X[i, j].x > 0.5:
                            binary_arcs[i, j] = 1
        else:
            objective_value = -1

    return objective_value, len(helper.get_routes_dict(binary_arcs)), solving_time
