#2025-02-23
#Karolis Rakutis ITnt24
#Justinas Vijeikis ITnt24

from geopy.distance import geodesic
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


locations = [
    (54.8899471, 23.8448639),  # KAUNAS
    (54.7005727, 25.0882306),  # UTENA
    (55.501502, 25.5670458),   # VILNIUS
    (55.7047078, 20.9927754),  # KLAIPĖDA
    (55.9062468, 23.245972),   # ŠIAULIAI
    (55.736003, 24.359206),    # PANEVĖŽYS
    (54.399231, 24.049386),    # ALYTUS
    (55.249107, 26.161332),    # ZARASAI
    (56.315982, 22.326637),    # MAŽEIKIAI
    (55.505981, 22.197546),    # TELŠIAI
    (55.251839, 23.773809),    # KĖDAINIAI
    (55.065251, 24.281537),    # JONAVA
    (55.921806, 21.067669),    # PALANGA
    (54.479521, 23.928100),    # MARIJAMPOLĖ
    (54.081524, 23.547548),    # DRUSKININKAI
    (55.522776, 24.770364),    # ANYKŠČIAI
    (56.000988, 24.135465),    # PASVALYS
    (55.733496, 23.113635),    # RADVILIŠKIS
    (55.385827, 21.568837),    # KRETINGA
    (55.265785, 24.761039)     # UKMERGĖ
]

# SUGENERUOJAMA KOORDINAČIŲ MATRICA. I FUNKCIJA PADUODAM KOORDINACIU TASKUS KAIP
# AUTOMOBILIŲ SKAIČIŲ IR PRADINĮ TAŠKĄ (NURODOME JĮ KAIP POZICIJĄ KOORDINAČIŲ MASYVE)
def distance_matrix_generator(coords, num_vehicles=10, depot=0):
    n = len(coords)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            if i == j:
                dist = 0  # tikrinama ar tai yra vienas ir tas pats taskas
            else:
                dist = geodesic(coords[i], coords[j]).kilometers  # apskaičiuojamas atstumas tarp taškų
            row.append(int(round(dist)))  # pasiverciam i INT ir pridedam masyvą i eilutes masyvą
        matrix.append(row) #i matrica pridedamos eilutes su atstumais nuo tasko iki kitu tasku

    data = {
    "distance_matrix": matrix,
    "num_vehicles": num_vehicles,
    "depot": depot
    }

    return data
#REZULTATU ISVEDIMAS I EKRANA
def print_solution(data, manager, routing, solution):
    max_route_distance = 0
    for vehicle_id in range(data["num_vehicles"]):
        if not routing.IsVehicleUsed(solution, vehicle_id):
            continue
        index = routing.Start(vehicle_id)
        plan_output = f"Marsrutas automobiliui {vehicle_id+1}:\n"
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += f" {manager.IndexToNode(index)} -> "
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id
            )
        plan_output += f"{manager.IndexToNode(index)}\n"
        plan_output += f"Maršruto ilgis: {route_distance}km\n"
        print(plan_output)
        max_route_distance = max(route_distance, max_route_distance)
    print(f"Ilgiausias maršrutas: {max_route_distance}km")



def main():
    #kvieciame koordinaciu generavimo metodas ir jo rezultatas priskiriamas DATA kintamajam
    data = distance_matrix_generator(locations)

    #paduodami OR-tools bibliotekai esamos problemos duomenys t.y atstumu matrica tarp taksu, pradinis taskas ir automobiliu skaicius
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # PALEIDZIAMAS OPTIMIZAVIMO MODELIS KURIS NAUDOS IS MANAGER GAUTA INFORMACIJA MARSRUTAMS APSKAICIUOTI
    routing = pywrapcp.RoutingModel(manager)

    # apskaiciuojamas ilgiai tarp tasku, pateikiamas lanko ilgis.
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Apibreziamas lanku ilgis
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    dimension_name = "Atstumas"
    routing.AddDimension(
        transit_callback_index,
        3,  # maksimalus neidarbintu automobiliu skaicius
        500,  # maksimalus keliones atstumas
        True,  # pradine reiksme butinai nuo 0
        dimension_name,
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(150) #maksimalus atstumas tarp marsrutu????

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # SPRENDIMAS
    solution = routing.SolveWithParameters(search_parameters)

    # SPRENDIMO ISVEDIMAS I TERIMANALA
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found !")


if __name__ == "__main__":
    main()