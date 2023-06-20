def create_mtsp_input_file(filename, drone_coords, node_coords):
    with open(filename, 'w') as file:
        # Write the problem information
        file.write("NAME: test_drones_MTSP\n")
        file.write("COMMENT: Not universal, adapted for our project\n")
        file.write("TYPE: MTSP\n")
        file.write("DIMENSION: {}\n".format(len(drone_coords) + len(node_coords)))
        file.write("EDGE_WEIGHT_TYPE: EUC_2D\n")

        # Write the DRONE_COORD_SECTION
        file.write("DRONE_COORD_SECTION\n")
        for i, coord in enumerate(drone_coords, start=1):
            file.write("{} {} {} {}\n".format(i, coord[0], coord[1], coord[2]))

        # Write the NODE_COORD_SECTION
        file.write("NODE_COORD_SECTION\n")
        for i, coord in enumerate(node_coords, start=1):
            file.write("{} {} {}\n".format(i, coord[0], coord[1]))

        # Write the EOF marker
        file.write("EOF\n")

# Example usage
drone_coords = [(0, 0, 100), (0, 0, 100)]
node_coords = [(0, 13), (0, 26), (0, 27), (0, 39), (2, 0), (5, 13), (5, 19), (5, 25), (5, 31), (5, 37)]
create_mtsp_input_file("test_drones_MTSP.tsp", drone_coords, node_coords)



