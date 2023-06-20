import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math



#%% Parameters
Drones = {
    1:(0,0,10000),
    2:(0,0,10000),
    3:(0,0,10000),
    4:(0,0,10000),

}
Nodes = {
    1:(0,800),
    2:(0,900),
    3:(0,1000),
    4:(0,1100),
    5:(100,900),
    6:(100,1000),
    7:(100,1100),
    8:(-100,900),
    9:(-100,1000),
    10:(-100,1100),
    11:(200,900),
    12:(200,1000),
    13:(200,1100),
    14:(-200,900),
    15:(-200,1000),
    16:(-200,1100)
}
mode = 'car'
freespeed = 45 #m/s
capacity = 2000
permlanes = 1

begin_time = '08:00'
end_time = '12:00'
duration_of_surveillancy = '00:01'
duration_of_refill = '00:02'
def distance(a,b):
    "return the distance between a & b (the upper integer value) "
    return math.ceil(((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5)


from MTSP_inputfile import create_mtsp_input_file
#%% Writting of matsim_points.tsp ##################################################################################
with open('matsim_points.tsp', 'w') as out_file:
    out_file.write('NAME : matsim_points\n')
    out_file.write('COMMENT : Points from MATSim\n')
    out_file.write('TYPE : TSP\n')
    out_file.write('DIMENSION : {}\n'.format(len(Nodes)))
    out_file.write('EDGE_WEIGHT_TYPE : EUC_2D\n')
    out_file.write('NODE_COORD_SECTION\n')
    for city, coords in Nodes.items():
        out_file.write('{} {} {}\n'.format(city, coords[0], coords[1]))
    out_file.write('EOF\n')

create_mtsp_input_file("test_drones_MTSP.tsp", Drones.values(), Nodes.values())


from ALNS_MTSP import compute_MTSP
#%% order from surveillancy

routes = compute_MTSP()
routes = [[[node + 1 for node in route] for route in drone_route] for drone_route in routes] #accorded to Nodes
print(routes)

#%% Writting of network.xml #######################################################################################
""" Create the root element <network>"""
network = ET.Element('network')
network.set('name', 'drone network')

# Create the <nodes> element
nodes = ET.SubElement(network, 'nodes')

# Create the <links> element
links = ET.SubElement(network, 'links')
links.set('capperiod', '01:00:00')

# Create the XML tree
tree = ET.ElementTree(network)

#Nodes
for i in range(1,len(Drones)+1):
    # Create a new drone element
    new_node = ET.SubElement(nodes, 'node')
    new_node.set('id', str(i))
    new_node.set('x', str(Drones[i][0]))
    new_node.set('y', str(Drones[i][1]))
for i in range(1,len(Nodes)+1):
    # Create a new node element
    new_node = ET.SubElement(nodes, 'node')
    new_node.set('id', str(i+len(Drones)))
    new_node.set('x', str(Nodes[i][0]))
    new_node.set('y', str(Nodes[i][1]))

#Link
for drone in range(len(Drones)):
    for idx in range(len(routes[drone])):
        route = [drone + 1 - len(Drones)] + routes[drone][idx] + [drone + 1 - len(Drones)]
        route_corrected = [idx + len(Drones) for idx in route]

        new_link = ET.SubElement(links, 'link')
        new_link.set('id', str(drone+1) + '999' + str(route_corrected[1]))
        new_link.set('from', str(drone+1))
        new_link.set('to', str(route_corrected[1]))
        new_link.set('length', str(distance(Drones[drone+1], Nodes[route[1]])))
        new_link.set('capacity', str(capacity))
        new_link.set('freespeed', str(freespeed))
        new_link.set('permlanes', str(permlanes))
        new_link.set('modes', str(mode))
        for i in range(1,len(route)-2):
            # Create a new link element
            new_link = ET.SubElement(links, 'link')
            new_link.set('id', str(route_corrected[i])+'999'+str(route_corrected[i+1]))
            new_link.set('from', str(route_corrected[i]))
            new_link.set('to', str(route_corrected[i+1]))
            new_link.set('length', str(distance(Nodes[route[i]],Nodes[route[i+1]])))
            new_link.set('capacity', str(capacity))
            new_link.set('freespeed', str(freespeed))
            new_link.set('permlanes', str(permlanes))
            new_link.set('modes', str(mode))
        new_link = ET.SubElement(links, 'link')
        new_link.set('id', str(str(route_corrected[-2])) +'999'+ str(drone + 1))
        new_link.set('from', str(route_corrected[-2]))
        new_link.set('to', str(drone+1))
        new_link.set('length', str(distance(Drones[drone + 1], Nodes[route[-2]])))
        new_link.set('capacity', str(capacity))
        new_link.set('freespeed', str(freespeed))
        new_link.set('permlanes', str(permlanes))
        new_link.set('modes', str(mode))

# Generate the modified XML string
xml_string = ET.tostring(network, encoding='utf-8')

# Format the XML string with indentation and newlines
dom = minidom.parseString(xml_string)
formatted_xml = dom.toprettyxml(indent="   ")

# Remove empty lines
formatted_xml = '\n'.join(line for line in formatted_xml.splitlines() if line.strip())


# Insert the DOCTYPE declaration at the desired position
doctype = '<!DOCTYPE network SYSTEM "http://www.matsim.org/files/dtd/network_v1.dtd">'
formatted_xml = formatted_xml.replace('<?xml version="1.0" ?>', '<?xml version="1.0" ?>\n' + doctype, 1)

# Save the formatted XML to a file
with open('network.xml', 'w', encoding='utf-8') as f:
    f.write(formatted_xml)

#%% Writing of plans.xml ##########################################################################################
"""
In MATSim, an agent always has to travel along a full link. It is not possible for the agent to leave in the middle
of a link, independent of the actual activity coordinate (due to technical reasons/implementation details). 
Agents can enter or leave traffic on a link always at the end of a link in MATSim. 

Create the root element <plans>
"""
root = ET.Element('plans')
root.set('xml:lang','de-CH')


for drone in range(len(Drones)):
    person = ET.SubElement(root, 'person')
    person.set('id', str(drone+1))

    # Create the <plan> element
    plan = ET.SubElement(person, 'plan')


    # Create the XML tree
    tree = ET.ElementTree(root)
    for idx in range(len(routes[drone])):
        route = [drone + 1 - len(Drones)] + routes[drone][idx] + [drone + 1 - len(Drones)]
        route_corrected = [idx + len(Drones) for idx in route]
        #plan
        if idx == 0:
        #first one
            new_act = ET.SubElement(plan, 'act')
            new_act.set('type', 'h')
            new_act.set('x', str(Drones[drone+1][0]))
            new_act.set('y', str(Drones[drone+1][1]))
            new_act.set('link', str(route_corrected[-2])+'999'+str(route_corrected[-1]))
            new_act.set('end_time', begin_time)
            # Create a new leg element
            new_leg = ET.SubElement(plan, 'leg')
            new_leg.set('mode', str(mode))
            # new_route = ET.SubElement(leg, 'route')
        for i in range(1,len(route_corrected)-1):
            # Create a new act element
            new_act = ET.SubElement(plan, 'act')
            new_act.set('type', 's')
            new_act.set('x', str(Nodes[route[i]][0]))
            new_act.set('y', str(Nodes[route[i]][1]))
            new_act.set('link', str(route_corrected[i-1])+'999'+str(route_corrected[i]))
            new_act.set('dur',duration_of_surveillancy)
            # Create a new leg element
            new_leg = ET.SubElement(plan, 'leg')
            new_leg.set('mode', 'car')
            # new_route = ET.SubElement(leg, 'route')
        if idx != len(routes[drone])-1:
            new_act = ET.SubElement(plan, 'act')
            new_act.set('type', 'h')
            new_act.set('x', str(Drones[drone + 1][0]))
            new_act.set('y', str(Drones[drone + 1][1]))
            new_act.set('link', str(route_corrected[-2]) +'999'+ str(route_corrected[-1]))
            new_act.set('dur', duration_of_refill)
            new_leg = ET.SubElement(plan, 'leg')
            new_leg.set('mode', 'car')
        else:
        #last one
            new_act = ET.SubElement(plan, 'act')
            new_act.set('type', 'h')
            new_act.set('x', str(Drones[drone + 1][0]))
            new_act.set('y', str(Drones[drone + 1][1]))
            new_act.set('link', str(route_corrected[-2]) +'999'+ str(route_corrected[-1]))

# Generate the modified XML string
xml_string = ET.tostring(root, encoding='utf-8')

# Format the XML string with indentation and newlines
dom = minidom.parseString(xml_string)
formatted_xml = dom.toprettyxml(indent="    ")

# Remove empty lines
formatted_xml = '\n'.join(line for line in formatted_xml.splitlines() if line.strip())


# Insert the DOCTYPE declaration at the desired position
doctype = '<!DOCTYPE plans SYSTEM "http://www.matsim.org/files/dtd/plans_v4.dtd">'
formatted_xml = formatted_xml.replace('<?xml version="1.0" ?>', '<?xml version="1.0" ?>\n' + doctype, 1)

# Save the formatted XML to a file
with open('plans.xml', 'w', encoding='utf-8') as f:
    f.write(formatted_xml)




