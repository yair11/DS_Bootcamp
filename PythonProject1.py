from geopy.geocoders import Nominatim
import osmnx as ox
import taxicab as tc
import matplotlib.pyplot as plt
import networkx as nx
import plotly.graph_objects as go
import numpy as np

def validating_street(address,all_the_streets_in_city,calling_for_location = True):
  similar_streets = []
  # all_the_streets_in_city = all_streets_in_city(address[2])
  while True:
    street_for_check = address[0]
    if street_for_check in all_the_streets_in_city:
      print ('I find your street!')
      return address
    else:
      print ('I can\'t find your street, looking for smiliar one:')
      for street_in_city in all_the_streets_in_city:
        if street_for_check in street_in_city:
          similar_streets.append(street_in_city)
      if len(similar_streets) != 0:
        for similar_street in similar_streets:
          user_answer = (input(f'do you mean: "{similar_street}" ?(Y/N)'))
          if user_answer == 'Y':
            address[0] = similar_street
            return address
      if (len(similar_streets) == 0) or (user_answer == 'N'):
        print('Did\'nt find your street\'s name put another one:')
        if calling_for_location:
          address = getting_address('location')
        else:
          address = getting_address('destination')

def all_streets_in_city(city):
  query = {'city': city}
  gdf = ox.geocode_to_gdf(query)
  G = ox.graph_from_place(query, network_type='all')
  print(G.edges(data=True))
  list_of_streets = []
  for i in range(len(list(G.edges(data=True)))-1):
    try:
      if isinstance(list(G.edges(data=True))[i][2]['name'],str):
        list_of_streets.append(list(G.edges(data=True))[i][2]['name'])
      else:
        list_of_streets.extend(list(G.edges(data=True))[i][2]['name'])
    except:
      pass
  return set(list_of_streets)

#place = location\destination
#address[0-street,1-number,2-city]
def getting_address(place):
  address = []
  address.append(input(f'Please enter your {place} street\'s name: '))
  address.append(input(f'Please enter the number of your {place} address: '))
  address.append(input(f'Please enter your {place} city\'s name: '))
  return address

def convert_address_list_for_string(address):
  return ' '.join(address)

#Convert addresss of location & destination to coordinate
def convert_address_to_coordinate(address):
  geolocator = Nominatim(user_agent="my_request")
  coordinate = geolocator.geocode(address)
  return coordinate



# Finding the optimal path
def  finding_the_optimal_path(G, origin_node, destination_node):
  return nx.shortest_path(G, origin_node, destination_node, weight = 'length')

# storing the longitudes and latitudes of the routs in route list
def storing_lat_and_lon_of_the_routs(G,route):
  long = []
  lat = []
  for i in route:
      point = G.nodes[i]
      long.append(point['x'])
      lat.append(point['y'])
  return long,lat


def plot_path(lat, long, origin_point, destination_point):
    """
    Given a list of latitudes and longitudes, origin
    and destination point, plots a path on a map

    Parameters
    ----------
    lat, long: list of latitudes and longitudes
    origin_point, destination_point: co-ordinates of origin
    and destination
    Returns
    -------
    Nothing. Only shows the map.
    """
    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=long,
        lat=lat,
        marker={'size': 10},
        line=dict(width=4.5, color='blue')))
    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name="Source",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        marker={'size': 12, 'color': "red"}))

    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        lon=[destination_point[1]],
        lat=[destination_point[0]],
        marker={'size': 12, 'color': 'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)
    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="stamen-terrain",
                      mapbox_center_lat=30, mapbox_center_lon=-80)
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0},
                      mapbox={
                          'center': {'lat': lat_center,
                                     'lon': long_center},
                          'zoom': 13})
    fig.show()

"""<------------------------------------------------------------------------------------------------->"""
"""<--------------------------------------------Main------------------------------------------------->"""
"""<------------------------------------------------------------------------------------------------->"""
all_the_streets_in_city = all_streets_in_city('כפר סבא')

address = validating_street(getting_address('location'),all_the_streets_in_city)
user_location_address = convert_address_list_for_string(address)

address = validating_street(getting_address('destination'),all_the_streets_in_city,False)
user_destination_address = convert_address_list_for_string(address)

location_coordinate = convert_address_to_coordinate(user_location_address)
destination_coordinate = convert_address_to_coordinate(user_destination_address)

#Getting map of the location area
G = ox.graph_from_point((location_coordinate.latitude,location_coordinate.longitude), dist=5000, network_type='walk')

# define origin and desination locations
origin_point = (location_coordinate.latitude,location_coordinate.longitude)
destination_point = (destination_coordinate.latitude,destination_coordinate.longitude)

# get the nearest nodes to the location and destination
origin_node = ox.get_nearest_node(G, origin_point)
destination_node = ox.get_nearest_node(G, destination_point)

route = finding_the_optimal_path(G, origin_node, destination_node)

long,lat = storing_lat_and_lon_of_the_routs(G,route)

plot_path(lat, long, origin_point, destination_point)

def node_list_to_path(G, node_list):
    """
    Given a list of nodes, return a list of lines that together
    follow the path
    defined by the list of nodes.
    Parameters
    ----------
    G : networkx multidigraph
    route : list
        the route as a list of nodes
    Returns
    -------
    lines : list of lines given as pairs ( (x_start, y_start),
    (x_stop, y_stop) )
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(),
                   key=lambda x: x['length'])
        # if it has a geometry attribute
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute,
            # then the edge is a straight line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)
    return lines


# getting the list of coordinates from the path
# (which is a list of nodes)
lines = node_list_to_path(G, route)
long2 = []
lat2 = []
for i in range(len(lines)):
    z = list(lines[i])
    l1 = list(list(zip(*z))[0])
    l2 = list(list(zip(*z))[1])
    for j in range(len(l1)):
        long2.append(l1[j])
        lat2.append(l2[j])


plot_path(lat2, long2, origin_point, destination_point)

