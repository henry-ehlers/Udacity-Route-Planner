import math
import operator
import copy 

class Node :
    
    def __init__(self, index, longitude, latitude):
        self.index       = index
        self.longitude   = longitude
        self.latitude    = latitude
        self.connections = []
    
    # Calculate this node's distance to another node
    def distance(self, other_intersection) :
        assert(isinstance(other_intersection, Node))
        delta_longitude = self.longitude - other_intersection.longitude
        delta_latitude  = self.latitude  - other_intersection.latitude
        distance = math.sqrt(math.pow(delta_latitude, 2) + math.pow(delta_longitude, 2))
        return distance
    
    # Add a single node connection to this instance
    def add_connection(self, connection) :
        assert(isinstance(connection, Node))
        self.connections.append(connection)
    
    # Add a bunch of connections contained in a list of nodes
    def add_connections(self, connections) :
        assert(isinstance(connections, list))
        for connection in connections:
            self.add_connection(connection)
    
    # Return the list of node connections
    def get_connections(self) :
        return self.connections
    
    # Return this node's coordinates
    def get_coordinates(self) :
        return (self.longitude, self.latitude)
    
    # Return this node's map index
    def get_index(self) :
        return self.index
        
class Network:
    
    def __init__(self, intersections, roads, root = None, target = None):
        assert(isinstance(intersections, dict))
        assert(isinstance(roads, list))
        
        # Initialize the nodes and their connections
        self._initialize_nodes(intersections)
        self._connect_nodes(roads)
        
        # If a target and root are providede, set them
        if root:   self._set_root(root)
        if target: self._set_target(target)
        
        # Set all route finding related variables to empty/none
        self.route   = None
        self.visited = []
        self.queue   = []
    
    # iterate over the dictionary of nodes, and create node objects
    def _initialize_nodes(self, intersections):
        self.nodes = [None] * len(intersections)
        for key, info in enumerate(intersections.items()) :
            coord = info[1]
            index = info[0]
            self.nodes[index] = Node(index, coord[0], coord[1])
    
    # For each index's set of vertices, add the appropriate references to nodes
    def _connect_nodes(self, roads):
        for index, vertices in enumerate(roads):
            for vertice in vertices:
                if vertice: self.nodes[index].add_connection(self.nodes[vertice])
    
    # Ensure the provided root / origin is valid, and set it
    def _set_root(self, root_index):
        assert(isinstance(root_index, int))
        assert(root_index < len(self.nodes))
        self.root = self.nodes[root_index]
    
    # Ensure the provided target / goal is valid, and set it
    def _set_target(self, target_index):
        assert(isinstance(target_index, int))
        assert(target_index < len(self.nodes))
        self.target = self.nodes[target_index]
        
    def find_route(self, root = None, target = None):
        
        # If new roots and targets (for the same map are provided)
        if root:   self._set_root(root)
        if target: self._set_target(target)
        
        # Reset all route-relevant variables in case of new root and target
        self.queue   = [Route([self.root], 0.0, self.root.distance(self.target))]
        self.visited = []
        self.route   = None
        
        # Recurse to find the best route
        self._find_route_recursive()
    
    def _find_route_recursive(self) :
        
        # Identify best route so far and its frontier node
        current_route = self.queue.pop()
        self.visited.append(current_route.get_node())
        
        # Check if Target Has been found, if so return
        if current_route.get_node() == self.target:
            self.route = current_route
            return
            
        # Iterate over each of the current route's connections
        for connection in current_route.get_node().get_connections() :

            # Create new route based on next connection IF NOT ALREADY VISITED
            if connection not in self.visited:
                new_route = self._new_route(current_route, connection)
                self.queue.append(new_route)   
        
        # Sort available routes in REVERSE
        self.queue.sort(key=operator.attrgetter('f_value'), reverse = True)
        
        # If the queue is non-empty, recurse
        if self.queue:
            self._find_route_recursive()
        
        # Return
        return
    
    def get_found_path(self):
        
        # If a route has already been found, return its path indeces
        if self.route:
            return self.route.get_path()
        else:
            return None
    
    def _new_route(self, current_route, new_connection) :
        
        # Calculate h-value heuristic based on distance to target
        h_value = new_connection.distance(self.target)
        
        # Calculate total travel distance to current node
        g_value  = current_route.get_node().distance(new_connection)
        g_value += current_route.get_g_value()
        
        # Copy route and append new node to it
        new_history = copy.copy(current_route.get_route())
        new_history.append(new_connection)
        
        # return new route object
        return Route(new_history, g_value, h_value)

class Route :
    
    def __init__(self, route, g_value, h_value) :
        assert(isinstance(route, list))
        assert(isinstance(g_value, float))
        assert(isinstance(h_value, float))
        self.route   = route
        self.h_value = h_value
        self.g_value = g_value
        self._calculate_f_value()
    
    # Calculate the f-value
    def _calculate_f_value(self) :
        self.f_value = self.g_value + self.h_value
    
    # Return the calculated g-value
    def get_g_value(self) :
        return self.g_value
   
    # Return the frontier node
    def get_node(self) :
        return self.route[-1]
    
    # Return the route list
    def get_route(self) :
        return self.route
    
    # Create path indeces from route list of nodes
    def get_path(self) :
        path = [None] * len(self.route)
        for index, node in enumerate(self.route):
            path[index] = node.get_index()
        return path

def shortest_path(_map, _start, _goal):
    print("shortest path called")
    network = Network(_map.intersections, _map.roads, _start, _goal)
    network.find_route()
    return network.get_found_path()
