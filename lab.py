#!/usr/bin/env python3

from util import read_osm_data, great_circle_distance, to_local_kml_url

# NO ADDITIONAL IMPORTS!


ALLOWED_HIGHWAY_TYPES = {
    'motorway', 'trunk', 'primary', 'secondary', 'tertiary', 'unclassified',
    'residential', 'living_street', 'motorway_link', 'trunk_link',
    'primary_link', 'secondary_link', 'tertiary_link',
}


DEFAULT_SPEED_LIMIT_MPH = {
    'motorway': 60,
    'trunk': 45,
    'primary': 35,
    'secondary': 30,
    'residential': 25,
    'tertiary': 25,
    'unclassified': 25,
    'living_street': 10,
    'motorway_link': 30,
    'trunk_link': 30,
    'primary_link': 30,
    'secondary_link': 30,
    'tertiary_link': 25,
}


def build_auxiliary_structures(nodes_filename, ways_filename):
    """
    Create any auxiliary structures you are interested in, by reading the data
    from the given filenames (using read_osm_data)
    """
    '''Pretty much just a compilation of all my helpers

    ids_to_ids is a dictionary mapping ids to another dictionary
    This dictionary contains two keys. One of the keys, 'ids', 
    is a reference to all of the acceptable adjacent nodes
    which the selected id can reach. The other key is 'location'
    which maps to a tuple giving the location of the id.

    speed_dict is a dictionary mapping of (a tuple of one id to another id)
    to (the speed limit of the way that connects these two ids).

    location_to_id is a dictionary which maps (a tuple giving the location
    of the id) to the id. 
    It's like the second key of ids_to_ids but inverted.
    '''

    possible_ways = is_highway(ways_filename)
    ids_to_ids, speed_dict = ids_connected_to_ids(possible_ways)
    ids_to_ids = ids_to_location(ids_to_ids, nodes_filename)

    location_to_id = locs_to_ids(ids_to_ids)

    return ids_to_ids, speed_dict, location_to_id

def find_short_path(aux_structures, loc1, loc2):
    """
    Return the shortest path between the two locations

    Parameters:
        aux_structures: the result of calling build_auxiliary_structures
        loc1: tuple of 2 floats: (latitude, longitude), representing the start
              location
        loc2: tuple of 2 floats: (latitude, longitude), representing the end
              location

    Returns:
        a list of (latitude, longitude) tuples representing the shortest path
        (in terms of distance) from loc1 to loc2.
    """

    valid_ids, speed_dict, location_to_id = aux_structures
    id1, id2 = find_nearest_nodes(valid_ids, loc1, loc2)
    expanded_set = set() #will check if id has already been checked

    #agenda stores the path that has been followed to get from id1 to current id, current cost, and heuristic value (line 107 for description)
    agenda = [([valid_ids[id1]['location']], 0, great_circle_distance(valid_ids[id1]['location'], valid_ids[id2]['location']))]

    #another list which simulatenously goes with agenda, but only appends the value for which we are declaring the next valid id (heuristic value)
    determine_least_heuristic = [great_circle_distance(valid_ids[id1]['location'], valid_ids[id2]['location'])]

    while len(agenda) != 0: #if agenda empties means there's no possible path from id1 to id2
        current_index = determine_least_heuristic.index(min(determine_least_heuristic)) #finds index of element in list with least heuristic value
        least_cost_path, current_cost, least_heuristic = agenda[current_index] #splits tuple

        if least_cost_path[-1] == valid_ids[id2]['location']:
            #if last element of least_cost_path is id2, then we have found the cheapest path
            return least_cost_path
        
        agenda.pop(current_index) #gets rid of the index we're checking
        determine_least_heuristic.pop(current_index)#follows suit with above

        current_id = location_to_id[least_cost_path[-1]] #declares current id the last id of the least_cost_path list which has the path of least heuristic cost

        if current_id in expanded_set:
            continue #if current_id has been checked, then we go into another iteration of the while loop
        expanded_set.add(current_id) #adds current_id to expanded_set to declare we have checked it already

        for ID in valid_ids[current_id]['ids']:
            if ID not in expanded_set: #if ID has already been checked, we know all its IDs so it's superfluous. We skip it.
                x = great_circle_distance(valid_ids[current_id]['location'], valid_ids[ID]['location']) #finds cost which will accrue from current_id to ID
                y = great_circle_distance(valid_ids[ID]['location'], valid_ids[id2]['location']) #computes what we have left from ID to id2 (heuristic value)
                agenda.append((least_cost_path + [valid_ids[ID]['location']], current_cost + x, current_cost + x + y))#appends path and cost so far and heurstic value to agenda
                determine_least_heuristic.append(current_cost + x + y) #current_cost + distance for these two IDs + what we got left is heuristic value.
 
    return None

def find_fast_path(aux_structures, loc1, loc2):
    """
    Return the shortest path between the two locations, in terms of expected
    time (taking into account speed limits).

    Parameters:
        aux_structures: the result of calling build_auxiliary_structures
        loc1: tuple of 2 floats: (latitude, longitude), representing the start
              location
        loc2: tuple of 2 floats: (latitude, longitude), representing the end
              location

    Returns:
        a list of (latitude, longitude) tuples representing the shortest path
        (in terms of time) from loc1 to loc2.
    """

    valid_ids, speed_dict, location_to_id = aux_structures
    id1, id2 = find_nearest_nodes(valid_ids, loc1, loc2)
    expanded_set = set() #will check if id has already been checked

    #agenda stores the path that has been followed to get from id1 to current id, current cost, and heuristic value (line 107 for description)
    agenda = [([valid_ids[id1]['location']], 0)]
    determine_least_time = [0] #fast path will be indexing into the path with the least time of traversal so far.

    while len(agenda) != 0: #if length of agenda is 0 then there is not path connecting id1 and id2
        current_index = determine_least_time.index(min(determine_least_time)) #finds the index of the item with the least time taken so far
        least_time_path, current_time = agenda[current_index] #splits tuple in agenda list

        if least_time_path[-1] == valid_ids[id2]['location']:
            return least_time_path #if last id in least_time_path matches 

        agenda.pop(current_index)#gets rid of the id we're checking (we're gonna find all the ids it's connected to, no need to check it again)
        determine_least_time.pop(current_index)#follows suit as above

        current_id = location_to_id[least_time_path[-1]]#makes current_id the last id of the path which has taken the least time so far

        if current_id in expanded_set:
            continue #enters into another iteration of the while loop

        expanded_set.add(current_id) #adds current_id into expanded_set. We're gonna be checking it's children. No need to check it again after this time.

        for ID in valid_ids[current_id]['ids']:
            if ID not in expanded_set: #if ID has already been checked, no need to check it again
                #evaluates the time that it will take to get from current_id to ID using the speed limit connecting the 2 ids.
                time_elapsed = (great_circle_distance(valid_ids[current_id]['location'], valid_ids[ID]['location']))/(speed_dict[(current_id, ID)])

                #appends a new element into agenda. This element contains the path that has been taken so far to get ID. Also contains the time that it's taken from id1 to ID.
                agenda.append((least_time_path + [valid_ids[ID]['location']], current_time + time_elapsed))

                #appends only the time from id1 to ID so we can easily index into this list and get the index with the least time.
                determine_least_time.append(current_time + time_elapsed)
    return None

def is_highway(filename):
    highways = []

    for node in read_osm_data(filename):
        if 'highway' in node['tags']:
            if node['tags']['highway'] in ALLOWED_HIGHWAY_TYPES:
                highways.append(node)
                
    return highways

def ids_connected_to_ids(valid_ways):
    '''Given a list of valid ways (which have nodes in them)'''
    valid_ids = {} #this will be a dictionary mapping an id to all its valid adjacent ids and its location given by a (lat, lon) tuple.
    speed_dict = {} #a dictionary mapping a tuple (id1, id2) to the maximum speed limit allowed in the way that connects id1 and id2.

    for highway in valid_ways: #loops through all the possible ways
        '''
        Lines 185 - 188
        Obtain the speed limit of the highway. Looks into the highway node
        to find if there is a maxspeed key. If there is, it assigns the limit to the value in this key.
        If there isn't, then it looks at what type of way it is, looks into the dictionary
        DEFAULT_SPEED_LIMIT_MPH and assigns the appropriate limit.
        '''
        if 'maxspeed_mph' in highway['tags']: 
            speed = highway['tags']['maxspeed_mph']
        else:
            speed = DEFAULT_SPEED_LIMIT_MPH[highway['tags']['highway']]
        for i in range(len(highway['nodes'])): #is going to loop through all the nodes that are connected by the chosen highway.
            if 'oneway' not in highway['tags'] or highway['tags']['oneway'] == 'no':
                #mapping ids to adjacent ids and to their location for a 2 way street.
                if highway['nodes'][i] not in valid_ids: #will make the id key map it to its adjacent ids and location.
                    valid_ids[highway['nodes'][i]] = {'ids': set(), 'location': None}
                if len(highway['nodes']) == 1:
                    break #without this line, ways of length one will try to get next value and will go out of range.
                if i == 0:
                    valid_ids[highway['nodes'][i]]['ids'].add(highway['nodes'][1]) #zeroth id, so can only add the first.
                elif i == (len(highway['nodes']) - 1): #last id can only add (last-1)th id. 
                    valid_ids[highway['nodes'][i]]['ids'].add(highway['nodes'][i-1])
                else: #middle ids can add both behind and ahead of them.
                    valid_ids[highway['nodes'][i]]['ids'].add(highway['nodes'][i-1])
                    valid_ids[highway['nodes'][i]]['ids'].add(highway['nodes'][i+1])
                #speeds dictionary for two way
                if i < len(highway['nodes']) - 1:
                    #two way street, so limit is valid both ways.
                    speed_dict[(highway['nodes'][i], highway['nodes'][i+1])] = speed
                    speed_dict[(highway['nodes'][i+1], highway['nodes'][i])] = speed
            else:
                #mapping ids to next id and to their location (one way street)
                if highway['nodes'][i] not in valid_ids: #declares a key which will map to its adjacent ids and location.
                    valid_ids[highway['nodes'][i]] = {'ids': set(), 'location': None}
                if len(highway['nodes']) == 1 or (i == len(highway['nodes'])-1):
                    break #if nodes list is length 1 then nothing to add. If index is in the last element, then nothing to add since we're on a one way street.
                else:
                    valid_ids[highway['nodes'][i]]['ids'].add(highway['nodes'][i+1]) #always add only next id.
                #speeds dictionary for one ways
                if i < len(highway['nodes']) - 1:
                    #one way street, so only adding the index and the next one to the dictionary.
                    speed_dict[(highway['nodes'][i], highway['nodes'][i+1])] = speed

    return valid_ids, speed_dict

            
def ids_to_location(valid_ids, node_filename):
    '''
    Will replace location values in our dictionary.
    Key will be 'location'.
    Maps to the coordinates.
    '''
    data = read_osm_data(node_filename)

    for node in data:#loops through all nodes in dataset
        if node['id'] in valid_ids: #if it finds an id which is in our dictionary, adjust the value in its 'location' key and make it the coordinates.
            valid_ids[node['id']]['location'] = (node.get('lat', 0), node.get('lon', 0))
    return valid_ids

def find_nearest_nodes(valid_ids, loc1, loc2):
    '''
    Takes two locations. Finds the nearest id to each loc by computing the
    minimum distance from loc to all our ids. Only stores the minimum 
    distance (to compare it) and the corresponding minimum id, which
    it will actualy return. Does this for both loc1 and loc2.
    '''
    inputs = [loc1, loc2]
    minimum = float('inf')
    out = []
        
    for value in inputs:
        for ID in valid_ids:
            location = valid_ids[ID]['location']
            if great_circle_distance(value, location) < minimum:
                minimum = great_circle_distance(value, location)
                minimum_id = ID
        out.append(minimum_id)
        minimum = float('inf') #redeclares the minimum to correctly evaluate for loc2
    
    return out[0], out[1]

def locs_to_ids(valid_ids):
    '''
    Creates a dictionary that maps from locations to ids.

    Utilizes the structure we have already built. Extracts the value
    that we have in 'location'. Makes that the key, then makes the value 
    the ID.
    '''

    locs_to_id_dic = {}

    for ID in valid_ids:
        locs_to_id_dic[valid_ids[ID].get('location', None)] = ID

    return locs_to_id_dic

if __name__ == '__main__':
    # additional code here will be run only when lab.py is invoked directly
    # (not when imported from test.py), so this is a good place to put code
    # used, for example, to generate the results for the online questions.

    #a = find_short_path(build_auxiliary_structures('resources/cambridge.nodes', 'resources/cambridge.ways'), (42.3858, -71.0783), (42.5465, -71.1787))
    #c = find_short_path_heurs(build_auxiliary_structures('resources/cambridge.nodes', 'resources/cambridge.ways'), (42.3858, -71.0783), (42.5465, -71.1787))
    #print(a[1])
    #print(c[1])
    x, u, v = build_auxiliary_structures('resources/mit.nodes', 'resources/mit.ways')
    
    print(find_short_path((x, u, v), (42.355, -71.1009), (42.3612, -71.092)))