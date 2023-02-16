"""
Assignment #2 Template file
"""
import random
import math
import numpy as np

"""
Problem Statement
--------------------
Implement the planning algorithm called Rapidly-Exploring Random Trees* (RRT)
for the problem setup given by the RRT_DUBINS_PROMLEM class.

INSTRUCTIONS
--------------------
1. The only file to be submitted is this file rrt_star_planner.py. Your
   implementation can be tested by running RRT_DUBINS_PROBLEM.PY (check the 
   main function).
2. Read all class and function documentation in RRT_DUBINS_PROBLEM carefully.
   There are plenty of helper function in the class to ease implementation.
3. Your solution must meet all the conditions specificed below.
4. Below are some do's and don'ts for this problem as well.

Conditions
-------------------
There are some conditions to be satisfied for an acceptable solution.
These may or may not be verified by the marking script.

1. The solution loop must not run for more that a certain number of random iterations
   (Specified by a class member called MAX_ITER). This is mainly a safety
   measure to avoid time-out-related issues and will be set generously.
2. The planning function must return a list of nodes that represent a collision-free path
   from start node to the goal node. The path states (path_x, path_y, path_yaw)
   specified by each node must define a Dubins-style path and traverse from node i-1 -> node i.
   (READ the documentation for the node class to understand the terminology)
3. The returned path should have the start node at index 0 and goal node at index -1,
   while the parent node for node i from the list should be node i-1 from the list, ie,
   the path should be a valid list of nodes.
   (READ the documentation of the node to understand the terminology)
4. The node locations must not lie outside the map boundaries specified by
   RRT_DUBINS_PROBLEM.map_area.

DO(s) and DONT(s)
-------------------
1. Do not rename the file rrt_star_planner.py for submission.
2. Do not change change the PLANNING function signature.
3. Do not import anything other than what is already imported in this file.
4. You can write more function in this file in order to reduce code repitition
   but these function can only be used inside the PLANNING function.
   (since only the planning function will be imported)
"""

def rrt_star_planner(rrt_dubins, display_map=False):
    """
        Execute RRT* planning using Dubins-style paths. Make sure to populate the node_list.

        Inputs
        -------------
        rrt_dubins  - (RRT_DUBINS_PROBLEM) Class conatining the planning
                      problem specification
        display_map - (boolean) flag for animation on or off (OPTIONAL)

        Outputs
        --------------
        (list of nodes) This must be a valid list of connected nodes that form
                        a path from start to goal node

        NOTE: In order for rrt_dubins.draw_graph function to work properly, it is important
        to populate rrt_dubins.nodes_list with all valid RRT nodes.
    """
    # LOOP for max iterations
    i = 0
    child_dict={} # a dictionary to maintain the children of every node
    while i < rrt_dubins.max_iter:
        i += 1

        # Generate a random vehicle state (x, y, yaw)- every 60th time it trys for goal
        if i %60 ==0:
            try_node=rrt_dubins.goal
            print("trying goal")
        else:
            # the random state is always within the bounds of the map
            x= random.random()*(rrt_dubins.x_lim[1]-rrt_dubins.x_lim[0])+ rrt_dubins.x_lim[0]
            y= random.random()*(rrt_dubins.y_lim[1]-rrt_dubins.y_lim[0])+ rrt_dubins.y_lim[0]
            yaw= np.deg2rad(random.random()*360)
            try_node=rrt_dubins.Node(x,y,yaw)

        minimum=float('inf') # variable to keep track of closest node within a certain radius of of the
        # node we are trying
        minimum_dist=float('inf')#variable to keep track of the closest node used if no node is within
        # the radius

        gamma=30 # a factor we set to control the size of our radius
        radius= gamma*math.sqrt(math.log(len(rrt_dubins.node_list))/len(rrt_dubins.node_list)) # radius in which
        # we search for nodes

        check_nodes=[] # list of nodes in the radius
        chosen_node= None # the node with least cost within the radius
        closest_node= None # the node with least cost if no node is within radius

        for nodes in rrt_dubins.node_list: #search through list of all valid nodes

            #find closest node irrespective of radius
            if rrt_dubins.calc_new_cost(nodes,try_node)-nodes.cost<minimum_dist:
                closest_node=nodes
                minimum_dist= rrt_dubins.calc_new_cost(nodes,try_node)

            # find nodes within radius and add to check node list
            if rrt_dubins.calc_new_cost(nodes,try_node)-nodes.cost<radius:
                check_nodes.append(nodes)
                # find the closest node within the given radius
                if rrt_dubins.calc_new_cost(nodes,try_node)< minimum:
                    chosen_node=nodes
                    minimum= rrt_dubins.calc_new_cost(nodes,try_node)

        # if there is no node within radius consider the closest node
        if isinstance(chosen_node, type(None)):
            chosen_node=closest_node

        # make a path to the closest node
        new_node=rrt_dubins.propogate(chosen_node,try_node)


        # Draw current view of the map
        # PRESS ESCAPE TO EXIT
        if display_map:
            rrt_dubins.draw_graph()

        # Check if the path between nearest node and random state has obstacle collision
        if rrt_dubins.check_collision(new_node):
            # Add the node to nodes_list if it is valid
            rrt_dubins.node_list.append(new_node) # Storing all valid nodes

            #add node to the child dictionary under the chosen node as it is a child of the
            #chosen node
            if chosen_node in child_dict:
                child_list=child_dict[chosen_node]
                child_list.append(new_node)
                #child_dict[chosen_node]=child_list
            else:
                child_dict[chosen_node]=[new_node, ]


            # check if there is a shorter path via the new node to the nodes within the radius
            for nodes in check_nodes:
                if rrt_dubins.calc_new_cost(new_node,nodes)< nodes.cost:
                    # if it is shorter- check for collision to connect
                    rewire_node=rrt_dubins.propogate(new_node,nodes)
                    if rrt_dubins.check_collision(rewire_node):
                        #calculate the cost difference in the re wiring
                        cost_change=nodes.cost-rewire_node.cost

                        #propergate the change in cost through all the children from the re wire node
                        if nodes in child_dict:
                            cost_propogate=child_dict[nodes]
                            while cost_propogate:
                                n= cost_propogate[0]
                                n.cost= n.cost-cost_change
                                cost_propogate.pop(0)
                                if n in child_dict:
                                    cost_propogate.extend(child_dict[n])

                        #add the new rewired node in place of the old node
                        rrt_dubins.node_list.remove(nodes)
                        nodes=rewire_node
                        rrt_dubins.node_list.append(nodes)



        else:
            continue




        # Check if new_node is close to goal
        if new_node.is_state_identical(rrt_dubins.goal):
            print("Iters:", i, ", number of nodes:", len(rrt_dubins.node_list))
            node_list=[] # list of nodes from goal to start
            while new_node is not None:
                node_list.append(new_node) # append nodes on path from goal node to start
                new_node=new_node.parent
            return node_list[::-1] #reverse list so it is from start to goal

    if i == rrt_dubins.max_iter:
        print('reached max iterations')

    # Return path, which is a list of nodes leading to the goal in case  goal has not been found
    # in this case we find the closest node to the goal and make a path from the goal
    minimum=float('inf')
    for nodes in rrt_dubins.node_list:
        if rrt_dubins.calc_dist_to_goal(nodes.x,nodes.y)<minimum:
            chosen_node=nodes
            minimum= rrt_dubins.calc_dist_to_goal(nodes.x,nodes.y)
    node_list=[]
    while chosen_node is not None:
        node_list.append(chosen_node)
        chosen_node=chosen_node.parent
    return node_list[::-1] #reverse list so it is from start to goal
