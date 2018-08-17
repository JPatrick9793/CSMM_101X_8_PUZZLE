#!/usr/bin/python3
import sys
import numpy as np
import copy
import time

def bfs(initial_state_matrix, goal_matrix):
    start_time = time.time()
    # create initial state object
    initial_state = State(initial_state_matrix)
   
    # create frontier queue
    frontier_object = []
    frontier_states = []
    explored_states = []
    
    # put the initial state in the frontier
    frontier_object.append(initial_state)
    frontier_states.append(initial_state.get_matrix())
    
    count = 0
    # iterate through the frontier
    while frontier_object:
        # pop state object from stack
        state = frontier_object.pop(0)
        # pop state matrix from stack
        state_matrix = frontier_states.pop(0)
        # add the current state matrix to the explored set
        explored_states.append(state.get_matrix())
        # check if the current state is the goal state
        
        # print ("current node level:\t", state.get_level())
        # print ("current node path:\t", state.get_path())
        # print ("state_matrix:\n", state_matrix)
        
        if (state_matrix == goal_matrix).all():
            time_elapsed = time.time() - start_time
            # memory = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            return "SUCCESS", state, time_elapsed, count
        # get list of child nodes
        child_nodes = state.generate_children_nodes()
        count += 1
        # iterate through children nodes
        for child in child_nodes:
            child_matrix = child.get_matrix()
          
            if not any((child_matrix==x).all() for x in explored_states):
                if not any((child_matrix==y).all() for y in frontier_states):
              
                    frontier_object.append(child)
                    frontier_states.append(child_matrix)
    
    # memory = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    time_elapsed = time.time() - start_time           
    return "FAILURE", state, time_elapsed, count
   
   

def dfs(initial_state_matrix, goal_matrix):
    start_time = time.time()
    # create initial state object
    initial_state = State(initial_state_matrix)
   
    # create frontier queue
    frontier_object = []
    frontier_states = []
    explored_states = []
    
    # put the initial state in the frontier
    frontier_object.append(initial_state)
    frontier_states.append(initial_state.get_matrix())
    
    node_count = 0
    max_depth = 1
    # iterate through the frontier
    # while frontier_object:
    while node_count < 3000:
        # pop state object from stack
        state = frontier_object.pop()
        # pop state matrix from stack
        state_matrix = frontier_states.pop()
        # add the current state matrix to the explored set
        explored_states.append(state.get_matrix())
        # check if the current state is the goal state
        
        if (state_matrix == goal_matrix).all():
            time_elapsed = time.time() - start_time
            # memory = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            return "SUCCESS", state, time_elapsed, node_count
        # get list of child nodes
        child_nodes = state.generate_children_nodes()
        node_count += 1
        # iterate through children nodes
        for child in reversed(child_nodes):
            child_matrix = child.get_matrix()
            depth = child.get_level()
            if not any((child_matrix==x).all() for x in explored_states):
                if not any((child_matrix==y).all() for y in frontier_states):
                    if depth > max_depth:
                        max_depth = depth 
                    frontier_object.append(child)
                    frontier_states.append(child_matrix)
    
    # memory = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    time_elapsed = time.time() - start_time           
    return "FAILURE", state, time_elapsed, node_count
   
  
  
class State:
    # constructor for initial state
    def __init__(self, state, level=0, path=list()):
        self.matrix = state
        self.level = level
        self.path = path
        # get shape of state matrix
        self.shape = self.matrix.shape
        # find position of the 0
        itemindex = np.where(self.matrix==0)
        # assign to variables to make easier
        self.i_row = itemindex[0][0]
        self.j_col = itemindex[1][0]
     
    # getters
    def get_matrix(self):
        return self.matrix
    def get_level(self):
        return self.level
    def get_path(self):
        return self.path
     
     
    # method for generating new state objects
    # returns list of possible moves
    def generate_children_nodes(state):
        moveset = state.determine_possible_moves()
        child_nodes = []
        if "Up" in moveset:
            dummy_state = copy.deepcopy(state)
            child_node = dummy_state.move_up()
            child_nodes.append(child_node)
        if "Down" in moveset:
            dummy_state = copy.deepcopy(state)
            child_node = dummy_state.move_down()
            child_nodes.append(child_node)
        if "Left" in moveset:
            dummy_state = copy.deepcopy(state)
            child_node = dummy_state.move_left()
            child_nodes.append(child_node)
        if "Right" in moveset:
            dummy_state = copy.deepcopy(state)
            child_node = dummy_state.move_right()
            child_nodes.append(child_node)
        return child_nodes

        
    def determine_possible_moves(self):
        # create empty list for possible moveset
        moveset = []
        # check if moves possible
        if self.i_row > 0:
            moveset.append("Up")
        if self.i_row < self.shape[0]-1:
            moveset.append("Down")
        if self.j_col > 0:
            moveset.append("Left")
        if self.j_col < self.shape[1]-1:
            moveset.append("Right")
        return moveset
       
    def move_up(self):
        # assign matrix to new variable
        # change indices
        next_matrix = self.matrix
        next_matrix[self.i_row][self.j_col] = next_matrix[self.i_row-1][self.j_col]
        next_matrix[self.i_row-1][self.j_col] = 0
        # append the move to the object path
        next_path = self.path
        next_path.append("Up")
        # increase the level by 1
        next_level = self.level + 1
        child_node = State(next_matrix, next_level, next_path)
        return child_node
      
    def move_down(self):
        # assign matrix to new variable
        # change indices
        next_matrix = self.matrix
        next_matrix[self.i_row][self.j_col] = next_matrix[self.i_row+1][self.j_col]
        next_matrix[self.i_row+1][self.j_col] = 0
        # append the move to the object path
        next_path = self.path
        next_path.append("Down")
        # increase the level by 1
        next_level = self.level + 1
        child_node = State(next_matrix, next_level, next_path)
        return child_node
      
    def move_left(self):
        # assign matrix to new variable
        # change indices
        next_matrix = self.matrix
        next_matrix[self.i_row][self.j_col] = next_matrix[self.i_row][self.j_col-1]
        next_matrix[self.i_row][self.j_col-1] = 0
        # append the move to the object path
        next_path = self.path
        next_path.append("Left")
        # increase the level by 1
        next_level = self.level + 1
        child_node = State(next_matrix, next_level, next_path)
        return child_node
      
    def move_right(self):
        # assign matrix to new variable
        # change indices
        next_matrix = self.matrix
        next_matrix[self.i_row][self.j_col] = next_matrix[self.i_row][self.j_col+1]
        next_matrix[self.i_row][self.j_col+1] = 0
        # append the move to the object path
        next_path = self.path
        next_path.append("Right")
        # increase the level by 1
        next_level = self.level + 1
        child_node = State(next_matrix, next_level, next_path)
        return child_node
       
       
       
# method for generating new state objects
# returns list of possible moves
def generate_children_nodes(state):
    moveset = state.determine_possible_moves()
    print ("moveset: ", moveset)
    child_nodes = []
    if "Up" in moveset:
        dummy_state = copy.deepcopy(state)
        child_node = dummy_state.move_up()
        child_nodes.append(child_node)
    if "Down" in moveset:
        dummy_state = copy.deepcopy(state)
        child_node = dummy_state.move_down()
        child_nodes.append(child_node)
    if "Left" in moveset:
        child_node = state.move_left()
        child_nodes.append(child_node)
    if "Right" in moveset:
        child_node = state.move_right()
        child_nodes.append(child_node)
    return child_nodes
       
      
      
def create_matrix_from_input(string_starting_input):
    # extract numbers from string input
    list_starting_input = [x.strip() for x in string_starting_input.split(',')]
    # convert entries to ints
    for i in range(len(list_starting_input)):
        list_starting_input[i] = int(list_starting_input[i])
    # determine number of columns
    n_rows = int(np.sqrt(len(list_starting_input)))
    # print for debugging
    # convert list to array for reshaping
    output_matrix = np.array(list_starting_input)
    # reshape to have squared property
    output_matrix = output_matrix[:n_rows*n_rows]
    output_matrix = output_matrix.reshape(n_rows, n_rows)
    # return the reshaped array
    return output_matrix
    

if __name__ == "__main__":
    
    initial_state_matrix = create_matrix_from_input(sys.argv[2])

    goal_matrix = np.matrix([[0, 1, 2],
                             [3, 4, 5],
                             [6, 7, 8]])
    
    if sys.argv[1] == 'bfs':
       response, output, time, n_nodes = bfs(initial_state_matrix, goal_matrix)
       # memory = memory / 1024 / 1024
       print (response)
       try:
           file = open("output.txt", "w")
           file.write("path_to_goal: {0}\n".format(output.get_path() ) )
           file.write("cost_of_path: {0}\n".format(output.get_level() ) )
           file.write("nodes_expanded: {0}\n".format(n_nodes) )
           file.write("search_depth: {0}\n".format(output.get_level() ) )
           file.write("max_search_depth: {0}\n".format(output.get_level()+1) )
           file.write("running_time: {0}\n".format(time) )
           # file.write("max_ram_usage: {0}".format(memory))
           file.close()
       except:
           print (output)
    
    if sys.argv[1] == 'dfs':
       response, output, time, n_nodes = dfs(initial_state_matrix, goal_matrix)
       # memory = memory / 1024 / 1024
       print (response)
       try:
           file = open("output.txt", "w")
           file.write("path_to_goal: {0}\n".format(output.get_path() ) )
           file.write("cost_of_path: {0}\n".format(output.get_level() ) )
           file.write("nodes_expanded: {0}\n".format(n_nodes) )
           file.write("search_depth: {0}\n".format(output.get_level() ) )
           file.write("max_search_depth: {0}\n".format(output.get_level()+1) )
           file.write("running_time: {0}\n".format(time) )
           # file.write("max_ram_usage: {0}".format(memory))
           file.close()
       except:
           print (output)
    
    