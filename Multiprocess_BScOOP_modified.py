

# -*- coding: utf-8 -*-
"""
Created on Fri Nov 17 13:56:47 2023

@author: James
"""

from abc import ABC, abstractmethod
import random
import copy
import numpy as np
from matplotlib import pyplot as plt
from multiprocessing import Pool
import numpy.polynomial.polynomial as poly

plt.rc('xtick',labelsize = 22)
plt.rc('ytick',labelsize = 22)
# GLOBAL PARAMETERS


safety_distance:int = 1
slowing_probability:float = 0.2
slowing_probability_0:float = 0
lane_changing_probability:float = 0.3
road_length = 1000
flow_rate_point = (0, road_length - 1)
max_velocity_default = 5
cars_to_follow = []
car_positons_over_time = [] # creating a list to store car positions at each time step.
#OBSTACLE PARAMETERS

obstacle_position = 0
time_to_add_obstacle =  0
time_to_remove_obstacle = 0

#ROAD INCIDENT PARAMETERS
#An incident occurs where all cars can only move at a speed < max_velocity for a period of time. 
#Until the incident clears again.
time_road_incident_true = -1 #set to -1 if not using
time_road_incident_false = 0
road_incident_velocity = 0 #THIS MESSES WITH STUFF (NEED TO CHECK)

#CAR BREAKDOWN PARAMETERS
#An car "breaks down" set the velocity of a car you choose to move with v < max_velocity for
#a specified time period. 
car_id_to_breakdown = 15
cars_to_follow.append(car_id_to_breakdown)
car_breakdown_time = -1 #set to -1 if not using
car_fixed_time = -1 #set to -1 if not using
car_breakdown_velocity = 1

class RoadObject(ABC):
    """ class RoadObject
    - serves as parent class for subsequent 'road objects' e.g. cars & obstacles
    - could expand to include several other objects, e.g. lorries (a car that occupies more than one cell)
    """
    @abstractmethod
    def pretty_print(self):
        pass

class RoadSimulation():
    def __init__(self):
        pass
    


class Car(RoadObject):
    """ class Car
    - inside RoadObject parent class, i.e. Car is type:RoadObject
    - each car assigned unique ID, makes tracking individual cars possible. (self.ID)
    Attributes:
    velocity (int) : V
    max_velocity (int) : , default value = 5
    """
    next_id = 0
    
    def __init__(self, velocity:int, max_velocity:int = 5):
        """
        Initialising car object
        
        Parameters
        ----------
        velocity : int
            Velocity of car 0<=v<=max_velocity
        max_velocity : int
            maximum velocity that a car can take. The default is 5.

        Returns
        -------
        None.
        """
        self.id = Car.next_id
        Car.next_id = Car.next_id + 1
        if (velocity == None):
            self.velocity = random.randint(0, max_velocity)
        else:
            self.velocity = velocity
        self.max_velocity = max_velocity
    
    def pretty_print(self):
        """
        function to display the car in a "pretty way"
        Returns
        -------
        output : chr, str
            Returns a unicode character for easy identification if the car has ID equal to car to follow.
            else returns the velocity of the current car as a str.
        """
        output = "" 
        if self.id in cars_to_follow:
            output = chr(0x1D7D8 + self.velocity)
        else:
            output = str(self.velocity)
        return output
         
class Obstacle(RoadObject):
    """ class Obstacle
    - inside RoadObject parent class, i.e. obstacle is type:RoadObject.
    """
    def __init__(self):
        """
        Returns
        -------
        None.
        """
        pass
    
    def pretty_print(self):
        """
        Returns
        -------
        str
            Prints a little unicode mountain to depict on the road the location of the obstacle.
        """
        return '⛰'


class RoadCell():
    """ class RoadCell:
    -The road is defined as a set of cells that a RoadObject can occupy.
        """
    def __init__(self, index: int, road_object: RoadObject, next_index:int, previous_index:int):
        """
        Parameters
        ----------
        index : int
            Current road cell index.
        road_object : RoadObject
            passing RoadObjects into RoadCell i.e. a road_object can occupy a RoadCell.
        next_index : int
            The next road cell i.e. index+1 .
        previous_index : int
            the previous road cell i.e. index-1.
        Returns
        -------
        None.

        """
        self.index = index
        self.road_object = road_object
        self.next_index = next_index
        self.previous_index = previous_index
    def add_road_object(self, road_object:RoadObject):
        """
        Parameters
        ----------
        road_object : RoadObject
            Inserting (add) a road_object.

        Returns
        -------
        None.
        """
        self.road_object = road_object
        
    def del_road_object(self):
        """
        Deletes a road_object. sets road_object in RoadCell to NoneType.
        Returns
        -------
        None.
        """
        self.road_object = None
        
    def pretty_print(self):
        """
        Returns
        -------
        TYPE
            If road_object is NoneType, display this as a '.' on the road, indicating an empty space.
            Else. use the pretty_print() functions defined previously for the road_objects and display those.
        """
        if self.road_object == None:
            return '.'
        else:
            return self.road_object.pretty_print()
        
            
                
def road_update(road:list[RoadCell], flow_count: int, road_length: int, slowing_probability: int, lane_changing_probability: int): 
    """
    Parameters
    ----------
    road : list[RoadCell]
        define the road, its type is a list of RoadCell's.
    flow_count : int
        When a car reaches the end of the road, the flow_count is incremented 
        and is used later to calculate flow rate.
    road_length : int
        The actual length of road used in the simulation .

    Returns
    -------
    next_road : TYPE
        A copy of the initial road is made using 'deepcopy' module then this road
        is adjusted in the function and output.
    flow_count : TYPE
        The cars are moved and stored here.
    """
    road = change_lanes(road, road_length, lane_changing_probability) #EXECUTE THE LANE CHANGES BEFORE THE MOVEMENT OF CARS.
    next_road = copy.deepcopy(road)
    for road_cell in road:
        if type(road_cell.road_object) == Car:
            headway = calculate_headway(road, road_cell.index, road_length) #gap between two adjacent cars
            car_velocity = road_cell.road_object.velocity
            velocity = car_velocity #deafult if no conditions are met
            
            #METASTABILITY SLOW TO START rule 0 
            if car_velocity == 0 and random.random() <= slowing_probability_0:
                velocity = 0
            else:
                #RULES FOR MOVING
               # print(car_velocity, road_cell.road_object.max_velocity, headway)                                     
                #acceleration   
                if car_velocity < road_cell.road_object.max_velocity and headway >= car_velocity + 1:
                    velocity = min(headway - safety_distance, car_velocity + 1)
                #deceleration
                if car_velocity > 0 and headway <= car_velocity:
                    velocity = min(headway - safety_distance, car_velocity)
                #Randomization
                if car_velocity > 0 and random.random() <= slowing_probability:
                    velocity = min(headway - safety_distance, car_velocity - 1)
            
                        
            velocity = max(0, velocity)
            
            flow_count = move_car(next_road, road_cell.index, velocity, flow_count)    
            
            
    return next_road, flow_count

def change_lanes(road:list[RoadCell], road_length, lane_changing_probability: int):
    """ def change_lanes
    Parameters
    ----------
    road : list[RoadCell]
        define the road, its type is a list of RoadCell's.
    road_length : TYPE
        The length of the road.
    Returns
    -------
    road : TYPE
        Returns the road after the lane changes have been performed.
    """
    skip_cell_indexes = []
    for i in range(len(road)):
        road_cell = road[i]
        if road_cell.index not in skip_cell_indexes:
            if isinstance(road_cell.road_object, Car):
                #print(road_cell.index,road_cell.road_object.id)
                if random.random() < lane_changing_probability: #4 If probability condition is satisfied;
                    headway = calculate_headway(road, road_cell.index, road_length)
                    car_velocity = road_cell.road_object.velocity
                    other_lane_indexes = []
                    #print(f"Index: {road_cell.index}, Car Id: {road_cell.road_object.id}, Headway: {headway}, Car Vel: {car_velocity}, ")

                    if (road_cell.index + road_length) < len(road):
                        other_lane_indexes.append(road_cell.index + road_length)
                    if (road_cell.index - road_length) >= 0:
                        other_lane_indexes.append(road_cell.index - road_length)
                    #print(other_lane_indexes)
                    if len(other_lane_indexes) == 0:
                        continue
                    other_lane_index = random.choice(other_lane_indexes)
                    if road[other_lane_index].road_object != None:
                        continue
                    #print(road_cell.index, other_lane_index, other_lane_indexes)
                    other_lane_headway = calculate_headway(road, other_lane_index, road_length)
                    other_lane_car_index, other_lane_behind_headway = calculate_behind_headway_and_index(road, other_lane_index, road_length)
                    #Perform lane change checks;
                    #1 Check the car on the current lane is not impeded by the car in front of it;
                    if headway - safety_distance <= car_velocity:
                        if other_lane_headway > car_velocity: #2 Check the cars position on the other lane, its headway with our car needs to be greater than the cars speed;
                            if type(road[other_lane_car_index].road_object) == Car:
                                if road[other_lane_car_index].road_object.velocity < other_lane_behind_headway:  #3 Check behind for the car on the other lane and find its headway with your car;
                                    road[other_lane_index].add_road_object(road[road_cell.index].road_object)
                                    road[road_cell.index].del_road_object()
                                    skip_cell_indexes.append(other_lane_index) #to prevent multiple changes at same time step.
                            else:
                                road[other_lane_index].add_road_object(road[road_cell.index].road_object)
                                road[road_cell.index].del_road_object()
                                skip_cell_indexes.append(other_lane_index) #to prevent multiple changes at same time step.
    # if len(skip_cell_indexes) > 0:
    #     print(''.join(['|' if i in np.mod(np.array(skip_cell_indexes), road_length) else ' ' for i in range(road_length)]))
    #     for i in range(len(road) // road_length):
    #         print(''.join(x.pretty_print() for x in road[road_length*i:road_length*i + road_length - 1]))
                       
           

           
                       

        #Change lane operation;
        #delete current road object;
    return road
        

def move_car(road:list[RoadCell], index:int , velocity:int, flow_count:int):
    """
    1. set next_index to be the current one
    2. iterate n times where n = velocity of car at index
    3. the new index is now the index + the velocity of the car we are concerned with.

    road[index].road_object.velocity sets the car to the new velocity
    then moves car using road[next_index] and then .add_road_object to insert the new object into the
    correct position determined in prev steps (1-3)
    removes car from previous using .del_road_object.
    
    Parameters
    ----------
    road : list[RoadCell]
        passing road as a Type (list RoadCells).
    index : int
        Current index.
    velocity : int
        velocity of car.
    flow_count : int
        When a car reaches the end of the road, the flow_count is incremented 
        and is used later to calculate flow rate.

    Returns
    -------
    flow_count : TYPE
        moving the car along.

    """
    next_index = index
    for i in range(velocity):
        #print(road[index].index, road[index].next_index, road[index].previous_index)
        if (road[index].next_index < road[index].previous_index):
        #if (road[next_index].next_index, next_index) == flow_rate_point:
            flow_count += 1
        next_index = road[next_index].next_index
    
    road[index].road_object.velocity = velocity
    road[next_index].add_road_object(road[index].road_object)
    if next_index != index:
        road[index].del_road_object()
        
    return flow_count
    


def calculate_headway(road: list[RoadCell], index:int, road_length: int, headway: int = 0):
    """ calculate_headway
    Parameters
    ----------
    road : list[RoadCell]
        passing road as a Type (list RoadCells).
    index : int
        Current Index.
    road_length : int
        Length of road.
    headway : int
        Set default headway to be 0. The default is 0.

    Returns
    -------
    get the next index by finding the current index and then using .next_index to find the
    road cell directly in front.
    
    if said road cell contains a road_object. Return the headway (i.e. gap between objects)
    
    elif for the multi lane case, if the lane is empty (i.e. the function will continue looking
                                                        in the lane forever without finding 
                                                        anything, then simply return road_length, i.e.
                                                        the headway of an empty lane is the length of
                                                        the lane itself.)
    
    if not, recursion; call function again, but increment headway by + 1 until next
    road_object is found.
    
    
        

    """
    next_index = road[index].next_index
   
    if road[next_index].road_object != None:
        return headway
    elif headway > max_velocity_default + 1:
        return max_velocity_default + 1
    else:
        return calculate_headway(road, next_index, road_length, headway+1) #headway was headway+1

def calculate_behind_headway_and_index(road: list[RoadCell], index:int, road_length: int, headway: int = 0):
    """ calc behind_headway & index
    
    Parameters
    ----------
    road : list[RoadCell]
        passing road as a Type (list RoadCells).
    index : int
        Current index.
    road_length : int
        Length of road.
    headway : int, optional
        Set default headway to be 0. The default is 0.
    Returns
    -------
 get the previous index by finding the current index and then using .previous_index to find the
 road cell directly behind.
 
 if said road cell contains a road_object. Return the headway (i.e. gap between objects)
 
 elif for the multi lane case, if the lane is empty (i.e. the function will continue looking
                                                     in the lane forever without finding 
                                                     anything, then simply return road_length, i.e.
                                                     the headway of an empty lane is the length of
                                                     the lane itself.)
 
 if not, recursion; call function again, but increment headway by + 1 until the previous
 road_object is found.

    """
    previous_index = road[index].previous_index
    if road[previous_index].road_object != None:
        return previous_index, headway
    elif headway > (max_velocity_default + 1):
        return previous_index,(max_velocity_default+1)
    else:
        return calculate_behind_headway_and_index(road, previous_index, road_length, headway+1) #headway was headway+1
    

    
def initialize_road(road_length:int, density:float):
    """
    Parameters
    ----------
    road_length : int
        length of road.
    density : float
        Density of cars on the road float value 0-1, 1 means every possible road cell is occupied.

    Returns
    -------
    road : Array
        road as an array of RoadCells.

    """
    Car.next_id = 0
    road = []
    for i in range(road_length):
        road.append(RoadCell(i,None,(i+1) % road_length, (i-1) % road_length))
        
    number_of_cars = min(int(road_length/2),int(density*road_length))           #calculating the number of real cars on the road based on the density
        
    safety_gap = max(1, road_length // number_of_cars)                          #calculate the gap between cars to maintain a one cell safety distance.

#ADDING THE CARS TO THE ROAD EVENLY SPACED OUT
    for i in range(0,number_of_cars): 
        position = safety_gap * i
        road[position].add_road_object(Car(velocity=None))                      #add a road object in the position and set it to have no velocity.
    return road

def initialize_road_multi_lane(road_length:int, density:float, no_of_lanes:int):
    """
    Parameters
    ----------
    road_length : int
        length of road.
    density : float
        Density of cars on the road, float value 0-1, 1 means every possible road cell is occupied.
    no_of_lanes : int
        The number of lanes on the highway (Must take a minimum value of 1).

    Returns
    -------
    road : Array
        road as an array of RoadCells.
    """
    Car.next_id = 0
    road = []
    for i in range(no_of_lanes):                                                #N.B. Python begins counting at 0, this is cruical to the function.
        for j in range(road_length):
            index = (i*road_length)+j                                           #Same as before except (i*road_length) prefactor gives 0 if 1st lane
            next_index = (i*road_length)+(j+1)%road_length                      #Same as before except (i*road_length) prefactor gives 0 if 1st lane
            previous_index = ((i*road_length)+j-1) % road_length + ((i*road_length)+j//road_length)
            """ previous_index documentation
            This is slightly more complex than the single lane case, so I think it deserves some explanation on how it works.
            The first term which is simple enough, ((i*road_length)+j-1) is the (index-1) % road_length which doesnt require
            much explanation. This is simply, take the current position and -1 from the index and modulo to give its position in the lane
            """
            road.append(RoadCell(index,None,next_index, previous_index))
            
        number_of_cars = min(int(road_length/2),int(density*road_length))      #calculating the number of real cars on the road based on the density
        
        safety_gap = 1                   #calculate the gap between cars to maintain a one cell safety distance.

        for j in range(0,number_of_cars):
            position = safety_gap * j
            road[position + i*road_length].add_road_object(Car(velocity=None))
    
    return road

def calculate_average_velocity(road: list[RoadCell]):
    total_velocity = 0
    total_cars = 0
    
    for road_cell in road:
        if isinstance(road_cell.road_object, Car):
            total_velocity += road_cell.road_object.velocity
            total_cars += 1
        
    if total_cars > 0:
        return round(total_velocity/total_cars,2)
    else:
        return 0 #No cars on the road.
            

#SIMULATION RUNNING
"""
iterations = 100
densities = np.linspace(0.01,1,50)
flow_rate_data_single_lane = []
average_velocities = []
for density in densities:
    if density > 0.5:
        flow_rate_data_single_lane.append(0)
        continue
    flow_count = 0
    road = initialize_road(road_length, density)
    for i in range(iterations):
        if i < 10:
            print(f"iteration {i + 1}:" + f"density{density}")
            print(''.join(x.pretty_print() for x in road))
        #OBSTACLE CHECK
        if i == time_to_add_obstacle:
            road[obstacle_position].add_road_object(Obstacle())
        if i == time_to_remove_obstacle:
            road[obstacle_position].del_road_object()
            
        #INCIDENT CHECK
        if i == time_road_incident_true:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    road[i].road_object.max_velocity = road_incident_velocity
                    road[i].road_object.velocity = min(road_incident_velocity, road[i].road_object.velocity)
                    
        if i == time_road_incident_false:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    road[i].road_object.max_velocity = max_velocity_default
        
        #CAR BREAKDOWN
        if i == car_breakdown_time:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    if road[i].road_object.id == car_id_to_breakdown:
                        road[i].road_object.max_velocity = car_breakdown_velocity
                        road[i].road_object.velocity = min(car_breakdown_velocity, road[i].road_object.velocity)
                        break
            
        if i == car_fixed_time:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    if road[i].road_object.id == car_id_to_breakdown:
                        road[i].road_object.max_velocity = max_velocity_default        
                        break
        
        road, flow_count = road_update(road, flow_count, road_length)
        
        # average_velocity = calculate_average_velocity(road)
        # print(average_velocity)
        # average_velocities.append(average_velocity)
        
        
    flow_rate_data_single_lane.append(flow_count / (iterations * road_length))

"""
#Seperate Test Simulation for avg velocity function
"""
density_to_test = 0.1
iterations = 100
avg_velocities_single_density = []
flow_count = 0
road = initialize_road(road_length, density_to_test)
print(road[0].road_object.id)
for i in range(iterations):
    
    positions_at_current_time = [1 if isinstance(cell.road_object,Car) else 0 for cell in road]
    #car_positons_over_time.append(positions_at_current_time)
    
    average_velocity = calculate_average_velocity(road)
    avg_velocities_single_density.append(average_velocity)
    print(f"iteration {i + 1}:" + f"avg velocity {average_velocity}")
    print(''.join(x.pretty_print() for x in road))
    # for road_cell in road:
    #     if isinstance(road_cell.road_object, Car):
    #         print(f"{road_cell.index} Car Velocity: {road_cell.road_object.velocity}")
    #     elif isinstance(road_cell.road_object, Obstacle):
    #         print(f"{road_cell.index} There is an obstacle here")
    if i == time_to_add_obstacle:
        road[obstacle_position].add_road_object(Obstacle())
    if i == time_to_remove_obstacle:
        road[obstacle_position].del_road_object()
        
    #INCIDENT CHECK
    if i == time_road_incident_true:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                road[i].road_object.max_velocity = road_incident_velocity
                road[i].road_object.velocity = min(road_incident_velocity, road[i].road_object.velocity)
                
    if i == time_road_incident_false:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                road[i].road_object.max_velocity = max_velocity_default
                
    #CAR BREAKDOWN
    if i == car_breakdown_time:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                if road[i].road_object.id == car_id_to_breakdown:
                    road[i].road_object.max_velocity = car_breakdown_velocity
                    road[i].road_object.velocity = min(car_breakdown_velocity, road[i].road_object.velocity)
                    break
        
    if i == car_fixed_time:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                if road[i].road_object.id == car_id_to_breakdown:
                    road[i].road_object.max_velocity = max_velocity_default        
                    break
    
    road, flow_count = road_update(road, flow_count, road_length)


#positions_array = np.array(car_positons_over_time)
print(avg_velocities_single_density)

# print(average_velocities)
#Plotting fundamental diagram


plt.title('Fundamental Traffic Flow-Density Graph 1 lane')
plt.xlabel('Density')
plt.ylabel('Flow Rate')
plt.grid(True)
plt.show()
"""
"""
plt.title(f"Average velocity // Time Steps for density {density_to_test} ")
plt.plot(range(iterations), avg_velocities_single_density, marker = 'o', markersize = 2, linestyle = '-', color ='blue')
#OBSTACLE CASE ANNOTATION
if obstacle_position > 0:
    plt.annotate("Inserted Obstacle", 
                 xy=(time_to_add_obstacle, avg_velocities_single_density[time_to_add_obstacle]), 
                 xytext =(time_to_add_obstacle,(avg_velocities_single_density[time_to_add_obstacle] + 0.2)),
                 arrowprops=dict(arrowstyle='->'))
    plt.annotate("Removed Obstacle", 
                 xy=(time_to_remove_obstacle, avg_velocities_single_density[time_to_remove_obstacle]),
                 xytext =(time_to_remove_obstacle, avg_velocities_single_density[time_to_remove_obstacle] + 0.2),
                 arrowprops=dict(arrowstyle = '->'))   
#ROAD INCIDENT ANNOTATION
if time_road_incident_true > 0:
    plt.annotate("road incident start", 
                 xy=(time_road_incident_true, avg_velocities_single_density[time_road_incident_true]), 
                 xytext =(time_road_incident_true,(avg_velocities_single_density[time_road_incident_true] + 0.2)),
                 arrowprops=dict(arrowstyle='->'))
    plt.annotate("Road incident end", 
                 xy=(time_road_incident_false, avg_velocities_single_density[time_road_incident_false]),
                 xytext =(time_road_incident_false, avg_velocities_single_density[time_road_incident_false] + 0.2),
                 arrowprops=dict(arrowstyle = '->'))   
#BREAKDOWN INCIDENT ANNOTATION
if car_breakdown_time > 0:
    plt.annotate("Breakdown starts",
                 xy=(car_breakdown_time, avg_velocities_single_density[car_breakdown_time]), 
                 xytext =(car_breakdown_time,(avg_velocities_single_density[car_breakdown_time] + 0.2 )),
                 arrowprops=dict(arrowstyle='->'))
    plt.annotate("Breakdown ends",
                 xy=(car_fixed_time, avg_velocities_single_density[car_fixed_time]), 
                 xytext =(car_fixed_time,(avg_velocities_single_density[car_fixed_time] + 0.2 )),
                 arrowprops=dict(arrowstyle='->'))

plt.show() 


"""

#road = []
#for i in range(100):
#    road.append(RoadCell(i, None, (i+1) % 100))
#road[1].road_object = Car(2)
#road[5].road_object = Car(2)
#road[99].road_object = Car(3)
#print(''.join(x.pretty_print() for x in road))
#road, flow_count = road_update(road, flow_count)
#print(''.join(x.pretty_print() for x in road))
#def update(road):
 #   for i in range(number_of_cars):
"""
no_of_lanes = 3
positions_array = []
density_to_test = 0.2
iterations = 1000
road=initialize_road_multi_lane(road_length, density_to_test, no_of_lanes)
for i in range(iterations):
    if i % 100 == 0:
        print(i)
    flow_count = 0
    
    positions_at_current_time = [1 if isinstance(cell.road_object,Car) else 0 for cell in road]
    car_positons_over_time.append(positions_at_current_time)
        
    average_velocity = calculate_average_velocity(road)
    
  #  print(f"iteration {i + 1}:" + f"avg velocity {average_velocity}")
    #for j in range(no_of_lanes):
     #   print(''.join(x.pretty_print() for x in road[road_length*j:road_length*j + road_length - 1]))
    
    if i == time_to_add_obstacle:
            road[obstacle_position].add_road_object(Obstacle())
    if i == time_to_remove_obstacle:
            road[obstacle_position].del_road_object()
            
        #INCIDENT CHECK
    if i == time_road_incident_true:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                road[i].road_object.max_velocity = road_incident_velocity
                road[i].road_object.velocity = min(road_incident_velocity, road[i].road_object.velocity)
                break
    if i == time_road_incident_false:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                road[i].road_object.max_velocity = max_velocity_default
                break
        #CAR BREAKDOWN
    if i == car_breakdown_time:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                if road[i].road_object.id == car_id_to_breakdown:
                    road[i].road_object.max_velocity = car_breakdown_velocity
                    road[i].road_object.velocity = min(car_breakdown_velocity, road[i].road_object.velocity)                        
                    break
            
    if i == car_fixed_time:
        for i in range(road_length):
            if isinstance(road[i].road_object, Car):
                if road[i].road_object.id == car_id_to_breakdown:
                    road[i].road_object.max_velocity = max_velocity_default        
                    break
    
    road, flow_count = road_update(road, flow_count,road_length)
positions_array = np.array(car_positons_over_time)

#GRAYSCALE IMAGE
# Display the grayscale plot
simulation_parameters = (
    f"Slowing Probability: {slowing_probability}\n"
    f"Road Length: {road_length}\n"
    f"Obstacle Position: {obstacle_position}\n"
    f"Time of Obstacle Insertion: {time_to_add_obstacle}")

plt.figure(figsize=(10, 6))
plt.imshow(positions_array, cmap='Greys', aspect='auto')
plt.title('Time over Car Positions' + f' @ Density {density_to_test}')
plt.xlabel('Position')
plt.ylabel('Time')
plt.colorbar(label='Car Presence')  # Add color bar for clarity



#Plotting a red line to show obstacle position
obstacle_start = time_to_add_obstacle
obstacle_end = time_to_remove_obstacle

line_positions = np.zeros((iterations, len(road)))
line_positions[obstacle_start:obstacle_end, obstacle_position] = 1

# Overlay the red line onto the grayscale plot
plt.imshow(line_positions, cmap='Reds', aspect='auto', alpha=0.5)

plt.text(
    road_length * 0.7, iterations * 0.9, simulation_parameters,
    bbox=dict(facecolor='white', alpha=0.8),
    fontsize=10,
    verticalalignment='top'
)
plt.show()

positions_array_array = [[inner[(len(inner)*i//no_of_lanes):(len(inner)*(i+1)//no_of_lanes)] for inner in positions_array] for i in range(no_of_lanes)]

for i in range(no_of_lanes):
    #GRAYSCALE IMAGE
    # Display the grayscale plot
    simulation_parameters = (
        f"Slowing Probability: {slowing_probability}\n"
        f"Road Length: {road_length}\n"
        f"Obstacle Position: {obstacle_position}\n"
        f"Time of Obstacle Insertion: {time_to_add_obstacle}")

    plt.figure(figsize=(10, 6))
    plt.imshow(positions_array_array[i], cmap='Greys', aspect='auto')
    plt.title('Time over Car Positions' + f' @ Density {density_to_test}')
    plt.xlabel('Position')
    plt.ylabel('Time')
    plt.colorbar(label='Car Presence')  # Add color bar for clarity


    if (obstacle_position > len(road) * i // no_of_lanes and obstacle_position < len(road) * (i+1) // no_of_lanes):
        #Plotting a red line to show obstacle position
        obstacle_start = time_to_add_obstacle
        obstacle_end = time_to_remove_obstacle
    
        line_positions = np.zeros((iterations, len(road) // no_of_lanes))
        line_positions[obstacle_start:obstacle_end, obstacle_position%road_length] = 1
    
        # Overlay the red line onto the grayscale plot
        plt.imshow(line_positions, cmap='Reds', aspect='auto', alpha=0.5)

    plt.text(
        road_length * 0.7, iterations * 0.9, simulation_parameters,
        bbox=dict(facecolor='white', alpha=0.8),
        fontsize=10,
        verticalalignment='top'
    )
    plt.show()
"""
#MULTI LANE SIM (MULTI DENSITIES FUNDAMENTAL PLOT)
# for i in range(190): DELETE CARS IN SOME LANES IF YOU WISH.
#     if road[i+100].road_object != None:
#         road[i+100].del_road_object()
#  Flow density code   

def simulation(density,number_of_lanes,iterations,slowing_probability,lane_changing_probability):
    if density > 0.5:
        return 0
    #number_of_lanes = 3
    flow_count = 0
    #iterations = 1000
    percentage_of_iterations_to_be_removed = 10
    road = initialize_road_multi_lane(road_length, density, number_of_lanes)
    for i in range(iterations):
        if i == (iterations // 100)*percentage_of_iterations_to_be_removed:
            flow_count = 0
       # if i < 1:
        #    print(f"iteration {i + 1}:" + f"density{density}")
         #   for j in range(number_of_lanes):
          #      print(''.join(x.pretty_print() for x in road[road_length*j:road_length*j + road_length - 1]))


        positions_at_current_time = [1 if isinstance(cell.road_object,Car) else 0 for cell in road]
        car_positons_over_time.append(positions_at_current_time)
         
        average_velocity = calculate_average_velocity(road)
    
        # print(f"iteration {i + 1}:" + f"avg velocity {average_velocity}")
        # for j in range(3):
        #     print(''.join(x.pretty_print() for x in road[road_length*j:road_length*j + road_length - 1]))
    
        if i == time_to_add_obstacle:
            road[obstacle_position].add_road_object(Obstacle())
        if i == time_to_remove_obstacle:
            road[obstacle_position].del_road_object()
            
        #INCIDENT CHECK
        if i == time_road_incident_true:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    road[i].road_object.max_velocity = road_incident_velocity
                    road[i].road_object.velocity = min(road_incident_velocity, road[i].road_object.velocity)
                    break
        if i == time_road_incident_false:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    road[i].road_object.max_velocity = max_velocity_default
                    break
        #CAR BREAKDOWN
        if i == car_breakdown_time:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    if road[i].road_object.id == car_id_to_breakdown:
                        road[i].road_object.max_velocity = car_breakdown_velocity
                        road[i].road_object.velocity = min(car_breakdown_velocity, road[i].road_object.velocity)
                        break
            
        if i == car_fixed_time:
            for i in range(road_length):
                if isinstance(road[i].road_object, Car):
                    if road[i].road_object.id == car_id_to_breakdown:
                        road[i].road_object.max_velocity = max_velocity_default        
                        break
        
        road, flow_count = road_update(road, flow_count,road_length, slowing_probability, lane_changing_probability)
    return flow_count/((iterations // 100)*percentage_of_iterations_to_be_removed)


"""
# Use Pool from multiprocessing
if __name__ == '__main__':
    densities = np.linspace(0.01, 0.5, 50)
    with Pool(processes = 23) as pool:
        flow_rates_data = pool.map(simulation, densities)
        
    plt.scatter(densities, flow_rates_data, marker = '.', s = 4, color = 'blue')
    trend2 = np.polyfit(densities,flow_rates_data, 2)
    trend3 = np.polyfit(densities,flow_rates_data, 3)
    trend4 = np.polyfit(densities,flow_rates_data, 4)
    trend5 = np.polyfit(densities,flow_rates_data, 5)
    trendpoly2 = np.poly1d(trend2)
    trendpoly3 = np.poly1d(trend3)
    trendpoly4 = np.poly1d(trend4)
    trendpoly5 = np.poly1d(trend5)
    #plt.plot(densities,trendpoly2(densities), label = 'degree 2')
    #plt.plot(densities,trendpoly3(densities), label = 'degree 3')
    #plt.plot(densities,trendpoly4(densities),  label = 'degree 4')
    plt.plot(densities,trendpoly5(densities), label = 'degree 5')     
    plt.legend()
    #plt.plot(densities, flow_rate_data_single_lane, marker = 'x', color = 'red', label = 'Single Lane')
    plt.title('Fundamental Traffic Flow-Density Graph')
    plt.xlabel('Density')
    plt.ylabel('Flow Rate')
    plt.grid(True)
    plt.show()
    
"""    
from scipy.signal import savgol_filter
#N LANES FLOW DIAGRAM
#Use Pool from multiprocessing
if __name__ == '__main__':
    densities_to_test = np.linspace(0.01, 0.5, 100)
    number_of_lanes_to_test = [3]
    no_of_iterations = [1000]
    slowing_probabilities_to_test = [0.2]
    lane_changing_probabilities_to_test = [0,0.3,0.7,1]
    

    
    # We need to get this into a form (density, no_of_lanes, iterations, slowing_prob, lane_changing_prob) and array of these
    no_of_simulations = len(densities_to_test) * len(number_of_lanes_to_test) * len(no_of_iterations) * len(slowing_probabilities_to_test) * len(lane_changing_probabilities_to_test)

    # Need to decide what we are testing
    # We are testing and plotting slowing_probability so we are going to have len(slowing_probabilities_to_test) sets of data to plot
    no_of_plots = len(lane_changing_probabilities_to_test) # Change this to whatever you are testing

    # Since we are just testing slowing_probabilty everything else becomes a constant (apart from density)
    # Everything we are not testing should be a constant in array like [5] 
    lanes_array = np.tile(number_of_lanes_to_test, no_of_simulations)
    iterations_array = np.tile(no_of_iterations, no_of_simulations)
    slowing_array = np.tile(slowing_probabilities_to_test, no_of_simulations)
    #changing_array = np.tile(lane_changing_probabilities_to_test, no_of_simulations)

    # The densities need to be ran for each thing we are testing, i.e. [0,0.1,0.2,0.3,0.4,0.5] -> [0,0.1,0.2,0.3,0.4,0.5,0,0.1,0.2,0.3,0.4,0.5] if we were testing only two values
    density_array = np.tile(densities_to_test, no_of_plots)

    # For the thing we are testing that needs to be repeated for example if we are testing values [1,2] against 5 different densities we need array [1,1,1,1,1,2,2,2,2,2]
    changing_array = np.repeat(lane_changing_probabilities_to_test, len(densities_to_test))

    # All the parameters we are simulating looks like [(0,2,4,2,1), ...]
    parameter_list = list(zip(density_array, lanes_array, iterations_array, slowing_array, changing_array))

    with Pool(processes = 23) as pool:
        flow_rates_data = pool.starmap(simulation, parameter_list)

    split_arrays = np.array_split(flow_rates_data, no_of_plots)

    for i, flow_rates_data in enumerate(split_arrays):
        colour = (np.random.random(), np.random.random(), np.random.random()) 
        plt.scatter(densities_to_test, flow_rates_data, marker = 'x', color = colour, s=2)
        yhat = savgol_filter(flow_rates_data, 41, 2)
        #trend4 = np.polyfit(densities_to_test,flow_rates_data, 4)
        #trendpoly4 = np.poly1d(trend4)
        plt.plot(densities_to_test, yhat, label = f'p={round(lane_changing_probabilities_to_test[i], 1)}', color=colour) # Change here to change label you are testing
        plt.legend(fontsize = '22', loc = 'upper right')
        plt.title('Fundamental Traffic Flow-Density Graph',fontsize = 30)
        plt.xlabel('Density', fontsize = 30)
        plt.ylabel('Flow Rate', fontsize = 30)
        plt.grid(True)
plt.show()

print("DONE!")

 
# densities = np.linspace(0.01,0.5,100)
# number_of_lanes = 3
# flow_rates_data = []
# iterations = 100
# for density in densities:
#     if density > 0.5:
#         flow_rates_data.append(0)
#         continue
#     flow_count = 0
    
#     road = initialize_road_multi_lane(road_length, density, number_of_lanes)
#     for i in range(iterations):
#         if i < 1:
#             print(f"iteration {i + 1}:" + f"density{density}")
#             for j in range(number_of_lanes):
#                 print(''.join(x.pretty_print() for x in road[road_length*j:road_length*j + road_length - 1]))


#         positions_at_current_time = [1 if isinstance(cell.road_object,Car) else 0 for cell in road]
#         car_positons_over_time.append(positions_at_current_time)
        
#         average_velocity = calculate_average_velocity(road)
    
#         # print(f"iteration {i + 1}:" + f"avg velocity {average_velocity}")
#         # for j in range(3):
#         #     print(''.join(x.pretty_print() for x in road[road_length*j:road_length*j + road_length - 1]))
    
#         if i == time_to_add_obstacle:
#             road[obstacle_position].add_road_object(Obstacle())
#         if i == time_to_remove_obstacle:
#             road[obstacle_position].del_road_object()
            
#         #INCIDENT CHECK
#         if i == time_road_incident_true:
#             for i in range(road_length):
#                 if isinstance(road[i].road_object, Car):
#                     road[i].road_object.max_velocity = road_incident_velocity
#                     road[i].road_object.velocity = min(road_incident_velocity, road[i].road_object.velocity)
#                     break
#         if i == time_road_incident_false:
#             for i in range(road_length):
#                 if isinstance(road[i].road_object, Car):
#                     road[i].road_object.max_velocity = max_velocity_default
#                     break
#         #CAR BREAKDOWN
#         if i == car_breakdown_time:
#             for i in range(road_length):
#                 if isinstance(road[i].road_object, Car):
#                     if road[i].road_object.id == car_id_to_breakdown:
#                         road[i].road_object.max_velocity = car_breakdown_velocity
#                         road[i].road_object.velocity = min(car_breakdown_velocity, road[i].road_object.velocity)
#                         break
            
#         if i == car_fixed_time:
#             for i in range(road_length):
#                 if isinstance(road[i].road_object, Car):
#                     if road[i].road_object.id == car_id_to_breakdown:
#                         road[i].road_object.max_velocity = max_velocity_default        
#                         break
    
#         road, flow_count = road_update(road, flow_count,road_length)
    
#     flow_rates_data.append(flow_count / (iterations)) #*roadlength