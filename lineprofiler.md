
## Curve with dent


Total time: 0.274159 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: euclidean_distance at line 58

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    58                                               @profile                                                                             
    59                                               def euclidean_distance(self, node1, node2):                                          
    60                                                   # Calculate Euclidean distance between two nodes                                 
    61    667372     224879.0      0.3     82.0          distance = math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
    62    667372      49280.0      0.1     18.0          return distance                                                                  


Total time: 2.70027 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: check_collision at line 64

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    64                                               @profile                                                                               
    65                                               def check_collision(self, node1, node2):                                               
    66                                                   # Check if there is a collision between two nodes by checking the points in between
    67    146560     576189.0      3.9     21.3          points_between = zip(np.linspace(node1.row, node2.row, dtype=int),                 
    68     73280     540462.0      7.4     20.0                               np.linspace(node1.col, node2.col, dtype=int))                 
    69   3715338     492997.0      0.1     18.3          for point in points_between:                                                       
    70   3642680    1084379.0      0.3     40.2              if self.map_array[point[0]][point[1]] == 0:                                    
    71       622         51.0      0.1      0.0                  return False                                                               
    72     72658       6192.0      0.1      0.2          return True                                                                        


Total time: 0.022274 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: get_new_sample_point at line 74

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    74                                               @profile                                                                                   
    75                                               def get_new_sample_point(self, goal_bias):                                                 
    76                                                   # Generate a new sample point, with a probability (goal_bias) of picking the goal point
    77      5000       4986.0      1.0     22.4          a = randrange(100)                                                                     
    78      5000        542.0      0.1      2.4          if a <= goal_bias:                                                                     
    79       228         37.0      0.2      0.2              return self.goal                                                                   
    80                                                   else:                                                                                  
    81      4772       7786.0      1.6     35.0              p1rows = random.randint(self.size_row)                                             
    82      4772       6081.0      1.3     27.3              p1cols = random.randint(self.size_col)                                             
    83      4772       2394.0      0.5     10.7              new_point = TreeNode(p1rows, p1cols)                                               
    84      4772        448.0      0.1      2.0              return new_point                                                                   


Total time: 1.90249 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: rewire_neighbors at line 98

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    98                                               @profile                                                                                                  
    99                                               def rewire_neighbors(self, new_node, neighbors):                                                          
   100                                                   # Rewire neighbors of a new node to improve the path                                                  
   101       766         90.0      0.1      0.0          checkdis = []                                                                                         
   102       766         63.0      0.1      0.0          free = []                                                                                             
   103     36717       4772.0      0.1      0.3          for neighbor in neighbors:                                                                            
   104     35951    1747594.0     48.6     91.9              if not self.check_collision(neighbor, new_node):                                                  
   105                                                           continue                                                                                      
   106     35951       6692.0      0.2      0.4              free.append(neighbor)                                                                             
   107                                                                                                                                                         
   108     36717       3540.0      0.1      0.2          for neighbor in free:                                                                                 
   109     35951      60261.0      1.7      3.2              cost1 = neighbor.cost + self.euclidean_distance(neighbor, new_node)                               
   110     35951       5603.0      0.2      0.3              checkdis.append(cost1)                                                                            
   111                                                                                                                                                         
   112       766        676.0      0.9      0.0          minindex = checkdis.index(min(checkdis))                                                              
   113       766        169.0      0.2      0.0          new_node.parent = neighbors[minindex]                                                                 
   114       766       1560.0      2.0      0.1          new_node.cost = neighbors[minindex].cost + int(self.euclidean_distance(neighbors[minindex], new_node))
   115                                                                                                                                                         
   116     36717       3748.0      0.1      0.2          for neighbor in free:                                                                                 
   117     35951       3123.0      0.1      0.2              initcost = neighbor.cost                                                                          
   118     35951      60966.0      1.7      3.2              rewiredcost = int(self.euclidean_distance(new_node, neighbor)) + new_node.cost                    
   119                                                                                                                                                         
   120     35951       3458.0      0.1      0.2              if initcost > rewiredcost:                                                                        
   121       574        108.0      0.2      0.0                  neighbor.parent = new_node                                                                    
   122       574         62.0      0.1      0.0                  neighbor.cost = rewiredcost                                                                   





# S curve

Total time: 0.090116 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: euclidean_distance at line 58

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    58                                               @profile                                                                             
    59                                               def euclidean_distance(self, node1, node2):                                          
    60                                                   # Calculate Euclidean distance between two nodes                                 
    61    214588      74037.0      0.3     82.2          distance = math.sqrt((node1.row - node2.row) ** 2 + (node1.col - node2.col) ** 2)
    62    214588      16079.0      0.1     17.8          return distance                                                                  


Total time: 0.928865 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: check_collision at line 64

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    64                                               @profile                                                                               
    65                                               def check_collision(self, node1, node2):                                               
    66                                                   # Check if there is a collision between two nodes by checking the points in between
    67     50438     197902.0      3.9     21.3          points_between = zip(np.linspace(node1.row, node2.row, dtype=int),                 
    68     25219     185838.0      7.4     20.0                               np.linspace(node1.col, node2.col, dtype=int))                 
    69   1274556     168182.0      0.1     18.1          for point in points_between:                                                       
    70   1249581     374816.0      0.3     40.4              if self.map_array[point[0]][point[1]] == 0:                                    
    71       244         18.0      0.1      0.0                  return False                                                               
    72     24975       2109.0      0.1      0.2          return True                                                                        


Total time: 0.020737 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: get_new_sample_point at line 74

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    74                                               @profile                                                                                   
    75                                               def get_new_sample_point(self, goal_bias):                                                 
    76                                                   # Generate a new sample point, with a probability (goal_bias) of picking the goal point
    77      5000       4627.0      0.9     22.3          a = randrange(100)                                                                     
    78      5000        562.0      0.1      2.7          if a <= goal_bias:                                                                     
    79       266         38.0      0.1      0.2              return self.goal                                                                   
    80                                                   else:                                                                                  
    81      4734       6891.0      1.5     33.2              p1rows = random.randint(self.size_row)                                             
    82      4734       5697.0      1.2     27.5              p1cols = random.randint(self.size_col)                                             
    83      4734       2513.0      0.5     12.1              new_point = TreeNode(p1rows, p1cols)                                               
    84      4734        409.0      0.1      2.0              return new_point                                                                   


Total time: 0.648311 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: rewire_neighbors at line 98

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
    98                                               @profile                                                                                                  
    99                                               def rewire_neighbors(self, new_node, neighbors):                                                          
   100                                                   # Rewire neighbors of a new node to improve the path                                                  
   101       422         56.0      0.1      0.0          checkdis = []                                                                                         
   102       422         44.0      0.1      0.0          free = []                                                                                             
   103     12701       1442.0      0.1      0.2          for neighbor in neighbors:                                                                            
   104     12279     595246.0     48.5     91.8              if not self.check_collision(neighbor, new_node):                                                  
   105                                                           continue                                                                                      
   106     12279       2290.0      0.2      0.4              free.append(neighbor)                                                                             
   107                                                                                                                                                         
   108     12701       1161.0      0.1      0.2          for neighbor in free:                                                                                 
   109     12279      20455.0      1.7      3.2              cost1 = neighbor.cost + self.euclidean_distance(neighbor, new_node)                               
   110     12279       1975.0      0.2      0.3              checkdis.append(cost1)                                                                            
   111                                                                                                                                                         
   112       422        294.0      0.7      0.0          minindex = checkdis.index(min(checkdis))                                                              
   113       422         94.0      0.2      0.0          new_node.parent = neighbors[minindex]                                                                 
   114       422        832.0      2.0      0.1          new_node.cost = neighbors[minindex].cost + int(self.euclidean_distance(neighbors[minindex], new_node))
   115                                                                                                                                                         
   116     12701       1296.0      0.1      0.2          for neighbor in free:                                                                                 
   117     12279       1099.0      0.1      0.2              initcost = neighbor.cost                                                                          
   118     12279      20861.0      1.7      3.2              rewiredcost = int(self.euclidean_distance(new_node, neighbor)) + new_node.cost                    
   119                                                                                                                                                         
   120     12279       1063.0      0.1      0.2              if initcost > rewiredcost:                                                                        
   121       269         74.0      0.3      0.0                  neighbor.parent = new_node                                                                    
   122       269         29.0      0.1      0.0                  neighbor.cost = rewiredcost                                                                   
