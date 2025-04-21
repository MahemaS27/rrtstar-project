## S CURVE


Total time: 0.686465 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: RRT_star at line 204

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
   204                                               @profile                                                                      
   205                                               def RRT_star(self, n_pts, neighbor_size, name):                               
   206                                                                                                                             
   207         1        104.0    104.0      0.0          map_flat = self.map_array.flatten().astype(np.uint8)                      
   208                                                   # c_map = map_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))         
   209                                                                                                                             
   210                                                   # # Call the C function                                                   
   211                                                   # result = self.rrt_star_lib.run_rrt_star(                                
   212                                                   #     c_map,                                                              
   213                                                   #     self.size_row,                                                      
   214                                                   #     self.size_col,                                                      
   215                                                   #     self.start.row,                                                     
   216                                                   #     self.start.col,                                                     
   217                                                   #     self.goal.row,                                                      
   218                                                   #     self.goal.col,                                                      
   219                                                   #     n_pts,                                                              
   220                                                   #     neighbor_size,                                                      
   221                                                   #     name.encode('utf-8')                                                
   222                                                   # )                                                                       
   223                                                                                                                             
   224                                           #         if result == 1:                                                         
   225                                           #             print("RRT* path found (from C)")                                   
   226                                           #             self.found = True                                                   
   227                                           # #             self.visualize_map("RRT*")                                        
   228                                           #         else:                                                                   
   229                                           #             print("No path found (from C)")                                     
   230                                           # #             self.visualize_map("RRT*")                                        
   231                                                                                                                             
   232                                                   # RRT* algorithm implementation                                           
   233         1          3.0      3.0      0.0          self.init_map()                                                           
   234         1         31.0     31.0      0.0          print('init map')                                                         
   235         1          0.0      0.0      0.0          goal1 = False                                                             
   236         1          0.0      0.0      0.0          goal_bias = 4                                                             
   237         1          0.0      0.0      0.0          step = 10                                                                 
   238         1          1.0      1.0      0.0          start_time = time.time()                                                  
   239      5001        616.0      0.1      0.1          for i in range(n_pts):                                                    
   240      5000      19771.0      4.0      2.9              new_sample = self.get_new_sample_point(goal_bias)                     
   241      5000       2453.0      0.5      0.4              if self.map_array[new_sample.row][new_sample.col] == 0:               
   242      4348        330.0      0.1      0.0                  continue                                                          
   243                                                       else:                                                                 
   244       652      45085.0     69.1      6.6                  nearest_node = self.get_nearest_neighbor(new_sample)              
   245       652         52.0      0.1      0.0                  if goal1:                                                         
   246         7          0.0      0.0      0.0                      new_node = self.goal                                          
   247                                                           else:                                                             
   248       645       1430.0      2.2      0.2                      new_node = self.extend_towards_point(nearest_node, new_sample)
   249       645      16913.0     26.2      2.5                      collision = self.check_collision(new_node, nearest_node)      
   250       652         62.0      0.1      0.0                  if collision:                                                     
   251       396     313194.0    790.9     45.6                      neighbors = self.find_neighbors(new_node, neighbor_size)      
   252       396     285878.0    721.9     41.6                      self.rewire_neighbors(new_node, neighbors)                    
   253       396        274.0      0.7      0.0                      dist = self.euclidean_distance(new_node, self.goal)           
   254       396         39.0      0.1      0.0                      if dist <= step:                                              
   255        97          9.0      0.1      0.0                          goal1 = True                                              
   256       396        113.0      0.3      0.0                      self.vertices.append(new_node)                                
   257                                                                                                                             
   258       396         42.0      0.1      0.0                      if dist == 0:                                                 
   259        90         17.0      0.2      0.0                          self.found = True                                         
   260        90          8.0      0.1      0.0                          goal1 = False                                             
   261         1         20.0     20.0      0.0          print('finishloop')                                                       
   262         1          1.0      1.0      0.0          if self.found:                                                            
   263         1          2.0      2.0      0.0              print(name)                                                           
   264         1          0.0      0.0      0.0              steps = len(self.vertices) - 2                                        
   265         1          1.0      1.0      0.0              length = self.goal.cost                                               
   266         1          4.0      4.0      0.0              print("It took %d nodes to find the path using RRT*" % steps)         
   267         1          5.0      5.0      0.0              print("The length of path is %.2f" % length)                          
   268         1          1.0      1.0      0.0              end_time = time.time()                                                
   269         1          1.0      1.0      0.0              elapsed_time = end_time - start_time                                  
   270         1          5.0      5.0      0.0              print(f"Elapsed time: {elapsed_time} seconds")                        
   271                                                      # self.visualize_map("RRT*")                                           
   272                                                   else:                                                                     
   273                                                       print(name)                                                           
   274                                                       print("No path found")                                                
   275                                                       end_time = time.time()                                                
   276                                                       elapsed_time = end_time - start_time                                  
   277                                                       print(f"Elapsed time: {elapsed_time} seconds")                        
   278                                                       #self.visualize_map("RRT*")                                           


## Curve with Dent
Total time: 2.44017 s
File: /Users/mahema.singh/Desktop/CV/rrtstar-project/RRT.py
Function: RRT_star at line 204

Line #      Hits         Time  Per Hit   % Time  Line Contents
==============================================================
   204                                               @profile                                                                      
   205                                               def RRT_star(self, n_pts, neighbor_size, name):                               
   206                                                                                                                             
   207         1         97.0     97.0      0.0          map_flat = self.map_array.flatten().astype(np.uint8)                      
   208                                                   # c_map = map_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))         
   209                                                                                                                             
   210                                                   # # Call the C function                                                   
   211                                                   # result = self.rrt_star_lib.run_rrt_star(                                
   212                                                   #     c_map,                                                              
   213                                                   #     self.size_row,                                                      
   214                                                   #     self.size_col,                                                      
   215                                                   #     self.start.row,                                                     
   216                                                   #     self.start.col,                                                     
   217                                                   #     self.goal.row,                                                      
   218                                                   #     self.goal.col,                                                      
   219                                                   #     n_pts,                                                              
   220                                                   #     neighbor_size,                                                      
   221                                                   #     name.encode('utf-8')                                                
   222                                                   # )                                                                       
   223                                                                                                                             
   224                                           #         if result == 1:                                                         
   225                                           #             print("RRT* path found (from C)")                                   
   226                                           #             self.found = True                                                   
   227                                           # #             self.visualize_map("RRT*")                                        
   228                                           #         else:                                                                   
   229                                           #             print("No path found (from C)")                                     
   230                                           # #             self.visualize_map("RRT*")                                        
   231                                                                                                                             
   232                                                   # RRT* algorithm implementation                                           
   233         1          2.0      2.0      0.0          self.init_map()                                                           
   234         1         18.0     18.0      0.0          print('init map')                                                         
   235         1          0.0      0.0      0.0          goal1 = False                                                             
   236         1          0.0      0.0      0.0          goal_bias = 4                                                             
   237         1          0.0      0.0      0.0          step = 10                                                                 
   238         1          1.0      1.0      0.0          start_time = time.time()                                                  
   239      5001        660.0      0.1      0.0          for i in range(n_pts):                                                    
   240      5000      24149.0      4.8      1.0              new_sample = self.get_new_sample_point(goal_bias)                     
   241      5000       2302.0      0.5      0.1              if self.map_array[new_sample.row][new_sample.col] == 0:               
   242      4143        318.0      0.1      0.0                  continue                                                          
   243                                                       else:                                                                 
   244       857     126171.0    147.2      5.2                  nearest_node = self.get_nearest_neighbor(new_sample)              
   245       857         82.0      0.1      0.0                  if goal1:                                                         
   246         9          2.0      0.2      0.0                      new_node = self.goal                                          
   247                                                           else:                                                             
   248       848       2431.0      2.9      0.1                      new_node = self.extend_towards_point(nearest_node, new_sample)
   249       848      28673.0     33.8      1.2                      collision = self.check_collision(new_node, nearest_node)      
   250       857        107.0      0.1      0.0                  if collision:                                                     
   251       709    1175108.0   1657.4     48.2                      neighbors = self.find_neighbors(new_node, neighbor_size)      
   252       709    1078927.0   1521.8     44.2                      self.rewire_neighbors(new_node, neighbors)                    
   253       709        528.0      0.7      0.0                      dist = self.euclidean_distance(new_node, self.goal)           
   254       709         74.0      0.1      0.0                      if dist <= step:                                              
   255       218         19.0      0.1      0.0                          goal1 = True                                              
   256       709        267.0      0.4      0.0                      self.vertices.append(new_node)                                
   257                                                                                                                             
   258       709        103.0      0.1      0.0                      if dist == 0:                                                 
   259       209         42.0      0.2      0.0                          self.found = True                                         
   260       209         49.0      0.2      0.0                          goal1 = False                                             
   261         1         22.0     22.0      0.0          print('finishloop')                                                       
   262         1          1.0      1.0      0.0          if self.found:                                                            
   263         1          2.0      2.0      0.0              print(name)                                                           
   264         1          1.0      1.0      0.0              steps = len(self.vertices) - 2                                        
   265         1          1.0      1.0      0.0              length = self.goal.cost                                               
   266         1          5.0      5.0      0.0              print("It took %d nodes to find the path using RRT*" % steps)         
   267         1          4.0      4.0      0.0              print("The length of path is %.2f" % length)                          
   268         1          1.0      1.0      0.0              end_time = time.time()                                                
   269         1          1.0      1.0      0.0              elapsed_time = end_time - start_time                                  
   270         1          7.0      7.0      0.0              print(f"Elapsed time: {elapsed_time} seconds")                        
   271                                                      # self.visualize_map("RRT*")                                           
   272                                                   else:                                                                     
   273                                                       print(name)                                                           
   274                                                       print("No path found")                                                
   275                                                       end_time = time.time()                                                
   276                                                       elapsed_time = end_time - start_time                                  
   277                                                       print(f"Elapsed time: {elapsed_time} seconds")                        
   278                                                       #self.visualize_map("RRT*")                                           
