# **Optimization of Azimuth Only Passive Location for Conventional Formation of Unmanned Aerial Vehicles**

# **Abstract**

This article first uses the position estimation method of multi round optimization algorithm to obtain angle information from the signals transmitted and received by each drone. The quadratic optimization method is used to obtain the optimal solution for each round, which is compared with the standard template. After multiple rounds of adjustment, the position estimation accuracy of the drone formation is improved. The results show that using the optimization method proposed in this article and limited data, the positioning performance of unmanned aerial vehicle formations has been significantly improved. The standard deviation of relative positions between various drones in the formation decreases, and after multiple iterations, the standard deviation tends to 0.07, which can be ignored in real life.

**Keywords: Drone Formation; Quadratic Optimization Algorithm; Bearings-only Passive Localization**



1. **Research background and purpose**
   Formation flying refers to a group of unmanned aerial vehicles working together according to specific geometric shapes and motion rules to achieve a common mission or goal. Drone formation technology has a wide range of applications in military, civilian, scientific research, and entertainment fields, but it also faces various challenges and problems. For example, in a drone formation, each aircraft needs to exchange information and coordinate actions in real time. The problem lies in how to ensure reliable communication links and how to design effective collaborative algorithms that enable collaborative actions between drones without collisions or confusion. This article uses a universal adjustment strategy for pure azimuth passive positioning, with two fixed drones transmitting signals per round and the remaining drones receiving signals and adjusting once. After multiple rounds of adjustment, the formation is adjusted.

   

2. **Research Methods**
   2.1 Problem Restatement
   This article uses the function of sending shuttle signals between drones and the principle of similarity in transmission and reception angles to determine the position deviation of drones and adjust it based on the known initial position and standard template.
   2.2 Problem solving
   Firstly, select a general formation of unmanned aerial vehicles, with a total of drones. This article selects a basic rectangle as the template and adds random errors to the original template to obtain the initial drone coordinates. Plot Table 1 and Table 2 to obtain intuitive positions.

   Then fix two drones as transmission signal sources, denoted as $n_1,n_2$ , and the remaining drones as reception signals. In each round of adjustment, $n_1$ collaborates with another unmanned aerial vehicle (non $n_2$), referred to as the launch crew $n_1,j$. The remaining $n-2$  drones $j$  receive signals, and the angle seen from the observation angle of the receiving drone is recorded as $\alpha_{ij}$. After the same operation of the $n_2$  drone, the angle is recorded as $\beta_{ij}$. When any two of the three numbers, $n_1,i,j$ are equal, $\alpha_{ij}=\beta_{ij}=0$, these two angles will be used to optimize the algorithm based on the following constraints. Record the fixed coded $i$ drone position coordinates as $Q_i(\hat{x}_i,\hat{y}_i),j=1,2,...,n.$

   Let $\widehat{\alpha}_{ij}=\angle Q_{n1}Q_{i}Q_{j},\widehat{\beta}_{ij}=\angle Q_{n2}Q_{i}Q_{j}$，when the objective function(below) 
   $$
   argmin\sum^{n}[(\widehat{\alpha}_{ij}-\alpha_{ij})^{2}+(\widehat{\beta}_{ij}-\beta_{ij})^{2}]
   $$
   
   
   is minimized, the solution can be obtained as $Q_i(\hat{x}_i,\hat{y}_i),j=1,2,...,n.$
   
   The final adjustment amount is calculated:
   $$
   \Delta P(\Delta x,\Delta y)=M_j(x_j,y_j)-Q_i(\hat{x}_i,\hat{y}_i)
   $$
   The adjusted image is as follows:

![TCPD](D:\AAA_studyfiles\Upload_GitHub\wurenji\assets\TCPD.png)

Obtain the curve of standard deviation with iteration times. From this, it can be concluded that the standard deviation can be reduced to a lower level after 15 iterations, basically achieving formation adjustment.

![](D:\AAA_studyfiles\Upload_GitHub\wurenji\assets\itreation.png)





**3 Summary**

At present, algorithms can achieve stability and lower standard deviation after fewer iterations, but can only handle simple center stacked graphics. For more complex graphics, further algorithm modifications are needed. At the same time, the larger the sample size, the longer the iteration time, and further algorithm improvement is needed to achieve better optimization.

















