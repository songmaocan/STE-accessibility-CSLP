from Data import Read_data
from gurobipy import *
import copy
class Solve:
    def __init__(self):
        #input
        self.multiplier=0
        data=Read_data(self.multiplier)
        self.node_list, self.link_list, self.candidate_charging_stations_nodes, self.OD_pair_list, \
        self.g_number_of_links, self.g_number_of_nodes, self.g_number_of_charging_stations, self.g_number_of_ODs,self.SP_list=data.read_candidate_charging_stations()

        self.construction_budget=6
        # self.construction_cost=1
        self.electric_quantity=6
        # self.charging_quality =10

        self.ratio_time_budget=1.2

        self.iteration_times=100
        self.acceptable_gap=0.005

        #DP parameter

        self.Best_K_Size=3

        #output
        self.local_LB = []
        self.local_UB = []
        self.global_LB = []
        self.global_UB = []
        self.solutions_of_KS_subproblem = []
        self.space_of_routing_subproblem=[] #state-space-time path
        self.time_of_routing_subproblem = []  # state-space-time path
        self.state_of_routing_subproblem = []  # state-space-time path
        self.charging_stations_flag_of_routing_subproblem=[]
        self.record_multiplier_miu = []

    def g_solving_the_charging_location_problem_by_LR(self):
        print("Solving...")
        for i in range(self.iteration_times):
            self.record_multiplier_miu.append([])
            self.space_of_routing_subproblem.append([])
            self.time_of_routing_subproblem.append([])
            self.state_of_routing_subproblem.append([])
            self.charging_stations_flag_of_routing_subproblem.append([])
            self.local_LB.append(0)
            self.local_UB .append(0)
            self.global_LB .append(-100000)
            self.global_UB .append(100000)
            print("Iteration: {}".format(i+1))

            #1.lower bound generation
            #1.1 solve routing subproblems
            candidate_travel_flag_list=[] # if the OD pair use these candidate charging locations
            for od_index in range(self.g_number_of_ODs):
                od_pair=self.OD_pair_list[od_index]
                self.g_solving_routing_subproblem(od_index,1)
                self.local_LB[i] +=self.g_ending_state_vector.state_vector[0].Label_cost_for_LR
                candidate_travel_flag_list.append(self.g_ending_state_vector.state_vector[0].m_visit_charging_nodes)

            #1.2 solve knapsack subproblem
            solution, obj = self.g_solving_KS()
            self.solutions_of_KS_subproblem.append(solution)
            self.local_LB[i] += obj

            #2. upper bound generation
            #Apply the KS solution(node.construction_flag)
            for candidate_node_id in range(self.g_number_of_charging_stations):
                node=self.candidate_charging_stations_nodes[candidate_node_id]
                if solution[candidate_node_id]==1:
                    node.construction_flag=1
                if solution[candidate_node_id]==0:
                    node.construction_flag=0

            for od_index in range(self.g_number_of_ODs):
                od_pair=self.OD_pair_list[od_index]
                self.g_solving_routing_subproblem(od_index,2)
                self.local_UB[i]+=self.g_ending_state_vector.state_vector[0].Primal_label_cost

                # output
                self.space_of_routing_subproblem[i].append(self.g_ending_state_vector.state_vector[0].m_visit_node_seq)
                self.time_of_routing_subproblem[i].append(self.g_ending_state_vector.state_vector[0].m_visit_time_seq)
                self.state_of_routing_subproblem[i].append(self.g_ending_state_vector.state_vector[0].m_visit_state_seq)
                self.charging_stations_flag_of_routing_subproblem[i].append(self.g_ending_state_vector.state_vector[0].m_visit_charging_nodes)

            #3. update multiplier
            #3.1 update the global bounds and obtain the step size
            if i==0:
                self.global_LB[i]=self.local_LB[i]
                self.global_UB[i] = self.local_UB[i]
            else:
                self.global_LB[i] = max(self.local_LB[i],self.global_LB[i-1])
                self.global_UB[i] = min(self.local_UB[i],self.global_UB[i-1])

            print(self.global_UB[i])
            print(self.global_LB[i])

            #3.2 subgradient method
            a=self.global_UB[i]-self.local_LB[i]
            b=0
            for k in range(self.g_number_of_ODs):
                traveling_flag=candidate_travel_flag_list[k]
                for j in range(self.g_number_of_charging_stations):
                    b+=(traveling_flag[j]-solution[j])**2
            if b!=0:
                step_size=a/b
            else:
                step_size=0.01

            step_size = 1 / (i + 1)
            for j in range(self.g_number_of_charging_stations):
                node = self.candidate_charging_stations_nodes[j]
                for k in range(self.g_number_of_ODs):
                    traveling_flag=candidate_travel_flag_list[k]
                    #output
                    self.record_multiplier_miu[i].append(node.base_profit_for_lagrangian)
                    node.base_profit_for_lagrangian[k]+=step_size*(traveling_flag[j]-solution[j])
                    if node.base_profit_for_lagrangian[k]<0:
                        node.base_profit_for_lagrangian[k]=0

            #TERMINAL CONDITION
            if self.global_UB[i]!=0:
                gap=(self.global_UB[i]-self.global_LB[i])/self.global_UB[i]
                if gap<=self.acceptable_gap:
                    self.max_iteration=i+1
                    break
            self.max_iteration=i+1


    def g_solving_KS(self):
        self.KS=Model("KS")
        self.KS.setParam('OutputFlag',0)
        #obj
        expr=LinExpr()
        for charging_station_index in range(self.g_number_of_charging_stations):
            charging_node=self.candidate_charging_stations_nodes[charging_station_index]
            name="y_{}".format(charging_station_index)
            name=self.KS.addVar(vtype=GRB.BINARY, name=name)
            value=-sum(charging_node.base_profit_for_lagrangian)
            expr.addTerms(value,name)
        self.KS.setObjective(expr,GRB.MINIMIZE)
        self.KS.update()
        #budget constraint
        expr=LinExpr()
        for charging_station_index in range(self.g_number_of_charging_stations):
            charging_node=self.candidate_charging_stations_nodes[charging_station_index]
            name=self.KS.getVarByName(name="y_{}".format(charging_station_index))
            expr.addTerms(1,name)
        self.KS.addConstr(expr,GRB.LESS_EQUAL,self.construction_budget,name="budget")
        self.KS.optimize()
        obj=self.KS.objval
        values=self.KS.getVars()
        solution=[]
        for value in values:
            solution.append(value.x)
        return solution,obj #return constrcution decisions and obj


    def g_solving_routing_subproblem(self,od_index,flag):#core
        """
       flag=1:LR cost
       flag=2:primal cost
        """
        self.time_budget=int(self.SP_list[od_index]*self.ratio_time_budget)
        self.g_ending_state_vector = []
        self.g_time_dependent_state_vector = []
        for t in range(self.time_budget + 1):
            self.g_time_dependent_state_vector.append([None] * self.g_number_of_nodes)

        ending_flag=0
        OD_pair=self.OD_pair_list[od_index]
        #Initialization
        for t in range(0,self.time_budget+1):
            for n in range(self.g_number_of_nodes):
                self.g_time_dependent_state_vector[t][n]=C_time_indexed_state_vector()
                self.g_time_dependent_state_vector[t][n].Reset()
                self.g_time_dependent_state_vector[t][n].current_time=t
                self.g_time_dependent_state_vector[t][n].current_node=n
        self.g_ending_state_vector=C_time_indexed_state_vector()
        #new element 1: for the origin node
        element=CVSState(self.g_number_of_charging_stations)
        element.current_node=OD_pair[0]
        element.current_time=0
        element.current_state=self.electric_quantity
        element.m_visit_node_seq.append(OD_pair[0])
        element.m_visit_time_seq.append(0)
        element.m_visit_state_seq.append(self.electric_quantity)
        self.g_time_dependent_state_vector[0][OD_pair[0]].update_state(element,flag)
        #new element 2: for the ending node
        new_element = CVSState(self.g_number_of_charging_stations)
        new_element.my_copy(element)
        new_element.current_node = OD_pair[1]
        new_element.current_time = 1
        new_element.current_state = self.electric_quantity
        new_element.m_visit_node_seq.append(OD_pair[1])
        new_element.m_visit_time_seq.append(1)
        new_element.m_visit_state_seq.append(self.electric_quantity)
        new_element.Primal_label_cost=1
        new_element.Label_cost_for_LR=1
        self.g_ending_state_vector.state_vector.append(new_element)
        # self.g_time_dependent_state_vector[1][OD_pair[1]].update_state(element)
        #DP gogogo
        for t in range(self.time_budget):
            if ending_flag==1:
                break
            for n in range(self.g_number_of_nodes):
                self.g_time_dependent_state_vector[t][n].Sort(flag)
                max_num=len(self.g_time_dependent_state_vector[t][n].state_vector)
                for index in range(min(self.Best_K_Size,max_num)):
                    pElement=self.g_time_dependent_state_vector[t][n].state_vector[index]
                    from_node_id=pElement.current_node
                    from_node=self.node_list[from_node_id]
                    #neigbors
                    for i in range(from_node.outbound_nodes_number):
                        to_node_id=from_node.outbound_nodes_list[i]
                        to_node=self.node_list[to_node_id]
                        link_to=from_node.outbound_links_list[i]
                        next_time=t+link_to.travel_time
                        #time resource
                        if next_time>self.time_budget:
                            continue
                        # electricty resource
                        electricity=pElement.current_state-link_to.consumed_electricity
                        if electricity<0:
                            continue

                        #destination node
                        if to_node_id==OD_pair[1]:
                            new_element=CVSState(self.g_number_of_charging_stations)
                            new_element.my_copy(pElement)
                            new_element.current_node=OD_pair[1]
                            new_element.current_time=next_time
                            new_element.current_state-=link_to.consumed_electricity
                            new_element.m_visit_node_seq.append(OD_pair[1])
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(new_element.current_state)
                            new_element.calculate_label_cost(0,0)
                            self.g_ending_state_vector.state_vector.append(new_element)

                            if flag==2 and new_element.Primal_label_cost==0:
                                ending_flag=1
                            if flag==1 and new_element.Label_cost_for_LR==0:
                                ending_flag=1

                            continue

                        # transportation node
                        # if link_to.link_type==0:
                        if to_node.node_type==0: #transportation
                            new_element=CVSState(self.g_number_of_charging_stations)
                            new_element.my_copy(pElement)
                            new_element.current_node = to_node_id
                            new_element.current_time = next_time
                            new_element.current_state -= link_to.consumed_electricity
                            new_element.m_visit_node_seq.append(to_node_id)
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(new_element.current_state)
                            new_element.calculate_label_cost(0, 0)
                            self.g_time_dependent_state_vector[next_time][to_node_id].update_state(new_element,flag)
                            continue

                        #charging node
                        # if link_to.link_type!=0:
                        if to_node.node_type!=0:
                            charging_station_index=to_node.node_type #from 1
                            # if not constructed and the DP-2
                            if flag==2 and to_node.construction_flag==0:
                                continue #not constructed
                            if new_element.m_visit_charging_nodes[charging_station_index-1]==1:
                                continue #has been used

                            new_element=CVSState(self.g_number_of_charging_stations)
                            new_element.my_copy(pElement)
                            new_element.current_node = to_node_id
                            new_element.current_time = next_time
                            new_element.current_state = self.electric_quantity
                            new_element.m_visit_charging_nodes[charging_station_index-1]=1
                            # new_element.m_visit_charging_nodes.append(link_to.link_type)
                            new_element.m_visit_node_seq.append(to_node_id)
                            new_element.m_visit_time_seq.append(next_time)
                            new_element.m_visit_state_seq.append(new_element.current_state)
                            #find the multiplier
                            multiplier=to_node.base_profit_for_lagrangian[od_index]
                            new_element.calculate_label_cost(0,multiplier)
                            self.g_time_dependent_state_vector[next_time][to_node_id].update_state(new_element,flag)
                            continue
        self.g_ending_state_vector.Sort(flag)

    def output_results(self,spend_time):
        with open("output_gap.csv","w") as fl:
            fl.write("iteration,local_LB,local_UB,LB,UB,gap\n")
            for i in range(len(self.global_UB)):
                local_LB=round(self.local_LB[i],3)
                local_UB=round(self.local_UB[i],3)
                LB=round(self.global_LB[i],3)
                UB=round(self.global_UB[i],3)
                gap=0
                if UB!=0:
                    gap=round((UB-LB)/UB,3)
                fl.write(str(i+1)+","+str(local_LB)+","+str(local_UB)+","+str(LB)+","+str(UB)+","+str(gap)+"\n")
            fl.write("CPU time: {}".format(spend_time))

        with open("outout_solution_of_KS.txt","w") as fl:
            fl.write("iteration,solution\n")
            for i in range(len(self.global_UB)):
                result=self.solutions_of_KS_subproblem[i]
                fl.write(str(i + 1) + ",")
                fl.write(str(result) + "\n")

        with open("output_multiplier.csv","w") as fl:
            fl.write("iteration,")
            for k in range(self.g_number_of_ODs):
                for linkid in range(self.g_number_of_charging_stations):
                    fl.write("{}_{},".format(k,linkid))
            fl.write("\n")
            for i in range(len(self.global_UB)):
                fl.write(str(i+1)+",")
                for k in range(self.g_number_of_ODs):
                    for linkid in range(self.g_number_of_charging_stations):
                        multiplier=self.record_multiplier_miu[i][linkid][k]
                        fl.write(str(multiplier)+",")
                fl.write("\n")


        with open("output_solution_of_routing_subproblems.txt","w") as fl:
            fl.write("iteration,OD,space,time,state,charging_flag\n")
            for i in range(len(self.global_UB)):
                for k in range(self.g_number_of_ODs):
                    fl.write(str(i)+","+str(k)+",")
                    space=self.space_of_routing_subproblem[i][k]
                    time = self.time_of_routing_subproblem[i][k]
                    state = self.state_of_routing_subproblem[i][k]
                    flag = self.charging_stations_flag_of_routing_subproblem[i][k]
                    fl.write(str(space)+","+str(time)+","+str(state)+","+str(flag)+"\n")


class C_time_indexed_state_vector:
    def __init__(self):
        self.current_time=0
        self.current_node=0
        self.state_vector=[]
        self.state_map=[]

    def Reset(self):
        self.current_time =0
        self.current_node =0
        self.state_vector =[]
        self.state_map =[]

    def m_find_state_index(self,string_key):
        if string_key in self.state_map:
            return self.state_map.index(string_key)
        else:
            return -1

    def update_state(self,element,flag):
        #LR cost：flag=1
        if flag==1:
            string_key=element.generate_string_key()
            state_index=self.m_find_state_index(string_key)
            if state_index == -1:
                self.state_vector.append(element)
                self.state_map.append(string_key)
            else:
                if element.Label_cost_for_LR<self.state_vector[state_index].Label_cost_for_LR:
                    self.state_vector[state_index]=element

        #primal cost: only keep one state
        if flag==2:
            if self.state_vector==[]:
                self.state_vector.append(element)
            else:
                if element.current_state>self.state_vector[0].current_state:
                    self.state_vector[0]=element

### flag: generalized routing cost
    def Sort(self,flag):
        if flag==1:
            self.state_vector=sorted(self.state_vector,key=lambda x:x.Label_cost_for_LR)

        if flag==2:
            self.state_vector=sorted(self.state_vector,key=lambda x:x.current_state)


class CVSState:
    def __init__(self,charging_station_num):
        self.current_node=None
        self.current_time=None
        self.current_state=None
        self.m_visit_node_seq=[]
        self.m_visit_time_seq = []
        self.m_visit_state_seq = []
        self.m_visit_charging_nodes=[0]*charging_station_num
### flag: generalized routing cost
        self.Primal_label_cost=0
        self.Label_cost_for_LR=0

    def generate_string_key(self):
        str=self.current_state
        return str

    def my_copy(self,pElement):
        self.current_node =copy.deepcopy(pElement.current_node)
        self.current_time = copy.deepcopy(pElement.current_time)
        self.current_state = copy.deepcopy(pElement.current_state)
        self.m_visit_node_seq = copy.deepcopy(pElement.m_visit_node_seq)
        self.m_visit_time_seq = copy.deepcopy(pElement.m_visit_time_seq)
        self.m_visit_state_seq = copy.deepcopy(pElement.m_visit_state_seq)
        self.m_visit_charging_nodes = copy.deepcopy(pElement.m_visit_charging_nodes)
### flag: generalized routing cost
        self.Primal_label_cost = copy.deepcopy(pElement.Primal_label_cost)
        self.Label_cost_for_LR = copy.deepcopy(pElement.Label_cost_for_LR)

    def calculate_label_cost(self,primal_cost,multiplier):
        #except for the dummy link (primal-cost=1), other primal cost equal 0
        #the cost value
        self.Primal_label_cost+=primal_cost
        self.Label_cost_for_LR+=primal_cost+multiplier
### flag: generalized routing cost