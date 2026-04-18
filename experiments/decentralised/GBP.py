import numpy as np

class GBP():
    def __init__(self, variables: list):
        """
        Variables must have this kind of format:
        [
            'X0',
            'X1',
            'U',
            'M'
        ]
        (list of len 1 if a single variable)
        """
        self.variables = variables
        self.factors = {}

        # Messages: (factor -> variable)
        self.messages_fv = {} #f -> v
        self.messages_vf = {} #v -> f

        

    def add_factor(self, variables: np.ndarray, eta: np.ndarray, 
                   Lambda: np.ndarray, name: str):
        
        eta = np.array(eta) #in case it was passed as regular list
        Lambda = np.array(Lambda)
        
        assert eta.shape[1] == 1 #intial_values must be a column matrix
        
        # eta = Lambda @ intial_values

        #warning if factor already exists
        if name in self.factors:
            print(f'(GBP) WARNING: factor {name} has been overwritten')

        #add factor 
        self.factors[name] = {
            "variables": variables,
            "lambda": Lambda,
            "eta": eta
        }
        # print('factor:',name)
        # print('lambda',Lambda)
        # print('eta:',eta)

    def init_msgs_to_0(self):
        # Initialize messages to zero (must be done once after all factors added)
        for f_name in self.factors.keys():
            for v in self.factors[f_name]["variables"]:
                self.messages_fv[(f_name, v)] = (0.0, 0.0) #messages_fv[(1,0)] -> factor 1 (f12) variable 0 (x1). Message is (lambda, eta)
                self.messages_vf[(v, f_name)] = (0.0, 0.0)

    def perform_GBP_centralised(self, iterations):
        def sum_messages(msgs): #helper function
            Lambda = sum(m[0] for m in msgs)
            eta = sum(m[1] for m in msgs)
            return Lambda, eta
        
        for it in range(iterations):

            # -------------------------
            # 1) Variable → Factor
            # -------------------------
            #TODO doesn't that also include factors not connected to this variable ?
            #Nvm, they might be always equal to 0 if not connected 
            for v in self.variables:
                for f_name in self.factors.keys():
                    incoming = []

                    if v not in self.factors[f_name]['variables']: #only consider variables attached to the factor
                        continue

                    for other_f_name in self.factors.keys():
                        if other_f_name != f_name:
                            if v not in self.factors[other_f_name]['variables']: #only consider variables attached to the factor
                                continue
                            incoming.append(self.messages_fv[(other_f_name, v)])

                    Lambda, eta = sum_messages(incoming)

                    self.messages_vf[(v, f_name)] = (Lambda, eta)

            # -------------------------
            # 2) Factor → Variable
            # -------------------------
            for f_name in self.factors.keys():
                f = self.factors[f_name]

                if len(self.factors[f_name]["variables"]) == 1:
                    v = self.factors[f_name]["variables"][0]
                    self.messages_fv[(f_name, v)] = (self.factors[f_name]["lambda"][0,0], self.factors[f_name]["eta"][0])

                else: #TODO hard coded scnerio for 1 var and 2 vars => need N vars
                    i, j = f["variables"]

                    Lambda = f["lambda"]
                    eta = f["eta"]

                    Lambda_ii, Lambda_ij = Lambda[0,0], Lambda[0,1]
                    Lambda_ji, Lambda_jj = Lambda[1,0], Lambda[1,1]
                    eta_i, eta_j = eta[0], eta[1]

                    # i → j
                    Lambda_i_cav, eta_i_cav = self.messages_vf[(i, f_name)]
                    denom = Lambda_i_cav + Lambda_ii

                    self.messages_fv[(f_name, j)] = (
                        Lambda_jj - (Lambda_ji**2)/denom,
                        eta_j - Lambda_ji*(eta_i_cav + eta_i)/denom
                    )

                    # j → i
                    Lambda_j_cav, eta_j_cav = self.messages_vf[(j, f_name)]
                    denom = Lambda_j_cav + Lambda_jj

                    self.messages_fv[(f_name, i)] = (
                        Lambda_ii - (Lambda_ij**2)/denom,
                        eta_i - Lambda_ij*(eta_j_cav + eta_j)/denom
                    )

            # -------------------------
            # 3) Belief Update 
            # -------------------------
            # print('self.messages_fv:', self.messages_fv)

            new_beliefs = []

            for v in self.variables:
                incoming = []

                for f_name in self.factors.keys():
                    if v in self.factors[f_name]["variables"]:
                        incoming.append(self.messages_fv[(f_name, v)])

                Lambda, eta = sum_messages(incoming)
                new_beliefs.append((Lambda, eta))

            beliefs = new_beliefs

            # print('beliefs:', beliefs)

            # (optional) print progress
            print(f"\nIteration {it+1}")
            for i, (Lambda, eta) in enumerate(beliefs):
                mean = eta / Lambda
                # print(f"x{i+1} ≈ {mean:.4f}")
                print(f"x{i+1} ≈ {mean}")

    def perform_GBP_local(self, node1, node2, iterations, threshold = 0.1): #where a node is a variable
        def sum_messages(msgs): #helper function
            Lambda = sum(m[0] for m in msgs)
            eta = sum(m[1] for m in msgs)
            return Lambda, eta
        
        old_beliefs = None
        
        for it in range(iterations):

            # -------------------------
            # 1) Variable → Factor
            # -------------------------
            for v in (node1, node2): #only go through the 2 given vars
                for f_name in self.factors.keys():
                    incoming = []

                    if v not in self.factors[f_name]['variables']: #only consider variables attached to the factor
                        continue

                    for other_f_name in self.factors.keys():
                        if other_f_name != f_name:
                            if v not in self.factors[other_f_name]['variables']: #only consider variables attached to the factor
                                continue
                            incoming.append(self.messages_fv[(other_f_name, v)])

                    Lambda, eta = sum_messages(incoming)

                    self.messages_vf[(v, f_name)] = (Lambda, eta)

            # -------------------------
            # 2) Factor → Variable
            # -------------------------
            for f_name in self.factors.keys():
                #only process factors that have a connection to either node
                if (not (node1 in self.factors[f_name]["variables"])) and (not (node2 in self.factors[f_name]["variables"])):
                    continue

                f = self.factors[f_name]

                if len(self.factors[f_name]["variables"]) == 1:
                    v = self.factors[f_name]["variables"][0]
                    self.messages_fv[(f_name, v)] = (self.factors[f_name]["lambda"][0,0], self.factors[f_name]["eta"][0])

                else: #TODO hard coded scnerio for 1 var and 2 vars => need N vars
                    i, j = f["variables"]

                    Lambda = f["lambda"]
                    eta = f["eta"]

                    Lambda_ii, Lambda_ij = Lambda[0,0], Lambda[0,1]
                    Lambda_ji, Lambda_jj = Lambda[1,0], Lambda[1,1]
                    eta_i, eta_j = eta[0], eta[1]

                    # i → j
                    Lambda_i_cav, eta_i_cav = self.messages_vf[(i, f_name)]
                    denom = Lambda_i_cav + Lambda_ii

                    self.messages_fv[(f_name, j)] = (
                        Lambda_jj - (Lambda_ji**2)/denom,
                        eta_j - Lambda_ji*(eta_i_cav + eta_i)/denom
                    )

                    # j → i
                    Lambda_j_cav, eta_j_cav = self.messages_vf[(j, f_name)]
                    denom = Lambda_j_cav + Lambda_jj

                    self.messages_fv[(f_name, i)] = (
                        Lambda_ii - (Lambda_ij**2)/denom,
                        eta_i - Lambda_ij*(eta_j_cav + eta_j)/denom
                    )

            new_beliefs = {}

            for v in (node1, node2):
                incoming = []

                for f_name in self.factors.keys():
                    #only process factors that have a connection to either node
                    if (not (node1 in self.factors[f_name]["variables"])) and (not (node2 in self.factors[f_name]["variables"])):
                        continue
                    if v in self.factors[f_name]["variables"]:
                        incoming.append(self.messages_fv[(f_name, v)])

                Lambda, eta = sum_messages(incoming)
                new_beliefs[v] = (Lambda, eta)

            beliefs = new_beliefs

            # print('beliefs:', beliefs)

            # Print everything
            # (optional) print progress
            print(f"\nIteration {it+1}")
            for var in beliefs.keys():
                Lambda, eta = beliefs[var]
                mean = eta / Lambda
                print(f"{var} ≈ {mean}")
        

            if not (old_beliefs is None):
                if abs(new_beliefs[node1][0] - old_beliefs[node1][0]) < threshold:
                    break
                if abs(new_beliefs[node2][0] - old_beliefs[node2][0]) < threshold:
                    break

            old_beliefs = new_beliefs




system = GBP(variables=['x1', 'x2', 'x3'])

noise_prior = 0.1
noise_distance = 0.1

d12 = 1.0
d23 = 1.0

system.add_factor(variables=['x1'],
                  eta=(1/noise_distance**2) *np.array([[5.0]]),
                  Lambda= (1/noise_prior**2) * np.array([[1]]),
                  name="prior_x1")

system.add_factor(variables=['x2'],
                  eta=(1/noise_distance**2) *np.array([[6.2]]),
                  Lambda= (1/noise_prior**2) * np.array([[1]]),
                  name="prior_x2")

system.add_factor(variables=['x1', 'x2'],
                  eta=(1/noise_distance**2) *np.array([[-d12], [d12]]),
                  Lambda= (1/noise_distance**2) * np.array([[1, -1],[-1, 1]]),
                  name="distance_x1_x2")

system.add_factor(variables=['x2', 'x3'],
                  eta= (1/noise_distance**2) *np.array([[-d23], [d23]]),
                  Lambda= (1/noise_distance**2) * np.array([[1, -1],[-1, 1]]),
                  name="distance_x2_x3")

system.init_msgs_to_0()

# system.perform_GBP_centralised(5)

system.perform_GBP_local('x2', 'x3', 5, threshold=0.01)
system.perform_GBP_local('x1', 'x2', 5, threshold=0.01)
system.perform_GBP_local('x2', 'x3', 5, threshold=0.01)






# Solution it should get:

# x1 ≈ [5.06666667]
# x2 ≈ [6.13333333]
# x3 ≈ [7.13333333]


        