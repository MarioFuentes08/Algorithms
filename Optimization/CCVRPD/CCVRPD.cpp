/*
 * ===============================================================================
 * MILP Model for CCVRDP (Imen Hamdi, 18 Jan 2024)
 * This C++ implementation uses the CPLEX 22.1.1 to solve the mathematical, based 
 * on the paper "Solving the cumulative capacitated vehicle routing problem with drones"
 * by Imen Hamdi.
 * ===============================================================================
 */

// Main header file for CPLEX 
#include <ilcplex/ilocplex.h>
#include <fstream>
#include <cmath>
#include <sstream>   
#include <map>       
#include <vector>    
#include <stdexcept>

using namespace std;

int main(int argc, char* argv[]) {

    //Verify argument
    if (argc < 2) {
        
        cerr << "Error: VRP file name is not indicated." << endl;
        cerr << "Eg: .\\CCVRPD.exe ./Instances/P-n16-k8.vrp" << endl;
        return 1; // Retorna 1 para indicar un error
    }

    std::string vrp_filename = argv[1]; // .vrp file
    
    // Initialize the CPLEX environment
    IloEnv env;

    try {
        // =======================================================================
        // 1. DEFINE PROBLEM DATA (Reading from .vrp FILE)
        // =======================================================================
        
        cout << "Parsing .vrp file..." << endl;
        
        std::string line;
        std::string current_section;

        string instance_name = "";
        int file_dimension = 0;
        int file_capacity = 0;
        int file_depot_id = 0;
        int file_num_vehicles = 0; 

        std::map<int, std::pair<double, double>> temp_coords;
        std::map<int, double> temp_demand;

        // Read .vrp file

        std::ifstream file(vrp_filename);
        while (std::getline(file, line)) {

            std::stringstream ss(line);
            std::string keyword;

            ss >> keyword;

            if (keyword == "NAME") {
                ss.ignore(20, ':'); // Ignore " : "
                ss >> instance_name;
            }
            else if (keyword == "COMMENT") {
                // Number of trucks
                std::string temp;
                while (ss >> temp) {
                    if (temp == "trucks:") {
                        ss >> file_num_vehicles;
                        break;
                    }
                }
            }
            else if (keyword == "DIMENSION") {
                ss.ignore(20, ':'); // Ignore " : "
                ss >> file_dimension;
            }
            else if (keyword == "CAPACITY") {
                ss.ignore(20, ':'); // Ignore " : "
                ss >> file_capacity;
            }
            else if (keyword == "NODE_COORD_SECTION") {
                current_section = "COORDS";
            }
            else if (keyword == "DEMAND_SECTION") {
                current_section = "DEMAND";
            }
            else if (keyword == "DEPOT_SECTION") {
                current_section = "DEPOT";
            }
            else if (keyword == "EOF") {
                break; // Fin del archivo
            }
            //Process Data
            else if (current_section == "COORDS") {
                int id;
                double x, y;
                // 'keyword' with ID
                std::stringstream(keyword) >> id; 
                ss >> x >> y;
                temp_coords[id] = {x, y};
            }
            else if (current_section == "DEMAND") {
                int id;
                double demand;
                std::stringstream(keyword) >> id;
                ss >> demand;
                temp_demand[id] = demand;
            }
            else if (current_section == "DEPOT") {
                std::stringstream(keyword) >> file_depot_id;
                if (file_depot_id != -1) {
                    current_section = ""; 
                }
            }
        }
        file.close();

        // --- Map Data ---


        int num_customers = file_dimension - 1; //subtract the depot
        int total_nodes = num_customers + 2; // (N+2)
        int depot_start = 0;
        int depot_end = num_customers + 1; // (N+1)

        // Assign parameters

        int num_vehicles = file_num_vehicles;
        int num_drones = num_vehicles;                 // 1 dron per vehicle
        IloNum T = 10000.0;
        IloNum Q = file_capacity;                    // Read from vrp file
        IloNum Q_d = (uint16_t)(file_capacity/2);   // Drone capacity is considered as half that of the vehicle. No available in the vrp file
        IloNum E = 80.0;                            // Harcoded. No available in the vrp file
        double vehicle_speed = 35.0;                // Harcoded. No available in the vrp file
        double drone_speed = 50.0;                  // Harcoded. No available in the vrp file     


        // CPLEX structures
        typedef IloArray<IloNumArray> NumMatrix;
        NumMatrix w(env, total_nodes);
        NumMatrix W(env, total_nodes); 
        IloNumArray q(env, total_nodes);
        IloNumArray coord_x(env, total_nodes);
        IloNumArray coord_y(env, total_nodes);

        for (int i = 0; i < total_nodes; i++) {
            w[i] = IloNumArray(env, total_nodes);
            W[i] = IloNumArray(env, total_nodes);
        }

        //Mapping vrp data

        cout << "Mapping .vrp data to model structure..." << endl;

        // map depot (Node 'file_depot_id' of .vrp) to node 0 and N+1
        coord_x[depot_start] = temp_coords[file_depot_id].first;
        coord_y[depot_start] = temp_coords[file_depot_id].second;
        q[depot_start] = 0.0;

        coord_x[depot_end] = temp_coords[file_depot_id].first;
        coord_y[depot_end] = temp_coords[file_depot_id].second;
        q[depot_end] = 0.0;

        // Map costumers to the other nodes 
        int model_node_index = 1; // customer 1
        for (int vrp_node_id = 1; vrp_node_id <= file_dimension; vrp_node_id++) {
            // if the vrp node is not the depot, is a customer
            if (vrp_node_id != file_depot_id) {
                coord_x[model_node_index] = temp_coords[vrp_node_id].first;
                coord_y[model_node_index] = temp_coords[vrp_node_id].second;
                q[model_node_index] = temp_demand[vrp_node_id];
                model_node_index++;
            }
        }

        // --- Calculate Time Matrices from Coordinates ---

        for (int i = 0; i < total_nodes; i++) {
            for (int j = 0; j < total_nodes; j++) {
                if (i == j) {
                    // Time to the same node is 0
                    w[i][j] = 0.0; 
                    W[i][j] = 0.0; 
                } else {
                    //  Calculate Euclidean Distance
                    // dist = sqrt( (x_i - x_j)^2 + (y_i - y_j)^2 )
                    double delta_x = coord_x[i] - coord_x[j];
                    double delta_y = coord_y[i] - coord_y[j];
                    double distance = std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2));

                    // Calculate Time = Distance / Speed
                    // The model will now optimize based on these calculated times.
                    w[i][j] = distance / vehicle_speed;
                    W[i][j] = distance / drone_speed;
                }
            }
        }

        cout << "Data population completed. (N=" << num_customers << ", V=" << num_vehicles << ") " << "Instance: " << instance_name << endl;

        //This data will be used for plotting in python
        cout << "Exporting node data to node_data.csv..." << endl;

        // Open the output file
        std::ofstream node_file;
        node_file.open("node_data.csv");

        // write the CSV header
        node_file << "node_id,x_coord,y_coord,demand\n";
        
        // Write data for each node (0 to N+1)
        for (int i = 0; i < total_nodes; i++) {
            node_file << i << "," 
                      << coord_x[i] << "," 
                      << coord_y[i] << "," 
                      << q[i] << "\n";
        }
        
        node_file.close();

        // =======================================================================
        // 2. CREATE THE MILP MODEL
        // =======================================================================

        // Create the model object
        IloModel model(env);

        // =======================================================================
        // 3. DEFINE DECISION VARIABLES
        // =======================================================================

        // --- Typedefs for multi-dimensional arrays ---

        typedef IloArray<IloBoolVarArray> BoolVarMatrix;   // Matrix of boolean variables
        typedef IloArray<BoolVarMatrix>   BoolVarCube;     // For x[v][i][j] 
        typedef IloArray<BoolVarCube>     BoolVar4D;
        typedef IloArray<BoolVar4D>       BoolVar5D;       // For y[v][d][i][j][k]
        typedef IloArray<IloNumVarArray>  NumVarMatrix;    // For t_v[v][i]

        // --- x_ij^v ---
        // x[v][i][j] = 1 if the vehicle v moves from customer i to customer j, and 0 else
        BoolVarCube x(env, num_vehicles);
        for (int v = 0; v < num_vehicles; v++) {
            x[v] = BoolVarMatrix(env, total_nodes);
            for (int i = 0; i < total_nodes; i++) {
                // Defines a binary variable for each (i, j) pair
                x[v][i] = IloBoolVarArray(env, total_nodes); 
            }
        }

        // --- y_ijk^vd ---
        // y[v][d][i][j][k] = 1 if the drone d associated to the vehicle v leaves from i to serve j and meet v in k, and 0 else
        BoolVar5D y(env, num_vehicles);
        for (int v = 0; v < num_vehicles; v++) {
            y[v] = BoolVar4D(env, num_drones);
            for (int d = 0; d < num_drones; d++) {
                y[v][d] = BoolVarCube(env, total_nodes);
                for (int i = 0; i < total_nodes; i++) {
                    y[v][d][i] = BoolVarMatrix(env, total_nodes);
                    for (int j = 0; j < total_nodes; j++) {
                        // Defines a binary variable for each (i, j, k) triplet
                        y[v][d][i][j] = IloBoolVarArray(env, total_nodes);
                    }
                }
            }
        }

        // --- t_i^v ---
        // t_v[v][i] = It is the arrival time of the vehicle v to the node i
        NumVarMatrix t_v(env, num_vehicles);
        for (int v = 0; v < num_vehicles; v++) {
            // Continuous variable, non-negative (0.0) up to T
            t_v[v] = IloNumVarArray(env, total_nodes, 0.0, T, ILOFLOAT);
        }

        // --- t_i^d ---
        // t_d[d][i] = It is the arrival time of the drone d to the node i
        NumVarMatrix t_d(env, num_drones);
        for (int d = 0; d < num_drones; d++) {
            // Continuous variable, non-negative (0.0) up to T
            t_d[d] = IloNumVarArray(env, total_nodes, 0.0, T, ILOFLOAT);
        }


        // =======================================================================
        // 4. DEFINE THE OBJECTIVE FUNCTION
        // =======================================================================
        
        // Min Z = sum(t_i^v) + sum(t_i^d)
        
        // Create an expression object
        IloExpr objExpr(env);

        // Sum vehicle arrival times at customers (1 to N)
        // v and d starts from 0 due to it is zero based array, 
        // it means  v = 0 is the first vehicle and d = 0 is the first drone
        for (int v = 0; v < num_vehicles; v++) { 
            for (int i = 1; i <= num_customers; i++) {
                objExpr += t_v[v][i];
            }
        }

        // Sum drone arrival times at customers (1 to N)
        for (int d = 0; d < num_drones; d++) {
            for (int i = 1; i <= num_customers; i++) {
                objExpr += t_d[d][i];
            }
        }

        // Add the objective expression to the model (Minimization)
        model.add(IloMinimize(env, objExpr));

        // Clean up the expression object
        objExpr.end();


        // =======================================================================
        // 5. DEFINE THE CONSTRAINTS
        // =======================================================================
        cout << "Adding constraints..." << endl;

        // --- Missing constrain: Serve Each Customer Exactly Once ---
        /*
        Each customer must be served either by a drone or by a truck
        */
        // For each customer j (1...N)
        for (int j = 1; j <= num_customers; j++) {
            IloExpr expr_serve(env);

            // Term 1: Served by a vehicle
            // Sum over v, i
            for (int v = 0; v < num_vehicles; v++) {
                for (int i = 0; i < total_nodes; i++) {
                    if (i == j) continue; // Skip x[j][j]
                    expr_serve += x[v][i][j];
                }
            }

            // Term 2: Served by a drone
            // Sum over v, d, i, k
            for (int v = 0; v < num_vehicles; v++) {
                for (int d = 0; d < num_drones; d++) {
                    for (int i = 0; i < total_nodes; i++) {
                        //avoid launch and serve at the same node j
                        if (i == j) continue; 
                        
                        for (int k = 0; k < total_nodes; k++) {
                            //avoid serve and meet at the same node j
                            if (k == j) continue; 
                            //avoid serve and meet at the same node i
                            if (k == i) continue;
                            
                            expr_serve += y[v][d][i][j][k];
                        }
                    }
                }
            }
            
            // Force the model to serve the customer
            model.add(expr_serve == 1);
            expr_serve.end();
        }

        // --- Constraints (1) & (2): Depot Start/End ---
        // Constraints (1) and (2) ensure that each route
        // should begin and end at the depot (node “0”) at most
        // once, respectively
        for (int v = 0; v < num_vehicles; v++) {
            IloExpr total_starts(env); // For constraint (1)
            IloExpr total_ends(env); // For constraint (2)
            
            // Sum of x[v][0][j] for j=1...N (customers) outputs from start depot
            for (int j = 1; j <= num_customers; j++) {
                total_starts += x[v][depot_start][j];
            }
            // Sum of x[v][i][N+1] for i=1...N (customers) inputs to end depot
            for (int i = 1; i <= num_customers; i++) {
                total_ends += x[v][i][depot_end];
            }

            model.add(total_starts <= 1); 

            //Number of input and outputs must be same
            model.add(total_starts == total_ends);

            total_starts.end();
            total_ends.end();
        }

        //--- Prohibit start depot to end depot and vice versa ---

        for(int v = 0; v < num_vehicles; v++){
            model.add(x[v][depot_start][depot_end] == 0);
        }

        for(int v = 0; v < num_vehicles; v++){
            model.add(x[v][depot_end][depot_start] == 0);
        }

        // --- Constraint (3): Drone leaves customer i at most once ---
        // For each drone d, For each customer i (1..N)
        for (int d = 0; d < num_drones; d++) {
            for (int i = 1; i <= num_customers; i++) {
                IloExpr expr3(env);
                // Sum over v, j, k
                for (int v = 0; v < num_vehicles; v++) {
                    // Sum j=1...N (customers)
                    for (int j = 1; j <= num_customers; j++) { 
                        // Sum k=0...N (depot 0 + customers)
                        for (int k = 0; k <= num_customers; k++) { 
                            expr3 += y[v][d][i][j][k];
                        }
                    }
                }
                model.add(expr3 <= 1); // Add constraint (3)
                expr3.end();
            }
        }

        // --- Constraint (4): Drone enter at customer k at most once ---
        // For each drone d, For each customer k (1..N)
        for (int d = 0; d < num_drones; d++) {
            for (int k = 1; k <= num_customers; k++) {
                IloExpr expr4(env);
                // Sum over v, i, j
                for (int v = 0; v < num_vehicles; v++) {
                    // Sum i=0...N (depot 0 + customers)
                    for (int i = 0; i <= num_customers; i++) { 
                        // Sum j=1...N (customers), j != i
                        for (int j = 1; j <= num_customers; j++) { 
                            if (j == i) continue; // Skip j==i
                            expr4 += y[v][d][i][j][k];
                        }
                    }
                }
                model.add(expr4 <= 1); // Add constraint (4)
                expr4.end();
            }
        }

        // --- Constraint (5): Vehicle Capacity ---
        // For each vehicle v
        //state that the total load of each vehicle must not exceed its capacity
        for (int v = 0; v < num_vehicles; v++) {
            IloExpr expr5(env);
            
            // Term 1: sum(q_i * x_ij^v)
            // Sum i=1...N (customers)
            for (int i = 1; i <= num_customers; i++) { 
                // Sum j=0...N (depot 0 + customers), j != i
                for (int j = 0; j < total_nodes; j++) { 
                    if (i == j) continue;
                    expr5 += q[i] * x[v][i][j];
                }
            }

            // Term 2: sum(q_i * y_ijk^vd)

            for (int d = 0; d < num_drones; d++) {
                // Sum j=1...N (customers)
                for (int j = 1; j <= num_customers; j++) { 
                    // Sum i=0...N (depot 0 + customers), i != j
                    for (int i = 0; i < total_nodes; i++) { 
                        if (i == j) continue;
                        // Sum k=0...N (depot 0 + customers)
                        for (int k = 0; k < total_nodes; k++) { 
                            expr5 += q[j] * y[v][d][i][j][k];
                        }
                    }
                }
            }
            model.add(expr5 <= Q); // Add constraint (5)
            expr5.end();
        }

        // --- Constraint (5_2): Drone Capacity ---
        // For each drone d
        //state that the total load of each drone must not exceed its capacity
        for (int v = 0; v < num_vehicles; v++) {
            for (int d = 0; d < num_drones; d++) {
                for (int i = 0; i < total_nodes; i++) {
                    for (int j = 1; j <= num_customers; j++) { // j is a customer
                        
                        // Drone can not be launched from the node which is served
                        if (i == j) continue;

                        for (int k = 0; k < total_nodes; k++) {
                            // Drone can not meet v in the same node j 
                            if (k == j) continue;
                            
                            // Drone can not meet v in the same node i 
                            if (k == i) continue;

                            model.add((q[j] * y[v][d][i][j][k]) <= Q_d);
                        }
                    }
                }
            }
        }

        // --- Constraint (6): Drone Endurance ---
        // ensure that the battery endurance (E) of each drone should be respected. 
        for (int v = 0; v < num_vehicles; v++) {
            for (int d = 0; d < num_drones; d++) {
                IloExpr expr6(env);
                // Sum over i, j, k
                // Sum i=0...N+1 (all nodes)
                for (int i = 0; i < total_nodes; i++) { 
                    // Sum j=1...N (customers), j != i
                    for (int j = 1; j <= num_customers; j++) { 
                        if (i == j) continue;
                        // Sum k=0...N+1 (all nodes)
                        for (int k = 0; k < total_nodes; k++) { 
                            expr6 += (W[i][j] + W[j][k]) * y[v][d][i][j][k];
                        }
                    }
                }
                model.add(expr6 <= E); // Add constraint (6)
                expr6.end();
            }
        }

        // --- Constraint (7): Coupling constraint ---
        // For each v, d, i, j, k
        // explain
        // the relationship between the decision variables which
        // state that if a drone starts from a customer i and goes to
        // customer k, they must be visited by a vehicle.
        //It is not allowed for new launches to occur before recovering the previous drone in the same route

        for (int v = 0; v < num_vehicles; v++) {
            
            // i, k = 0...N (depot 0 + customers)
            for (int i = 0; i < total_nodes; i++) { 
                for (int k = 0; k < total_nodes; k++) {

                    if (i == k) continue;

                    int d_assigned = v;


                    IloExpr drone_mission_sum(env);

                    // j = 1...N (customers)
                    for (int j = 1; j <= num_customers; j++){
                        //Drone can not be launched from node i and serve the same node j
                        if (j == i || j == k) continue;

                        drone_mission_sum += y[v][d_assigned][i][j][k];
                    }

                    model.add(drone_mission_sum <= x[v][i][k]);
                    drone_mission_sum.end();

                    //These drones can not be used by this vehicle
                    for (int d_other = 0; d_other < num_drones; d_other++) {

                        if (d_other == v) continue;

                        for (int j = 1; j <= num_customers; j++) {
                            if (j == i || j == k) continue;

                            //Mission impossible, eg V0 using D1
                            model.add(y[v][d_other][i][j][k] == 0);
                        }
                    }
                }
            }
        }
            
        // --- Constraints (8) & (9): Time Initialization ---
        /*
        Constraints (8) and (9) provide the initialization times
        for the truck and the drone, respectively, at the beginning of each route
        */
        // t_0^v = 0
        for (int v = 0; v < num_vehicles; v++) {
            model.add(t_v[v][depot_start] == 0);
        }
        // t_0^d = 0
        for (int d = 0; d < num_drones; d++) {
            model.add(t_d[d][depot_start] == 0);
        }

        // --- Constraint (10): Vehicle Time & Subtour Elimination ---
        /*
        Constraints (10) find the arrival time
        at each customer visited by a vehicle and prevent subtours where T is defined as a large positive constant.
        */
        // For each v, i, j
        for (int v = 0; v < num_vehicles; v++) {
            // i = 0...N (depot 0 + customers)
            for (int i = 0; i < total_nodes; i++) { 
                // j = 1...N+1 (customers)
                for (int j = 1; j < total_nodes; j++) { 

                    if (i==j) continue;
                    // t_i^v + w_ij - (1-x_ij^v)T <= t_j^v
                    model.add((t_v[v][i] + w[i][j] - (1.0 - x[v][i][j]) * T) <= t_v[v][j]);
                }
            }
        }


        for (int v = 0; v < num_vehicles; v++) {
            // h is a client from 1 to N
            // Depots 0 and N+1 does not need  to conserve flow 
            for (int h = 1; h <= num_customers; h++) {
                
                IloExpr flow_in(env);  
                IloExpr flow_out(env); 

                //Input
                for (int i = 0; i < total_nodes; i++) {
                    if (i == h) continue; // avoid i = h
                    flow_in += x[v][i][h];
                }

                //Output
                for (int j = 0; j < total_nodes; j++) {
                    if (j == h) continue; // avoid j = h
                    flow_out += x[v][h][j];
                }

                // Input flow must be equal to output flow
                model.add(flow_in == flow_out);
                flow_in.end();
                flow_out.end();
            }
        }        
        
        // --- Constraint (11): Drone Arrival Time ---
        /*
        Constraints (11) find the arrival time of each customer visited by a drone. 
        */
        // For each v, d, i, j, k
        for (int v = 0; v < num_vehicles; v++) {
            for (int d = 0; d < num_drones; d++) {
                // i = 0...N (depot 0 + customers)
                for (int i = 0; i < total_nodes; i++) {
                    // j = 1...N (customers), j != i
                    for (int j = 1; j <= num_customers; j++) {
                        if (i == j) continue;
                        // k = 0...N (depot 0 + customers), k != i, k != j
                        for (int k = 0; k < total_nodes; k++) {
                            if (k == i || k == j) continue;
                            
                            // t_i^v + W_ij  - (1 - y_ijk^vd)T <= t_j^d
                            model.add((t_v[v][i] + W[i][j] - (1- y[v][d][i][j][k])* T) <= t_d[d][j]);
                        }
                    }
                }
            }
        }


        // --- Constraint: Seal start depot, outflow only ---
        for (int v = 0; v < num_vehicles; v++) {
            IloExpr flow_into_start(env);
            
            // Sum all arcs x[v][i][0] where i is any node excetp 0
            for (int i = 1; i < total_nodes; i++) { 

                flow_into_start += x[v][i][depot_start];
            }

            // Force the total flow into the start depot to be 0
            model.add(flow_into_start == 0);
            flow_into_start.end();
        }

        // --- Constraint: Seal end depot, inflow only ---

        for (int v = 0; v < num_vehicles; v++) {

            IloExpr flow_out_of_end(env);
            
            // Sum all arcs x[v][6][j] where j is any node excetp 0
            for (int j = 0; j < total_nodes; j++) {
                // Skip j == depot_end
                if (j == depot_end) continue; 
                
                flow_out_of_end += x[v][depot_end][j];
            }

            // Force the total flow out of the end depot to be 0
            model.add(flow_out_of_end == 0);
            flow_out_of_end.end();
        }        

        // --- Constraint: Truck can not wait at the same location for the drone ---
        for (int v = 0; v < num_vehicles; v++) {
            for(int d = 0; d < num_drones; d++){

                for(int i = 0; i <total_nodes; i++){

                    for(int j = 1; j<=num_customers; j++){

                        if(i == j) continue;

                        for(int k = 0; k < total_nodes; k++){

                            if((i == k)||(j == k)) continue;

                            model.add(w[i][k] <= ( (W[i][j] + W[j][k]) + (1-y[v][d][i][j][k])*T ) );
                        }

                    }

                }

            }
        }

        // --- Constrain: It is not allowed for the drone to start the operation from the depot, 
        //     serve the costumer, and then come back to the depot, otherwise operate independently ---
        
        for (int v = 0; v < num_vehicles; v++) {
            for(int d = 0; d < num_drones; d++){

                for(int j = 1; j<=num_customers; j++){

                    model.add(y[v][d][depot_start][j][depot_start] == 0);
                    model.add(y[v][d][depot_start][j][depot_end] == 0);

                    model.add(y[v][d][depot_end][j][depot_start] == 0);
                    model.add(y[v][d][depot_end][j][depot_end] == 0);

                }

            }

        }

        // =======================================================================
        // 6. SOLVE THE MODEL
        // =======================================================================

        // Create the Cplex solver object
        IloCplex cplex(model);

        // Optional: Set a time limit (e.g., 300 seconds)
        //cplex.setParam(IloCplex::Param::TimeLimit, 300);

        // Solve the model
        cout << "Solving model..." << endl;
        if (cplex.solve()) {
            // If a solution is found
            cout << "----------------------------------------" << endl;
            cout << "Solution status: " << cplex.getStatus() << endl;
            cout << "Objective value: " << cplex.getObjValue() << endl;
            cout << "----------------------------------------" << endl;

            double optimal_value = cplex.getObjValue();
            std::ofstream summary_file;
            summary_file.open("solution_summary.csv");
            summary_file << "objective_value\n"; 
            summary_file << optimal_value << "\n";      
            summary_file.close();
            
            // --- Output files ---
            std::ofstream vehicle_file;
            std::ofstream drone_file;

            // Create files to save the routes
            vehicle_file.open("vehicle_routes.csv");
            drone_file.open("drone_missions.csv");

            // Headers
            vehicle_file << "vehicle_id,from_node,to_node,arrival_at_to_node\n";
            drone_file << "vehicle_id,drone_id,from_node,served_node,to_node,arrival_at_served_node\n";

            // --- Print the solution (non-zero variables) ---
            double tolerance = 0.001; // To avoid printing tiny floating point values

            cout << "Vehicle Routes (x_ij^v):" << endl;
            for (int v = 0; v < num_vehicles; v++) {
                for (int i = 0; i < total_nodes; i++) {
                    for (int j = 0; j < total_nodes; j++) {
                        try {
                            if (cplex.getValue(x[v][i][j]) > tolerance) {
                                cout << "  Vehicle " << v << ": " << i << " -> " << j;
                                double arrival_time = -1.0; // default value for end depot, there is no reported time for this depot
                                // Only print time if j is NOT the end depot
                                if (j != depot_end) {
                                    arrival_time = cplex.getValue(t_v[v][j]);
                                    cout << " (Arrival at " << j << ": " << arrival_time << ")";
                                }
                                cout << endl;

                                vehicle_file << v << "," << i << "," << j << "," << arrival_time << "\n";
                            }
                        } catch (IloException& e) {
                            // Catch error if variable was eliminated by presolve
                            // Do nothing, just skip this variable
                        }
                    }
                }
            }

            cout << "\nDrone Missions (y_ijk^vd):" << endl;
            for (int v = 0; v < num_vehicles; v++) {
                for (int d = 0; d < num_drones; d++) {
                    for (int i = 0; i < total_nodes; i++) {
                        for (int j = 0; j < total_nodes; j++) {
                            for (int k = 0; k < total_nodes; k++) {
                                try {
                                    if (cplex.getValue(y[v][d][i][j][k]) > tolerance) {
                                        // Only prints a valid mission
                                        
                                        if (i == j || k == j || k == i) {
                                            // Do nothing
                                        } else {
                                            // Valid mission
                                            double arrival_time = cplex.getValue(t_d[d][j]);

                                            cout << "  V" << v << ", D" << d << ": " 
                                                 << i << " -> " << j << " -> " << k
                                                 << " (Arrival at " << j << ": " << arrival_time << ")"
                                                 << endl;

                                            drone_file << v << "," << d << "," << i << "," << j << "," << k << "," << arrival_time << "\n";
                                        }
                                    }
                                } catch (IloException& e) {
                                    // Catch error if variable was eliminated by presolve
                                    // Do nothing, just skip this variable
                                }
                            }
                        }
                    }
                }
            }

            // Close csv files
            vehicle_file.close();
            drone_file.close();
            cout << "\nSolution exported to vehicle_routes.csv and drone_missions.csv" << endl;    

        } else {
            // If no solution is found
            cout << "----------------------------------------" << endl;
            cout << "No solution found." << endl;
            cout << "Solution status: " << cplex.getStatus() << endl;
            cout << "----------------------------------------" << endl;
        }

    } catch (IloException& e) {
        // Handle CPLEX exceptions
        cerr << "Concert exception caught: " << e << endl;
    } catch (...) {
        // Handle other exceptions
        cerr << "Unknown exception caught" << endl;
    }

    // Clean up the environment
    // This frees all memory allocated by Ceplex
    env.end();

    return 0; 
}