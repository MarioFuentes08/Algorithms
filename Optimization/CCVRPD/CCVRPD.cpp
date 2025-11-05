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

using namespace std;

int main() {

    // Initialize the CPLEX environment
    IloEnv env;

    try {
        // =======================================================================
        // 1. DEFINE PROBLEM DATA (HARDCODING VALUES FOR TESTING)
        // =======================================================================
        
        // N' in the paper's text.
        int num_customers = 5; 
        
        /*
        There is a fleet of identical vehicles V available to serve
        the customers such that each of them is equipped with
        a drone
        */
        // V in the paper's text.
        int num_vehicles = 2;
        
        // D in the paper's text.
        int num_drones = 2; 


        // The set N = {0, 1, ..., N+1}
        // Set total_nodes as (N+2) to account for 0, (1...N), and (N+1)
        int total_nodes = num_customers + 2; // Total nodes = 7
        
        // Index for the start depot (Node "0")
        int depot_start = 0; 
        
        // Index for the end depot (Node "N+1")
        int depot_end = num_customers + 1; // 

        // --- Parameters ---
        
        // T: A large positive constant (for big-M constraints)
        IloNum T = 10000.0; 

        // Q: Vehicle's capacity
        IloNum Q = 100.0; 

        // E: Drone flight endurance
        IloNum E = 50.0; 

        // w_ij: Traveling time by vehicle (matrix)
        typedef IloArray<IloNumArray> NumMatrix;
        NumMatrix w(env, total_nodes);
        
        // W_ij: Traveling time by drone (matrix)
        NumMatrix W(env, total_nodes); 
        
        // q_i: Demand of customer i (array)
        IloNumArray q(env, total_nodes);

        // --- Harcoding values ---
        // This data is arbitrary, just for making the model runnable.
        cout << "Populating data..." << endl;
        for (int i = 0; i < total_nodes; i++) {
            // Initialize arrays for matrices
            w[i] = IloNumArray(env, total_nodes);
            W[i] = IloNumArray(env, total_nodes);

            // Set demand: 0 for depots, 10-15 for customers
            if (i == depot_start || i == depot_end) {
                q[i] = 0.0;
            } else {
                q[i] = 10.0 + (i % 5); // e.g., 10, 11, 12, 13, 14
            }

            // Populate travel time matrices
            for (int j = 0; j < total_nodes; j++) {
                if (i == j) {
                    w[i][j] = 0.0;  // Time to itself is 0
                    W[i][j] = 0.0;  // Time to itself is 0
                } else {
                    // Vehicle time (Euclidean distance)
                    // It is the traveling time between nodes i and j by the vehicle v
                    w[i][j] = (abs(i - j) + 1) * 2.0; 

                    // The drone moves at a higher speed than that of vehicles
                    // Drone time (e.g., 0.5x vehicle time)
                    // It is the traveling time between nodes i and j by the drone d
                    W[i][j] = (abs(i - j) + 1) * 1.0; 
                }
            }
        }
        cout << "Data population completed" << endl;


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

        // --- Constraints (1) & (2): Depot Start/End ---
        // Constraints (1) and (2) ensure that each route
        // should begin and end at the depot (node “0”) at most
        // once, respectively
        for (int v = 0; v < num_vehicles; v++) {
            IloExpr expr1(env); // For constraint (1)
            IloExpr expr2(env); // For constraint (2)
            
            // Sum of x[v][0][j] for j=1...N (customers)
            for (int j = 1; j <= num_customers; j++) {
                expr1 += x[v][depot_start][j];
            }
            // Sum of x[v][i][N+1] for i=1...N (customers)
            for (int i = 1; i <= num_customers; i++) {
                expr2 += x[v][i][depot_end];
            }

            model.add(expr1 <= 1); // Add constraint (1)
            model.add(expr2 <= 1); // Add constraint (2)
            expr1.end();
            expr2.end();
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
                for (int j = 0; j <= num_customers; j++) { 
                    if (i == j) continue;
                    expr5 += q[i] * x[v][i][j];
                }
            }

            // Term 2: sum(q_i * y_ijk^vd)

            for (int d = 0; d < num_drones; d++) {
                // Sum j=1...N (customers)
                for (int j = 1; j <= num_customers; j++) { 
                    // Sum i=0...N (depot 0 + customers), i != j
                    for (int i = 0; i <= num_customers; i++) { 
                        if (i == j) continue;
                        // Sum k=0...N (depot 0 + customers)
                        for (int k = 0; k <= num_customers; k++) { 
                            expr5 += q[i] * y[v][d][i][j][k];
                        }
                    }
                }
            }
            model.add(expr5 <= Q); // Add constraint (5)
            expr5.end();
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

        for (int v = 0; v < num_vehicles; v++) {
            for (int d = 0; d < num_drones; d++) {
                // i, k = 0...N (depot 0 + customers)
                for (int i = 0; i < total_nodes; i++) { 
                    for (int k = 0; k < total_nodes; k++) {
                        // j = 1...N (customers)
                        for (int j = 1; j <= num_customers; j++) { 
                            // Left: sum(x_il^v) + sum(x_mk^v)
                            IloExpr expr7_Left(env);
                            // Sum l=1...N (customers)
                            for (int l = 1; l <= num_customers; l++) {
                                expr7_Left += x[v][i][l];
                            }
                            // Sum m=0...N (depot 0 + customers)
                            for (int m = 0; m < total_nodes; m++) {
                                expr7_Left += x[v][m][k];
                            }
                            
                            // Right: 2 * y_ijk^vd
                            IloExpr expr7_Right(env);
                            expr7_Right = 2.0 * y[v][d][i][j][k];

                            model.add(expr7_Left <= expr7_Right); // Add constraint (7)
                            expr7_Left.end();
                            expr7_Right.end();
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
                // j = 1...N (customers)
                for (int j = 1; j <= num_customers; j++) { 
                    // t_i^v + w_ij - (1-x_ij^v)T <= t_j^v
                    model.add(t_v[v][i] + w[i][j] - (1.0 - x[v][i][j]) * T <= t_v[v][j]);
                }
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
                            
                            // t_i^v + w_ij * y_ijk^vd <= t_j^d
                            model.add(t_v[v][i] + w[i][j] * y[v][d][i][j][k] <= t_d[d][j]);
                        }
                    }
                }
            }
        }
        
        cout << "All constraints added." << endl;

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

            // --- Print the solution (non-zero variables) ---
            
            double tolerance = 0.001; // To avoid printing tiny floating point values

            cout << "Vehicle Routes (x_ij^v):" << endl;
            for (int v = 0; v < num_vehicles; v++) {
                for (int i = 0; i < total_nodes; i++) {
                    for (int j = 0; j < total_nodes; j++) {
                        try {
                            if (cplex.getValue(x[v][i][j]) > tolerance) {
                                cout << "  Vehicle " << v << ": " << i << " -> " << j;
                                // Only print time if j is NOT the end depot
                                if (j != depot_end) {
                                    cout << " (Arrival at " << j << ": " << cplex.getValue(t_v[v][j]) << ")";
                                }
                                cout << endl;
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
                                            cout << "  V" << v << ", D" << d << ": " 
                                                 << i << " -> " << j << " -> " << k
                                                 << " (Arrival at " << j << ": " << cplex.getValue(t_d[d][j]) << ")"
                                                 << endl;
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

    return 0; // End of main
}