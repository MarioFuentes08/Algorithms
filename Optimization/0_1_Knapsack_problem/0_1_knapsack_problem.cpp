#include<iostream>
#include<vector>
#include<algorithm>

using namespace std;
//Prototypes

int solve_knapsack(int W, const vector<int>& weights, const vector<int>& values, int N);



/*
Function to solve knapsack problem
*/
int solve_knapsack(int W, const vector<int>& weights, const vector<int>& values, int N){

    //Base case: There is no capacity and N is 0
    if(W == 0 || N == 0){
        return 0;
    }
    
    //Objetive function
    //Z = Sum{i=1}^{N} value_i * x_i

    //Subject to 
    //Sum{i=1}^{N} weight_i * x_i <= W
    //x_i in {0,1}, for all i = 1,...N

    int select_element = 0;
    //select the item if doesnt exceed the capacity
    if(weights[N-1] <= W){
        
        select_element = values[N-1] + solve_knapsack(W - weights[N-1], weights, values, N-1);
    }

    //If element was not selected, then the capacity "W" is the same and go for the next element
    int not_select = 0;
    not_select = solve_knapsack(W, weights, values, N-1);


    return max(select_element, not_select);
}



int main(){
    
    //Data
    vector<int> values = {100,45,50};
    vector<int> weights = {30, 10, 15};
    int capacity = 50;

    int N_weights = weights.size();
    int N_values = values.size();

    //Verifying size of weights and values. Must be not 0 and must be same
    if(N_weights == 0 || N_values == 0 || N_weights != N_values){
        return 0;
    }

    int result = solve_knapsack(capacity, weights, values, N_values);
    cout << "Maximum value: " << result <<endl;

    return 0;
}