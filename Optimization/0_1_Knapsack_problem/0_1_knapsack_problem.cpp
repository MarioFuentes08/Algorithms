#include<iostream>
#include<vector>
#include<algorithm>

using namespace std;

/*
Function to solve knapsack problem
*/
int solve_knapsack(int capacity, const vector<int>& weights, const vector<int>& values, int N){

    //Dynamic programming table
    /*
    Creating a 2D matrix
    Rows: n+1
    Each row is a vector<int>
    Columns: capacity+1 
    Initializated with zeros
    */
    vector<vector<int>> dp(N+1, vector<int>(capacity + 1, 0));

    //Fill table
    for(int i = 1; i <= N; i++){
        for(int j = 0; j <= capacity; j++){
            //If the current object weighs more than the current capacity, we cannot include it
            if(weights[i-1] > j){
                dp[i][j] = dp[i-1][j];
            }
            else{
                //Maximum between not including the object or including it
                dp[i][j] = max(dp[i-1][j], values[i-1]+dp[i-1][j-weights[i-1]]);
            }

        }
    }

    return dp[N][capacity];
}



int main(){
    
    //Data
    vector<int> values = {60,100,120};
    vector<int> weights = {10, 20, 30};
    int capacity = 50;
    int N = values.size();


    return 0;
}