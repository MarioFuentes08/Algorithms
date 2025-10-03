#include<iostream>
#include<vector>
#include<algorithm>

using namespace std;


struct Item {
    double value;
    double weight;
};


//Prototypes
double solve_fractional_knapsack(double W, vector<Item> &items, int N);
bool compare(const Item& a, const Item& b );


/*
Funcion to compare a and b element
Returns true if ratio of a is greater than b ratio
*/
bool compare(const Item& a, const Item& b ){
    return (a.value / a.weight) > (b.value / b.weight);
}


/*
Function to solve knapsack problem
*/
double solve_fractional_knapsack(double W, vector<Item> &items, int N){

    //Sort items based on value-to-weight  in descending order 
    //compare will return TRUE if the first element must be placed first before the second element
    sort(items.begin(), items.end(), compare);

    double remaining = W;
    double total_value = 0;
    
    
    //Objetive function
    //Z = Sum{i=1}^{N} value_i * x_i

    //Subject to 
    //Sum{i=1}^{N} weight_i * x_i <= W
    //0 <= x_i <= 1, for all i = 1,...N
 
    for(const auto &it : items){
        
        if(remaining <= 0) break; //knapsack is full
        
        if(it.weight <= remaining){
            //Take the whole object
            total_value += it.value;
            remaining -= it.weight;
        }
        else{
            //Take fractional part
            double fraction = remaining / it.weight;
            total_value += it.value * fraction;
            remaining = 0; //knapsack is full
        }

    }

    return total_value;
}

int main(){
    
    int n =  3;
    double capacity = 55;
    //Data
    vector<Item> items;
    items.resize(n);

    //hardcode values

    items[0].value = 20;
    items[0].weight = 32.5;

    items[1].value = 100;
    items[1].weight = 18;

    items[2].value = 50;
    items[2].weight = 20;

    
    double result = solve_fractional_knapsack(capacity, items,  n);
    cout << "Maximum value: " << result <<endl;

    return 0;
}