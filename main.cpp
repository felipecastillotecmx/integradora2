#include <iostream>
#include <string>

using namespace std;    
long long prhf(string S){
    int n = S.length();
    int p = 31;
    int m = 1e9 + 9;
    long long valorHash = 0;
    long long potencia = 1;
    for (int i = 0; i < n; i++){
        valorHash = (valorHash + (S[i] - 'a' + 1) * potencia) % m;
        potencia = (potencia * p) % m;
    }
    return valorHash;
}
int main(){
    string S;
    getline(cin, S);  
    cout << prhf(S) << endl;
    return 0;
}
