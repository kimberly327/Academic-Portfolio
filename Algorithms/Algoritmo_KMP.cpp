#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;

#define total_chars 256


//metodo con algoritmo naive
void naive_string_matcher(string T, string P){

	int n = T.length();
	int m = P.length();
	bool idem = false;
	int i, tmp;
	for(i = 0; i < n-m+1; i++){
		idem = true;
		tmp=i;

		for(int j=0; j<m && T[tmp]==P[j]; j++)
			tmp++;
		if(tmp-i==m)
			cout << "occorrenza alla posizione" << tmp-m << endl;
	}
}

//costruzione dell'automa a stati finiti
int calcola_nextstate(string P, int M, int stato, int x) {

    if (stato < M && x == P[stato])
        return stato+1;

    for (int ns = stato; ns > 0; ns--) {
        if (P[ns-1] == x) {
            for (int i = 0; i < ns-1; i++){
                if (P[i] != P[stato-ns+1+i]) break;
                if (i == ns-1) return ns;
            }  
        }
    }

    return 0;
}

void calcola_TF(string P, int M, int TF[][total_chars]) {

    for (int stato = 0; stato <= M; ++stato)
        for (int x = 0; x < total_chars; ++x)
            TF[stato][x] = calcola_nextstate(P, M, stato, x);
}

//ricerca delle occorrenze nel pattern
void calcola_occorrenza(string T, string P) {

	int n = T.length();
	int m = P.length();
	int TF[m+1][total_chars];
	calcola_TF(P, m, TF);
	int stato = 0;

	for (int i = 0; i < n; i++){
	   	stato = TF[stato][T[i]];
	    if (stato == m)
	        cout << "occorrenza alla posizione" << i-m+1 << endl;
	}
}

//Algoritmo di Knutt-Morris-Pratt con funzione prefisso
int* compute_prefix_function(string P, int dim){

	int* H = new int[dim];
	H[0] = 0;
	int k = 0;

	for(int q = 1; q < dim; q++){
		while(k > 0 && P[k] != P[q])
			k = H[k-1];
		if(P[k] == P[q])
			k++;
		H[q] = k;
	}

	return H;
}

void KMP_matcher(string T, string P){

	int n = T.length();
	int m = P.length();
	int *H = compute_prefix_function(P, m);
	int q = 0;

	for(int i = 0; i < n; i++){
		while(q > 0 && T[i] != P[q])
			q = H[q-1];
		if(T[i] == P[q])
			q++;
		if(q == m){
			cout << "occorrenza alla posizione" << i-m+1 << endl;
			q = H[q-1];
		}
	}
}

int main() {

    string Text = "AABCDAABBDCAABADAABDABAABA";
    string Pattern = "AABA";

    cout << endl;
    cout << "ricerca di matching con l'algoritmo naive" << endl;
    naive_string_matcher(Text, Pattern);
    cout << endl;
    cout << "ricerca di matching con gli automi a stati finiti" << endl;
    calcola_occorrenza(Text, Pattern);
    cout << endl;
    cout << "ricerca di matching con l'algoritmo KMP" << endl;
    KMP_matcher(Text, Pattern);
    cout << endl;
   
    return 0;
}