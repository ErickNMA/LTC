#include <stdio.h>
#include <stdlib.h>

//Ordem do sistema:
#define n 2

//Matriz de dinâmica do sistema:
float A[n][n] = {{-8.42, -21.4}, {1, 0}};

//Vetor coluna de demux do sinal:
float B[n][1] = {{1}, {0}};

//Vetor de mux da saída:
float C[n] = {0, 16.2};

//Vetor de ganhos do pi-like:
float k[n+1] = {63.02, 755.7828, -336.2326};

//Vetor coluna de realimentação do observador:
float L[n][1] = {{3239}, {103}};


//Definição de constantes e variáveis:
float dX[n][1], dXtil[n][1], dxa, e, etil;

int main()
{
    //Parâmetros da simulação:
    float ts = 1e-3;                // tempo de amostragem
    float tf = 2;                   // tempo final
    float X[n][1] = {{0}, {0}};     // valor inicial dos estados do sistema
    float xa = 0;                   // estado aumentado (pós-integrador)
    float Xtil[n][1] = {{0}, {0}};  // valor inicial dos estados do observador
    float t = 0;                    // valor inicial do tempo
    float y = 0;                    // valor inicial da saída
    float ytil = ((C[0]*Xtil[0][0])+(C[1]*Xtil[1][0]));                 // valor inicial da saída estimada
    float u = 0;                    // valor inicial do sinal de controle

    float ref = 50;                 // setpoint para o sistema

    //Loop infinito:
    do
    {
        //Print dos valores atuais:
        printf("\n \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f", t, u, y, Xtil[0][0], Xtil[1][0]);

        //Cálculo do erro de seguimento:
        e = (ref-y);

        //Cálculo do erro de estimação
        etil = (y-ytil);

        //Cálculo da derivada dos estados do sistema:
        for(int i=0; i<n; i++)
        {
            dX[i][0] = 0;
            for(int j=0; j<n; j++)
            {
                dX[i][0] += A[i][j]*X[j][0];
            }    
            dX[i][0] += B[i][0]*u;  
        }
        dxa = e;

        //Atualização dos estados do sistema (integração de Euler):
        for(int i=0; i<n; i++)
        {
            X[i][0] += dX[i][0]*ts;
        }
        xa += dxa*ts;

        //Cálculo da derivada dos estados do observador:
        for(int i=0; i<n; i++)
        {
            dXtil[i][0] = 0;
            for(int j=0; j<n; j++)
            {
                dXtil[i][0] += A[i][j]*Xtil[j][0];
            }    
            dXtil[i][0] += ((B[i][0]*u)+(L[i][0]*etil));  
        }

        //Atualização dos estados do observador (integração de Euler):
        for(int i=0; i<n; i++)
        {
            Xtil[i][0] += dXtil[i][0]*ts;
        }

        //Atualização do sinal de controle:
        u = ref;
        for(int i=0; i<n; i++)
        {
            u -= k[i]*X[i][0];
        }
        u += -k[n]*xa;

        //Atualização do valor da saída:
        y = 0;
        for(int i=0; i<n; i++)
        {
            y += C[i]*X[i][0];
        }

        //Atualização do valor da saída estimada:
        ytil = 0;
        for(int i=0; i<n; i++)
        {
            ytil += C[i]*Xtil[i][0];
        }

        //Atualização do tempo:
        t += ts;

    }while(t <= tf);

    return 0;
}