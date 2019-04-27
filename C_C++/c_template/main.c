#include <stdio.h>
#include <time.h>
#include "main.h"

//延时函数1s
void delay(int seconds) {

    clock_t start = clock();
    clock_t lay = (clock_t)seconds * CLOCKS_PER_SEC;

    while((clock() - start) < lay) {
        ;
    }
}

int main()
{
    while(1) {
        printf("hello wolrd.\r\n");  
        delay(1);
    }
    

    getchar();


    return 0;
}