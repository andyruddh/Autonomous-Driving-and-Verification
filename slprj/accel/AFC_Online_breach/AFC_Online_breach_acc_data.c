#include "__cf_AFC_Online_breach.h"
#include "AFC_Online_breach_acc.h"
#include "AFC_Online_breach_acc_private.h"
P_AFC_Online_breach_T AFC_Online_breach_rtDefaultP = { 0.001 , 0.0 , 1.0 ,
1.0 , 0.01 , 50.0 , 0.01 , 0.982 , 1100.0 , 900.0 , 0.10471975511965977 , -
10.0 , 10.0 , 8.8 , 90.0 , 0.0 , 1.0 , 1.0 , 0.5 , 9.5492965855137211 ,
3.1415926535897931 , { 0.8 , 0.7 , 0.7 , 0.8 , 0.9 , 0.7 , 0.66 , 0.65 , 0.73
, 0.85 , 0.66 , 0.66 , 0.63 , 0.66 , 0.8 , 0.6 , 0.6 , 0.6 , 0.6 , 0.7 } , {
1000.0 , 1500.0 , 2000.0 , 2500.0 , 3000.0 } , { 0.1 , 0.2 , 0.3 , 0.4 } ,
1.0 , 1.0 , - 1.0 , 50.0 , 0.0 , 1.0 , 14.7 , 1.0 , 0.5 , 1.0 , 1.0 , 0.0112
, { 0.4 , 0.3 , 0.35 , 0.3 , 0.2 , 0.22 , 0.22 , 0.4 , 0.35 , 0.5 , 0.2 ,
0.22 , 0.5 , 0.4 , 0.35 , 0.35 , 0.3 , 0.45 , 0.5 , 0.4 } , { 1000.0 , 1500.0
, 2000.0 , 2500.0 , 3000.0 } , { 0.1 , 0.2 , 0.3 , 0.4 } , 1.0 , 14.7 , 50.0
, 10.0 , 14.7 , 10.0 , 9.5492965855137211 , { 0.8 , 0.6 , 0.4 , 0.3 , 0.2 ,
0.4 , 0.3 , 0.2 , 0.2 , 0.2 , 0.3 , 0.25 , 0.2 , 0.2 , 0.2 , 0.25 , 0.2 , 0.2
, 0.2 , 0.2 } , { 800.0 , 1000.0 , 1500.0 , 2000.0 , 3000.0 } , { 0.05 , 0.15
, 0.2 , 0.25 } , 0.41328 , - 1.0 , 1.0 , { 1.0 , 8.0 } , { 65.0 , 70.0 , 44.0
, 65.0 , 70.0 , 114.0 , 101.0 , 102.0 } , { 1.0 , 87.0 } , { 97.0 , 108.0 ,
119.0 , 95.0 , 91.0 , 49.0 , 48.0 , 44.0 , 32.0 , 51.0 , 48.0 , 93.0 , 32.0 ,
40.0 , 40.0 , 97.0 , 98.0 , 115.0 , 40.0 , 65.0 , 70.0 , 91.0 , 116.0 , 93.0
, 45.0 , 65.0 , 70.0 , 114.0 , 101.0 , 102.0 , 91.0 , 116.0 , 93.0 , 41.0 ,
32.0 , 62.0 , 32.0 , 48.0 , 46.0 , 48.0 , 53.0 , 41.0 , 32.0 , 61.0 , 62.0 ,
32.0 , 40.0 , 101.0 , 118.0 , 95.0 , 91.0 , 48.0 , 44.0 , 32.0 , 49.0 , 93.0
, 32.0 , 40.0 , 97.0 , 98.0 , 115.0 , 40.0 , 65.0 , 70.0 , 91.0 , 116.0 ,
93.0 , 45.0 , 65.0 , 70.0 , 114.0 , 101.0 , 102.0 , 91.0 , 116.0 , 93.0 ,
41.0 , 32.0 , 60.0 , 32.0 , 48.0 , 46.0 , 48.0 , 53.0 , 41.0 , 41.0 , 41.0 }
, { 1.0 , 1.0 } , 0.5 , { 1.0 , 1.0 } , 0.1 , { 1.0 , 8.0 } , { 65.0 , 70.0 ,
44.0 , 65.0 , 70.0 , 114.0 , 101.0 , 102.0 } , { 1.0 , 87.0 } , { 97.0 ,
108.0 , 119.0 , 95.0 , 91.0 , 49.0 , 48.0 , 44.0 , 32.0 , 51.0 , 48.0 , 93.0
, 32.0 , 40.0 , 40.0 , 97.0 , 98.0 , 115.0 , 40.0 , 65.0 , 70.0 , 91.0 ,
116.0 , 93.0 , 45.0 , 65.0 , 70.0 , 114.0 , 101.0 , 102.0 , 91.0 , 116.0 ,
93.0 , 41.0 , 32.0 , 62.0 , 32.0 , 48.0 , 46.0 , 48.0 , 53.0 , 41.0 , 32.0 ,
61.0 , 62.0 , 32.0 , 40.0 , 101.0 , 118.0 , 95.0 , 91.0 , 48.0 , 44.0 , 32.0
, 49.0 , 93.0 , 32.0 , 40.0 , 97.0 , 98.0 , 115.0 , 40.0 , 65.0 , 70.0 , 91.0
, 116.0 , 93.0 , 45.0 , 65.0 , 70.0 , 114.0 , 101.0 , 102.0 , 91.0 , 116.0 ,
93.0 , 41.0 , 32.0 , 60.0 , 32.0 , 48.0 , 46.0 , 48.0 , 53.0 , 41.0 , 41.0 ,
41.0 } , { 1.0 , 1.0 } , 100.0 , { 1.0 , 1.0 } , 0.1 , 0.0 , 0.0 , 0.0 , 0.0
, 0.0F , 0.0F , 0.01F , - 0.366F , 0.08979F , - 0.0337F , 0.0001F , 0.982F ,
0.41328F , 0.01F , 0.04F , 0.14F , 0.0F , 1.0F , 1.0F , 1.66F , 0.13F , 14.7F
, 12.5F , 0.0F , 0.01F , 10.0F , 50.0F , 20.0F , - 1.0F , 1.0F , 0.1726F ,
14.7F , 0.0F , 0.0F , 0.0F , 0.0F , 0.1726F , 0.0F , 1.0F , { 4U , 3U } , {
4U , 3U } , { 4U , 3U } , 0 , 0 , 0 , 0 } ;
