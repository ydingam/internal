//
#include"punch.h"
 bool punch(bool punch){
   while(ture){
      if(punch){
        set_pad();//high voltage
        punch=!punch;
      }else
        clear_pad();//low voltage
    }
 }

